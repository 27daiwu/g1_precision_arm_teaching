"""Replay 1.0 offline foundation. No SDK imports, publishers or hardware entry."""
from dataclasses import dataclass
import json
from pathlib import Path
import zipfile
import numpy as np
from .storage import load_trajectory


def load_replay_trajectory(path, max_samples=30000):
    """Preflight bounded artifacts before the compatibility reader allocates."""
    path = Path(path)
    metadata_path = path.with_suffix('.json')
    if metadata_path.stat().st_size > 1024*1024:
        raise ValueError('metadata file too large')
    with metadata_path.open(encoding='utf-8') as stream:
        meta = json.load(stream)
    if not isinstance(meta, dict):
        raise ValueError('metadata must be object')
    if type(meta.get('sample_count')) is not int or not 2 <= meta['sample_count'] <= max_samples:
        raise ValueError('metadata sample count')
    if path.with_suffix('.npz').stat().st_size > 32*1024*1024:
        raise ValueError('NPZ file too large')
    with zipfile.ZipFile(path.with_suffix('.npz')) as archive:
        if len(archive.infolist()) > 16 or sum(info.file_size for info in archive.infolist()) > 32*1024*1024:
            raise ValueError('NPZ expanded size exceeds budget')
    try:
        return load_trajectory(path)
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError('incomplete trajectory artifact') from exc


@dataclass(frozen=True)
class ReplayLimits:
    q_min: tuple
    q_max: tuple
    velocity: float
    acceleration: float
    step: float
    gap: float
    start_error: float
    tracking_error: float
    state_timeout: float
    cycle_timeout: float
    max_duration: float
    start_velocity: float
    max_samples: int = 30000

    def __post_init__(self):
        lo, hi = np.asarray(self.q_min,dtype=float), np.asarray(self.q_max,dtype=float)
        if lo.shape != (14,) or hi.shape != (14,) or not np.isfinite([lo, hi]).all() or np.any(lo >= hi):
            raise ValueError('invalid explicit joint limits')
        object.__setattr__(self,'q_min',tuple(float(x) for x in lo))
        object.__setattr__(self,'q_max',tuple(float(x) for x in hi))
        for key in ('velocity', 'acceleration', 'step', 'gap', 'start_error',
                    'tracking_error', 'state_timeout', 'cycle_timeout', 'max_duration', 'start_velocity'):
            if not np.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(f'invalid {key}')
        if not 2 <= self.max_samples <= 30000:
            raise ValueError('invalid sample budget')


class ReplayPlan:
    """Piecewise cubic smoothstep: C1, no overshoot, zero velocity at knots.

    Recorded dq is validated as evidence, not used as unbounded tangents.
    Analytic maxima: |v| <= 1.5|delta|/h; |a| <= 6|delta|/h².
    Acceleration can jump at knots; jerk is not constrained in this MVP.
    """
    def __init__(self, metadata, arrays, limits, speed_scale=1., *, _raw_only=False):
        if not np.isfinite(speed_scale) or speed_scale <= 0:
            raise ValueError('invalid speed scale')
        self.limits, self.speed_scale = limits, speed_scale
        if 'timestamp' not in arrays:
            raise ValueError('missing timestamp')
        t = np.asarray(arrays['timestamp'], dtype=float)
        if t.ndim != 1:
            raise ValueError('timestamp shape')
        n = len(t)
        shapes = dict(timestamp=(n,), cycle_timestamp=(n,), q_measured=(n,14),
                      dq_measured=(n,14), q_ref=(n,14), waist_q=(n,3), torso_roll_pitch=(n,2))
        if not 2 <= n <= limits.max_samples:
            raise ValueError('sample count')
        for key, shape in shapes.items():
            value = np.asarray(arrays.get(key), dtype=float)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f'invalid field {key}')
        for key, ids in (('arm_motor_ids', np.arange(15,29)), ('waist_motor_ids', np.arange(12,15))):
            if not np.array_equal(arrays.get(key), ids):
                raise ValueError('motor mapping')
        if metadata.get('sample_count') != n or metadata.get('data_integrity') != 'PASS':
            raise ValueError('metadata integrity')
        dt = np.diff(t)
        if np.any(dt <= 0) or np.any(dt > limits.gap):
            raise ValueError('nonmonotonic or discontinuous recording time')
        cycle = arrays['cycle_timestamp']
        if np.any(np.diff(cycle) <= 0) or np.any(cycle < t):
            raise ValueError('cycle time')
        if not np.isclose(metadata.get('duration_s', -1), t[-1]-t[0], atol=1e-8, rtol=1e-8):
            raise ValueError('metadata duration')
        q = arrays['q_measured']
        if np.any(q < limits.q_min) or np.any(q > limits.q_max):
            raise ValueError('recorded position limit')
        dq = arrays['dq_measured']
        if np.max(np.abs(dq)) > limits.velocity or np.max(np.abs(np.diff(dq, axis=0)/dt[:,None])) > limits.acceleration:
            raise ValueError('recorded velocity/acceleration limit')
        delta = np.diff(q, axis=0)
        if np.max(np.abs(delta)) > limits.step:
            raise ValueError('recorded position discontinuity')
        slopes = delta/dt[:,None]
        if (np.max(np.abs(slopes)) > limits.velocity or
                (len(slopes)>1 and np.max(np.abs(np.diff(slopes,axis=0)/((dt[:-1]+dt[1:])/2)[:,None])) > limits.acceleration)):
            raise ValueError('recorded position-derived velocity/acceleration')
        for key, expected in (('start_q', q[0]), ('end_q', q[-1])):
            if key in metadata and not np.allclose(metadata[key], expected, atol=1e-10, rtol=0):
                raise ValueError('metadata endpoint')
        self.time = (t-t[0])/speed_scale
        self.q = q.copy()
        self.time.setflags(write=False)
        self.q.setflags(write=False)
        self.duration = float(self.time[-1])
        h = np.diff(self.time)[:,None]
        self.peak_velocity = float(np.max(1.5*np.abs(delta)/h))
        self.peak_acceleration = float(np.max(6*np.abs(delta)/h**2))
        if not _raw_only and (self.duration > limits.max_duration or self.peak_velocity > limits.velocity or self.peak_acceleration > limits.acceleration):
            raise ValueError('planned duration/velocity/acceleration limit')

    @classmethod
    def load(cls, path, limits, speed_scale=1.):
        metadata, arrays = load_replay_trajectory(path, limits.max_samples)
        return cls(metadata, arrays, limits, speed_scale)

    def sample(self, elapsed):
        if not np.isfinite(elapsed) or elapsed < 0:
            raise ValueError('invalid elapsed time')
        if elapsed >= self.duration:
            return self.q[-1].copy(), np.zeros(14), np.zeros(14)
        i = max(0, min(len(self.time)-2, int(np.searchsorted(self.time, elapsed, side='right')-1)))
        h = self.time[i+1]-self.time[i]
        u = (elapsed-self.time[i])/h
        delta = self.q[i+1]-self.q[i]
        return (self.q[i]+delta*u*u*(3-2*u), delta*6*u*(1-u)/h, delta*6*(1-2*u)/h**2)


@dataclass(frozen=True)
class ReplayGains:
    """Explicit offline candidate gains, never inferred from LOCK gains."""
    kp: tuple
    kd: tuple
    kp_min: tuple
    kp_max: tuple
    kd_min: tuple
    kd_max: tuple

    def __post_init__(self):
        for value in (self.kp, self.kd, self.kp_min, self.kp_max, self.kd_min, self.kd_max):
            if np.asarray(value).shape != (14,) or not np.isfinite(value).all():
                raise ValueError('invalid independent replay gains')
        kp,kd,kp_min,kp_max,kd_min,kd_max=map(np.asarray,
            (self.kp,self.kd,self.kp_min,self.kp_max,self.kd_min,self.kd_max))
        if (np.any(kp_min<=0) or np.any(kd_min<=0) or np.any(kp_max<kp_min)
                or np.any(kd_max<kd_min) or np.any(kp<kp_min) or np.any(kp>kp_max)
                or np.any(kd<kd_min) or np.any(kd>kd_max)):
            raise ValueError('replay gains outside separately reviewed ranges')


class MockOwner:
    """In-memory owner only: cannot publish DDS. No waist trajectory input."""
    def __init__(self):
        self.references = []
        self.acquired = self.released = False

    def acquire(self): self.acquired = True
    def command(self, q, gains): self.references.append(q.copy())
    def release(self): self.released = True


class MockReplayExecutor:
    def __init__(self, limits, gains, owner=None):
        self.limits, self.gains = limits, gains
        self.owner = owner or MockOwner()
        if type(self.owner) is not MockOwner:
            raise ValueError('offline mock owner required; hardware adapters prohibited')
        self.transitions = []
        self.plan = None
        self.last_time = self.last_reference = None
        self.started = self.hold_started = None
        self.state = None

    def transition(self, state):
        self.state = state
        self.transitions.append(state)

    def load(self, path, speed_scale=1.):
        if self.state is not None:
            raise ValueError('executor is single use')
        self.transition('LOAD')
        try:
            metadata, arrays = load_replay_trajectory(path, self.limits.max_samples)
            self.transition('VALIDATE')
            self.plan = ReplayPlan(metadata, arrays, self.limits, speed_scale)
            self.transition('ACQUIRE')
            self.owner.acquire()
            self.transition('START_CHECK')
        except BaseException:
            self.close()
            raise

    def tick(self, now, lowstate, hold_duration=1.):
        if self.state not in ('START_CHECK', 'EXECUTE', 'HOLD_END'):
            raise ValueError('executor not active')
        try:
            if not np.isfinite(now) or not np.isfinite(hold_duration) or hold_duration < 0:
                raise ValueError('invalid timing')
            q, dq = np.asarray(lowstate.q), np.asarray(lowstate.dq)
            if q.shape != (14,) or dq.shape != (14,) or not np.isfinite([q,dq]).all():
                raise ValueError('invalid LowState')
            if not np.isfinite(lowstate.timestamp) or not 0 <= now-lowstate.timestamp <= self.limits.state_timeout:
                raise ValueError('communication stale/future state')
            if self.last_time is not None and not 0 < now-self.last_time <= self.limits.cycle_timeout:
                raise ValueError('execution clock/cycle timeout')
            if np.any(q < self.limits.q_min) or np.any(q > self.limits.q_max) or np.max(np.abs(dq)) > self.limits.velocity:
                raise ValueError('measured position/velocity')
            if self.state == 'START_CHECK':
                if np.max(np.abs(q-self.plan.q[0])) > self.limits.start_error:
                    raise ValueError('start position mismatch; no automatic alignment')
                if np.max(np.abs(dq)) > self.limits.start_velocity:
                    raise ValueError('start not stationary')
                self.started = now
                self.last_reference = q.copy()
                self.transition('EXECUTE')
            elapsed = now-self.started
            if elapsed > self.plan.duration + hold_duration + self.limits.cycle_timeout:
                raise ValueError('total execution timeout')
            reference, _, _ = self.plan.sample(elapsed)
            if np.max(np.abs(q-reference)) > self.limits.tracking_error:
                raise ValueError('tracking error')
            if np.max(np.abs(reference-self.last_reference)) > self.limits.step:
                raise ValueError('adjacent target step')
            self.owner.command(reference, self.gains)
            self.last_time, self.last_reference = now, reference.copy()
            if elapsed >= self.plan.duration and self.state == 'EXECUTE':
                self.hold_started = now
                self.transition('HOLD_END')
            if self.state == 'HOLD_END' and now-self.hold_started >= hold_duration:
                self.close()
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.state != 'RELEASE':
            self.transition('RELEASE')
            self.owner.release()
