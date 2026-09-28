"""Single-publisher lifecycle, no-jump acquire, watchdog and safe release."""
import copy
import fcntl
import os
import threading
import time
import logging
import numpy as np
from .arm_sdk_types import vector
from .transport import SimulationTransport, UnitreeTransport
from ..control.command_builder import build_arm_command
from ..control.safety_guard import SafetyGuard
from ..utils.timing import ControlLoop

LOG = logging.getLogger(__name__)


class ArmSdkClient:
    topic = 'rt/arm_sdk'
    control_available = 'UNKNOWN'
    action_conflict_detected = 'UNKNOWN'

    def __init__(self, config, *, real=False, network_interface=None, read_only=False,
                 exclusive_control_confirmed=False, transport=None, current_pose_only=False, weight=None):
        config = copy.deepcopy(config)
        self.config, self.real, self.read_only = config, real, read_only
        from ..utils.hardware import weight_target, validate_variant
        validate_variant(config)
        self.current_pose_only = current_pose_only
        if current_pose_only:
            from .phase0_acquire import validate_phase0
            validate_phase0(config)
            if weight is not None and weight != 1.0:
                raise ValueError('Phase 0 partial weight is forbidden; ownership_weight must be 1.0')
            self.weight_target = float(config['arm_sdk']['ownership_weight'])
        else:
            self.weight_target = weight_target(config['arm_sdk']['acquire_weight_target'] if weight is None else weight)
        self.first_write_diagnostics = None
        self.telemetry = None
        self.phase = 'idle'
        self._last_publish_time = None
        self.interface = network_interface or config['robot']['network_interface']
        self.exclusive_control_confirmed = exclusive_control_confirmed
        self.transport = transport if transport is not None else (UnitreeTransport() if real else SimulationTransport())
        limits, safety = config['limits'], config['safety']
        self.guard = SafetyGuard(limits['q_min'], limits['q_max'],
                                 max_velocity_rad_s=limits['max_velocity_rad_s'], **safety)
        if real or current_pose_only or read_only:
            sanity = config['limits']['sanity_abs_rad']
            if not np.isfinite(sanity) or sanity <= 0:
                raise ValueError('invalid sanity limit')
            self.guard.joint_min = np.full(14, -sanity)
            self.guard.joint_max = np.full(14, sanity)
        self.period_s = 1.0 / config['control']['frequency_hz']
        self._mutex = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._lease = None
        self._initialized = self._acquired = self._closed = False
        self._last = None
        self._weight = 0.0
        self._fault = None
        self.acquire_q = self.acquire_waist_q = None

    def initialize(self):
        if self._initialized:
            return self
        if self._closed:
            raise RuntimeError('client closed')
        if self.real and not self.read_only:
            if self.config['robot']['hardware_reviewed'] is not True:
                raise RuntimeError('target hardware configuration has not been reviewed')
            if not self.exclusive_control_confirmed:
                raise RuntimeError('Arm Action and other streaming controllers must be stopped; explicit exclusive-control confirmation required')
        if self.real and not self.read_only:
            from ..utils.hardware import require_real_acquire, verified_limits
            require_real_acquire(self.config, self.current_pose_only)
            if not self.current_pose_only:
                limits = verified_limits(self.config)
                self.guard.joint_min = np.array([limits[i][0] for i in range(15, 29)])
                self.guard.joint_max = np.array([limits[i][1] for i in range(15, 29)])
        try:
            if not self.read_only:
                # Cooperative host-wide lease, held until DDS publisher has closed.
                path = f'/tmp/g1_dual_arm_teaching_{"real" if self.real else "sim"}.lock'
                fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
                self._lease = os.fdopen(fd, 'w')
                fcntl.flock(self._lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.transport.initialize(self.config, self.interface, not self.read_only)
            self._initialized = True
            loop = ControlLoop(self.config['control']['frequency_hz'])
            deadline = time.monotonic() + self.config['control']['initial_state_timeout_s']
            while self.transport.state() is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError('initial state timeout')
                loop.wait()
            self.get_joint_state()
            if not self.read_only:
                self._thread = threading.Thread(target=self._watchdog, name='arm-sdk-watchdog', daemon=True)
                self._thread.start()
            return self
        except BaseException:
            self._close_transport()
            raise

    def get_joint_state(self):
        if not self._initialized or self._closed:
            raise RuntimeError('client not initialized or already closed')
        state = self.transport.state()
        if state is None:
            raise TimeoutError('no state')
        self.guard.validate(state.q, state_timestamp=state.timestamp)
        if state.waist_q is None:
            raise ValueError('waist state is required')
        if self.real and not self.read_only and not self.current_pose_only:
            from ..utils.hardware import verified_limits, active_waist
            limits = verified_limits(self.config)
            for motor in active_waist(self.config):
                if not limits[motor][0] <= state.waist_q[motor - 12] <= limits[motor][1]:
                    raise ValueError('waist verified joint limit')
        if np.any(np.abs(state.waist_q) > self.config['limits']['sanity_abs_rad']):
            raise ValueError('waist sanity limit')
        if self.current_pose_only and self.acquire_waist_q is not None and self._acquired:
            self._check_waist_watchdog(state)
        return state

    def _check_waist_watchdog(self, state):
        settings = self.config['phase0_hold']['waist_watchdog']
        if state.waist_dq is None:
            raise RuntimeError('WAIST_SAFETY_ABORT: missing waist velocity')
        delta = state.waist_q - self.acquire_waist_q
        diagnostic = dict(WAIST_Q_AT_ACQUIRE=self.acquire_waist_q.tolist(),
                          WAIST_Q_CURRENT=state.waist_q.tolist(),
                          WAIST_DQ=state.waist_dq.tolist(), WAIST_DELTA_Q=delta.tolist())
        if self.telemetry is not None:
            self.telemetry.write(dict(event='waist_watchdog', timestamp=time.monotonic(), **diagnostic))
        if (np.max(np.abs(delta)) > settings['max_abs_delta_rad'] or
                np.max(np.abs(state.waist_dq)) > settings['max_abs_velocity_rad_s']):
            LOG.error('WAIST_SAFETY_ABORT %s', diagnostic)
            raise RuntimeError('WAIST_SAFETY_ABORT')
        return diagnostic

    def wait_stable_state(self):
        settings = self.config['control']
        deadline = time.monotonic() + settings['initial_state_timeout_s']
        loop = ControlLoop(settings['frequency_hz'])
        samples = []
        while time.monotonic() < deadline:
            state = self.get_joint_state()
            if state.dq is None or np.max(np.abs(state.dq)) > settings['stable_velocity_rad_s']:
                samples = []
            elif not samples or state.timestamp > samples[-1].timestamp:
                samples.append(state)
                span = np.ptp(np.array([np.r_[s.q, s.waist_q] for s in samples]), axis=0)
                if np.max(span) > settings['stable_position_span_rad']:
                    samples = [state]
                if len(samples) >= 3 and samples[-1].timestamp - samples[0].timestamp >= settings['stable_state_s']:
                    return state
            loop.wait()
        raise TimeoutError('stable lowstate window not observed')

    def _command(self, q, dq=None):
        gains = self.config['phase0_hold'] if self.current_pose_only else self.config['arm']
        return build_arm_command(q, dq, gains['kp'], gains['kd'])

    def acquire(self, q_reference=None):
        if self.current_pose_only and q_reference is not None:
            raise ValueError('CURRENT_POSE_ONLY forbids supplied target/reference')
        stable = self.wait_stable_state() if self.current_pose_only else None
        self.phase = 'acquire'
        with self._mutex:
            if self._fault or self._acquired or self.read_only:
                raise RuntimeError('acquire not allowed')
            state = stable if stable is not None else self.get_joint_state()
            if state.dq is not None and np.any(np.abs(state.dq) > self.guard.max_velocity_rad_s):
                raise ValueError('measured velocity limit at acquire')
            if q_reference is not None and not np.array_equal(vector(q_reference, 14), state.q):
                raise ValueError('acquire reference must exactly equal measured q')
            self.acquire_q = state.q.copy()
            self.acquire_waist_q = state.waist_q.copy()
            self.acquire_q.flags.writeable = False
            self.acquire_waist_q.flags.writeable = False
            if isinstance(self.transport, UnitreeTransport) and self.current_pose_only:
                self.transport.acquire_reference = self.acquire_q.copy()
                self.transport.acquire_reference.flags.writeable = False
            self._last = self._command(state.q)
            self._last_time = time.monotonic()
            if self.current_pose_only and isinstance(self.transport, UnitreeTransport):
                self.transport.first_write_check = self._check_first_wire
            self._acquired = not self.current_pose_only
            try:
                if self.current_pose_only and not isinstance(self.transport, UnitreeTransport):
                    wire = {f'WIRE_ARM_{name}': getattr(self._last, field).tolist()
                            for name, field in [('Q','q'), ('DQ','dq'), ('KP','kp'), ('KD','kd'), ('TAU','tau_ff')]}
                    wire['WIRE_WEIGHT'] = 1.0
                    self._check_first_wire(wire)
                self._publish(self._last, 1.0 if self.current_pose_only else 0.0)
                self._acquired = True
            except BaseException:
                self.abort('acquire DDS failure')
                raise
        if self.current_pose_only:
            self.phase = 'hold'
            return state
        loop = ControlLoop(self.config['control']['frequency_hz'])
        steps = max(1, int(np.ceil(self.config['control']['acquire_ramp_s'] / self.period_s)))
        try:
            for step in range(1, steps + 1):
                loop.wait()
                self.send_joint_command(self._command(self.acquire_q), weight=self.weight_target * (step / steps) ** 2 * (3 - 2 * step / steps))
        except BaseException:
            self.abort('acquire interrupted')
            raise
        self.phase = "hold"
        return state

    def _check_first_wire(self, wire):
        from .phase0_acquire import first_write_diagnostics, require_first_write
        latest = self.get_joint_state()
        data = first_write_diagnostics(wire, self.acquire_q, latest, self.config)
        self.first_write_diagnostics = data
        LOG.info('FIRST_WRITE_DIAGNOSTICS %s', data)
        if self.telemetry is not None:
            self.telemetry.write(dict(event='first_write_diagnostics', **data))
        require_first_write(data)

    def _publish(self, command, weight):
        # V1 matches the arm-only Golden Reference: waist slots retain SDK defaults.
        self.transport.write(command, None, weight)
        if self.current_pose_only and self.phase == 'acquire':
            self._acquired = True
        now = time.monotonic()
        dt = None if self._last_publish_time is None else now - self._last_publish_time
        self._last, self._weight, self._last_time = command, weight, now
        self._last_publish_time = now
        if self.telemetry is not None:
            self.telemetry.cycle(self.transport.state(), command, weight, self.phase, dt)

    def send_joint_command(self, command, *, weight=None):
        with self._mutex:
            if self._fault or not self._acquired or self._closed:
                raise RuntimeError(f'command rejected: {self._fault or "not acquired"}')
            try:
                if self.current_pose_only and (not np.array_equal(command.q, self.acquire_q) or np.any(command.dq != 0)):
                    raise ValueError('CURRENT_POSE_ONLY forbids target, waypoint or trajectory')
                command = self.guard.command(command, self._last.q, self.get_joint_state(),
                                             self._last_time, self.period_s)
                value = self._weight if weight is None else float(weight)
                if self.current_pose_only and self.phase != 'release' and value != 1.0:
                    raise ValueError('Phase 0 ownership weight must remain 1 during HOLD')
                if not np.isfinite(value) or not 0 <= value <= self.weight_target:
                    raise ValueError('invalid SDK weight')
                self._publish(command, value)
            except BaseException:
                self.abort('command validation or DDS failure')
                raise

    def hold(self):
        self.send_joint_command(self._command(self._last.q))

    def _watchdog(self):
        while not self._stop.wait(min(self.period_s, self.guard.command_timeout_s / 2)):
            with self._mutex:
                if self._acquired:
                    try:
                        self.get_joint_state()
                        self.guard.check_age(self._last_time, self.guard.command_timeout_s, 'command')
                    except Exception as exc:
                        self.abort(str(exc))
                        return

    def abort(self, reason='ABORT'):
        with self._mutex:
            self._fault = reason
            LOG.error('ABORT: %s', reason)
            try:
                if self._acquired and self._last is not None and not self._closed:
                    # One best-effort disable frame, never resume an old trajectory.
                    self._publish(self._command(self._last.q), 0.0)
            except Exception:
                LOG.exception('SDK disable failed; hardware safety intervention may be needed')
            finally:
                self._acquired = False
                self._close_transport()

    def release(self):
        if not self._acquired or self._closed:
            return
        loop = ControlLoop(self.config['control']['frequency_hz'])
        weight = self._weight
        self.phase = 'release'
        steps = max(1, int(np.ceil(self.config['control']['release_ramp_s'] / self.period_s)))
        try:
            if self.telemetry is not None:
                self.telemetry.before_release(self.get_joint_state(), weight)
            for step in range(1, steps + 1):
                self.send_joint_command(self._command(self._last.q), weight=weight * (1 - (step / steps) ** 2 * (3 - 2 * step / steps)))
                if step < steps:
                    loop.wait()
            with self._mutex:
                self._acquired = False
            if self.telemetry is not None:
                self.phase = 'post_release'
                until = time.monotonic() + self.config['control']['release_observe_s']
                while time.monotonic() < until:
                    self.telemetry.after_release(self.get_joint_state(), self._weight)
                    loop.wait()
        except BaseException:
            self.abort('release interrupted or failed')
            raise

    def _close_transport(self):
        self._stop.set()
        try:
            if not self._closed:
                self.transport.close()
        finally:
            self._closed = True
            if self._lease is not None:
                self._lease.close()
                self._lease = None

    def shutdown(self):
        try:
            self.release()
        finally:
            with self._mutex:
                self._close_transport()
            if self._thread is not None and self._thread is not threading.current_thread():
                self._thread.join(timeout=1.0)

    def __enter__(self):
        return self.initialize()

    def __exit__(self, kind, value, traceback):
        if kind is KeyboardInterrupt:
            # Match the Golden Reference: operator stop uses the normal smooth release.
            self.shutdown()
            return False
        if kind is not None:
            self.abort(str(value))
        self.shutdown()
