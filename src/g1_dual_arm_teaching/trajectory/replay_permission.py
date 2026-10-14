"""Load Replay runtime constraints from the internal configuration."""
from pathlib import Path
import numpy as np
import yaml
from .replay import ReplayLimits, ReplayGains
from .replay_c2 import RelativeC2ReplayPlan, RelativeReplaySpec


class ReplayPermission:
    """Compatibility name for the validated Replay runtime configuration."""
    def __init__(self, path, trajectory=None, *, execute=False):
        path = Path(path)
        if path.suffix not in ('.yaml', '.yml'):
            raise ValueError('Replay runtime configuration must be YAML')
        self.manifest = yaml.safe_load(path.read_text(encoding='utf-8'))
        m = self.manifest
        if not isinstance(m, dict):
            raise ValueError('Replay configuration must be a YAML mapping')
        self.replay_mode = m.get('candidate', {}).get('mode', 'absolute')
        if self.replay_mode not in ('absolute', 'relative'):
            raise ValueError('replay_mode must be absolute or relative')
        self.start_transition = {'enabled': False}
        self.start_transition_enabled = False
        self.speed = float(m.get('speed_scale', 1.0))
        if not np.isfinite(self.speed) or self.speed <= 0:
            raise ValueError('speed_scale must be finite and positive')
        self.allowed = m.get('hardware_execute_enabled') is True
        if execute and not self.allowed:
            raise ValueError('hardware execute disabled in configs/replay.yaml')
        if not execute:
            values = m.get('offline_limits')
            if not isinstance(values, dict):
                raise ValueError('configs/replay.yaml is missing offline_limits')
            self.limits = ReplayLimits(**values)
            self.jerk = float(m['offline_jerk_limit'])
            self.jerk_jump = float(m['offline_jerk_jump_limit'])
            if not np.isfinite([self.jerk, self.jerk_jump]).all() or min(self.jerk, self.jerk_jump) <= 0:
                raise ValueError('invalid offline jerk diagnostic bounds')
            self.gains = None
            self.hold = self.stop = self.period = self.gain_ramp = None
            self.fault_actions = {}
            return
        values = m.get('limits')
        limit_fields = ReplayLimits.__dataclass_fields__
        if not isinstance(values, dict) or any(values.get(key) is None
                                               for key in limit_fields if key != 'max_samples'):
            raise ValueError('configs/replay.yaml must contain measured hardware limits')
        self.limits = ReplayLimits(**m['limits'])
        self.jerk = float(m['jerk_limit'])
        self.jerk_jump = float(m['jerk_jump_limit'])
        if execute:
            for section, fields in (('gains', ReplayGains.__dataclass_fields__),):
                values = m.get(section)
                if not isinstance(values, dict) or any(values.get(key) is None for key in fields):
                    raise ValueError(f'configs/replay.yaml must contain measured {section} values')
            self.gains = ReplayGains(**m['gains'])
            for key in ('hold_end_s', 'stop_duration_s', 'period_s', 'gain_ramp_s',
                        'max_first_run_duration_s', 'max_first_run_span_rad'):
                if key == 'gain_ramp_s':
                    continue
                if m.get(key) is None or not np.isfinite(m[key]) or m[key] <= 0:
                    raise ValueError(f'configs/replay.yaml must contain explicit {key}')
            self.hold = float(m['hold_end_s'])
            self.stop = float(m['stop_duration_s'])
            self.period = float(m['period_s'])
            from ..sdk.golden_reference import GOLDEN_RAMP_S
            self.gain_ramp = GOLDEN_RAMP_S
            if self.period > self.limits.cycle_timeout:
                raise ValueError('invalid Replay period')
            values = (self.speed, self.jerk, self.jerk_jump, self.stop, self.period, self.gain_ramp)
            if any(not np.isfinite(value) or value <= 0 for value in values):
                raise ValueError('invalid Replay timing or constraint')
        policies = m.get('fault_actions', {})
        required = ('preflight', 'feedback', 'tracking', 'motion', 'timing', 'write', 'operator')
        if any(policies.get(key) not in (
                'freeze_then_bounded_release',
                'external_stop_required_then_best_effort_release') for key in required):
            raise ValueError('explicit per-fault release decisions required')
        self.fault_actions = policies
        if not np.array_equal(self.gains.kd, np.full(14, 1.5)):
            raise ValueError('Kd must match audited Golden Kd1.5')
        if any(lo > 40 or hi < 40 for lo, hi in zip(self.gains.kp_min, self.gains.kp_max)):
            raise ValueError('gain-transition ranges must include acquisition Kp40')

    def check_plan(self, plan):
        relative = isinstance(plan, (RelativeC2ReplayPlan, RelativeReplaySpec))
        if self.replay_mode != ('relative' if relative else 'absolute'):
            raise ValueError('configured replay_mode does not match planned trajectory')
        if relative:
            if (not np.isfinite(plan.motion_scale) or plan.motion_scale <= 0 or
                    type(plan.start_index) is not int or type(plan.end_index) is not int or
                    plan.start_index < 0 or plan.end_index - plan.start_index < 3):
                raise ValueError('invalid selected relative mode/scale/segment')
            candidate = self.manifest.get('candidate', self.manifest)
            if (candidate.get('motion_scale', candidate.get('relative_motion_scale')) != plan.motion_scale or
                    candidate.get('start_index', candidate.get('relative_start_index')) != plan.start_index or
                    candidate.get('end_index', candidate.get('relative_end_index')) != plan.end_index):
                raise ValueError('runtime config is not bound to the selected mode/scale/segment')
        if isinstance(plan, RelativeReplaySpec):
            plan = plan.bind(np.zeros(14))
        source = plan.q
        if getattr(self, 'allowed', False):
            self._check_envelope(plan.duration, np.ptp(source, axis=0), plan.peak_velocity,
                                 plan.peak_acceleration, plan.peak_jerk)

    def _check_envelope(self, duration, span, peak_velocity, peak_acceleration, peak_jerk):
        if (duration > self.manifest['max_first_run_duration_s'] or
                np.max(span) > self.manifest['max_first_run_span_rad']):
            raise ValueError('first-run duration/span envelope exceeded')
        if (peak_acceleration + 1.875 * peak_velocity / self.stop > self.limits.acceleration or
                peak_jerk + 5.625 * peak_acceleration / self.stop +
                6 * peak_velocity / self.stop**2 > self.jerk):
            raise ValueError('controlled-stop constraints cannot be met')

    def make_start_transition(self, start_q, plan):
        return None
