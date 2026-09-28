"""Load and validate the Phase 0/1 configuration before creating DDS entities."""
from pathlib import Path
import numpy as np
import yaml
from ..sdk.arm_sdk_types import vector


def positive(value, name):
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return float(value)


def load_config(directory):
    directory = Path(directory)
    config = {}
    for name in ('robot', 'arm_limits', 'controller'):
        with (directory / f'{name}.yaml').open() as stream:
            for key, value in yaml.safe_load(stream).items():
                if isinstance(value, dict) and key in config:
                    config[key].update(value)
                else:
                    config[key] = value
    r, c, a, w, s, limits = (config[k] for k in ('robot', 'control', 'arm', 'waist', 'safety', 'limits'))
    if r['arm_sdk_topic'] != 'rt/arm_sdk' or r['arm_dof'] != 14:
        raise ValueError('only G1 14-arm-joint rt/arm_sdk is supported')
    if r['waist_joint_indices'] != [12, 13, 14]:
        raise ValueError('invalid waist indices')
    if a['tau_ff_enabled'] is not False or w['mode'] != 'MONITOR_ONLY':
        raise ValueError('Phase 0 requires zero feedforward and monitor-only waist')
    if config['trajectory']['interpolation'] != 'smoothstep':
        raise ValueError('only smoothstep is implemented')
    for key in ('frequency_hz', 'acquire_ramp_s', 'release_ramp_s', 'initial_state_timeout_s'):
        positive(c[key], key)
    for key in ('state_timeout_s', 'command_timeout_s', 'max_joint_step_rad'):
        positive(s[key], key)
    positive(limits['max_velocity_rad_s'], 'max velocity')
    for group, size in ((a, 14), (w, 3)):
        for key in ('kp', 'kd'):
            values = vector(group[key], size)
            if np.any(values < 0):
                raise ValueError('gains must be non-negative')
    lo, hi = vector(limits['q_min'], 14), vector(limits['q_max'], 14)
    if np.any(lo >= hi):
        raise ValueError('invalid joint limits')
    positive(config['trajectory']['default_duration_s'], 'default duration')
    if not isinstance(r['hardware_reviewed'], bool):
        raise ValueError('hardware_reviewed must be boolean')
    if w['send_commands'] is not False or w['experiment'] != 'ZERO_GAIN_NO_COMMAND':
        raise ValueError('V1 waist command must remain disabled')
    from .hardware import validate_variant, weight_target
    validate_variant(config)
    weight_target(config["arm_sdk"]["acquire_weight_target"])
    for key in ("stable_state_s", "stable_position_span_rad", "stable_velocity_rad_s", "release_observe_s"):
        positive(c[key], key)
    positive(limits["sanity_abs_rad"], "sanity limit")
    from ..sdk.phase0_acquire import validate_phase0
    validate_phase0(config)
    return config
