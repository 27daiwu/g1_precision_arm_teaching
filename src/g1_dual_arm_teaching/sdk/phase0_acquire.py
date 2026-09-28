"""Full ownership and servo gains are independent Phase 0 concepts."""
import time
import numpy as np
from .wire_audit import payload, WIRE_EPSILON
from .arm_sdk_types import vector

STRATEGY = 'FULL_WEIGHT_CURRENT_POSE'


def validate_phase0(config):
    settings = config['phase0_hold']
    weight = config['arm_sdk']['ownership_weight']
    if isinstance(weight, bool) or weight != 1.0:
        raise ValueError('Phase 0 ownership_weight must be 1.0; partial weight is not a safe gain')
    for key in ('kp', 'kd'):
        if np.any(vector(settings[key], 14) < 0):
            raise ValueError('Phase 0 gains must be non-negative')
    for key in ('pre_acquire_motion_threshold', 'duration_s'):
        if not np.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError(f'invalid Phase 0 {key}')
    watchdog = settings['waist_watchdog']
    for key in ('max_abs_delta_rad', 'max_abs_velocity_rad_s'):
        if not np.isfinite(watchdog[key]) or watchdog[key] <= 0:
            raise ValueError(f'invalid waist watchdog {key}')


def first_write_diagnostics(wire, acquire_q, latest, config):
    """Return auditable PASS/FAIL, using the final wire fields and newest state."""
    reference = vector(acquire_q, 14)
    delta = latest.q - reference
    age = time.monotonic() - latest.timestamp
    errors = []
    if not 0 <= age <= config['safety']['state_timeout_s']:
        errors.append('STATE_TIMEOUT')
    if latest.dq is None or np.any(np.abs(latest.dq) > config['control']['stable_velocity_rad_s']):
        errors.append('PRE_ACQUIRE_VELOCITY')
    if np.max(np.abs(delta)) >= config['phase0_hold']['pre_acquire_motion_threshold']:
        errors.append('PRE_ACQUIRE_MOTION')
    if wire['WIRE_WEIGHT'] != 1.0:
        errors.append('FIRST_WEIGHT_NOT_ONE')
    for field, expected in (('Q', reference), ('DQ', np.zeros(14)), ('TAU', np.zeros(14)),
                            ('KP', config['phase0_hold']['kp']), ('KD', config['phase0_hold']['kd'])):
        if not np.allclose(wire[f'WIRE_ARM_{field}'], expected, rtol=0, atol=WIRE_EPSILON):
            errors.append(f'FIRST_WIRE_{field}_MISMATCH')
    return dict(ACQUIRE_STRATEGY=STRATEGY, ACQUIRE_Q=reference.tolist(),
                Q_MEASURED_IMMEDIATELY_BEFORE_FIRST_WRITE=latest.q.tolist(),
                LATEST_Q_BEFORE_FIRST_WRITE=latest.q.tolist(),
                DELTA_Q_BEFORE_FIRST_WRITE=delta.tolist(), FIRST_STATE_AGE=age,
                FIRST_WIRE_Q=wire['WIRE_ARM_Q'], FIRST_WIRE_KP=wire['WIRE_ARM_KP'],
                FIRST_WIRE_KD=wire['WIRE_ARM_KD'], FIRST_WIRE_WEIGHT=wire['WIRE_WEIGHT'],
                PHASE0_KP_15_28=config['phase0_hold']['kp'], PHASE0_KD_15_28=config['phase0_hold']['kd'],
                FIRST_WRITE_GUARD='FAIL' if errors else 'PASS', FIRST_WRITE_ERRORS=errors)


def require_first_write(data):
    if data['FIRST_WRITE_GUARD'] != 'PASS':
        raise ValueError('ABORT_BEFORE_DDS_WRITE: ' + ','.join(data['FIRST_WRITE_ERRORS']))
