"""Inspect final LowCmd fields, including float32 CDR round-trip values."""
import numpy as np
from .arm_sdk_types import vector

ARM_MOTOR_IDS = (15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28)
ARM_JOINT_NAMES = tuple(side + joint for side in ('Left', 'Right') for joint in (
    'ShoulderPitch', 'ShoulderRoll', 'ShoulderYaw', 'Elbow', 'WristRoll', 'WristPitch', 'WristYaw'))
WIRE_EPSILON = 1e-6


def payload(message):
    result = {f'WIRE_ARM_{field.upper()}': [float(getattr(message.motor_cmd[i], field))
              for i in ARM_MOTOR_IDS] for field in ('q', 'dq', 'kp', 'kd', 'tau')}
    result['WIRE_WEIGHT'] = float(message.motor_cmd[29].q)
    return result


def compare(message, acquire_q):
    q = vector(acquire_q, 14)
    data = payload(message)
    matches = np.isclose(data['WIRE_ARM_Q'], q, rtol=0, atol=WIRE_EPSILON)
    data['ACQUIRE_Q_ARM'] = q.tolist()
    for offset, motor in enumerate(ARM_MOTOR_IDS):
        for field in ('Q', 'DQ', 'KP', 'KD', 'TAU'):
            data[f'WIRE_MOTOR_{motor}_{field}'] = data[f'WIRE_ARM_{field}'][offset]
        data[f'WIRE_Q_MATCH_ACQUIRE_Q_{motor}'] = 'YES' if matches[offset] else 'NO'
    data['WIRE_MOTOR_29_Q_WEIGHT'] = data['WIRE_WEIGHT']
    data['ALL_ARM_WIRE_Q_MATCH_ACQUIRE'] = 'YES' if matches.all() else 'NO'
    data['REAL_ACQUIRE_ALLOWED'] = 'NO'  # Mapping evidence never lifts the hardware test suspension.
    return data


def guard(message, command, acquire_q=None):
    wire = payload(message)
    for field in ('q', 'dq', 'kp', 'kd', 'tau'):
        expected = command.tau_ff if field == 'tau' else getattr(command, field)
        actual = np.asarray(wire[f'WIRE_ARM_{field.upper()}'])
        if not np.isfinite(actual).all() or not np.allclose(actual, expected, rtol=0, atol=WIRE_EPSILON):
            raise ValueError(f'ABORT_BEFORE_DDS_WRITE: wire {field} mismatch')
    if acquire_q is not None:
        if compare(message, acquire_q)['ALL_ARM_WIRE_Q_MATCH_ACQUIRE'] != 'YES':
            raise ValueError('ABORT_BEFORE_DDS_WRITE: acquire reference mismatch')
        if np.any(np.asarray(wire['WIRE_ARM_DQ']) != 0) or np.any(np.asarray(wire['WIRE_ARM_TAU']) != 0):
            raise ValueError('ABORT_BEFORE_DDS_WRITE: HOLD dq/tau must be zero')
    return wire


def lowcmd_snapshot(message, label, sequence, timestamp=None):
    """Deterministic motor0..29 snapshot; callers may supply an audit timestamp."""
    motors = []
    for motor_id in range(30):
        motor = message.motor_cmd[motor_id]
        entry = {'motor_id': motor_id}
        for field in ('q', 'dq', 'kp', 'kd', 'tau'):
            entry[field] = float(getattr(motor, field))
        if hasattr(motor, 'mode'):
            entry['mode'] = int(motor.mode)
        if hasattr(motor, 'reserve'):
            entry['reserve'] = int(motor.reserve)
        motors.append(entry)
    result = {'label': label, 'sequence': int(sequence), 'mode_pr': int(message.mode_pr),
              'mode_machine': int(getattr(message, 'mode_machine', 0)), 'crc': int(message.crc), 'motors': motors}
    if hasattr(message, 'reserve'):
        result['reserve'] = list(message.reserve)
    if timestamp is not None:
        result['timestamp'] = float(timestamp)
    return result


def snapshot_json(snapshot):
    import json
    return json.dumps(snapshot, sort_keys=True, separators=(',', ':'), allow_nan=False)
