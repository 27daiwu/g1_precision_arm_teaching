"""Frozen wire law from scripts/test_armsdk.py (motor12..28)."""
import numpy as np
from .wire_audit import lowcmd_snapshot
from .arm_sdk_types import vector

GOLDEN_REFERENCE = 'WORKING_TEST_ARMSDK'
GOLDEN_COMMAND_HZ = 50.0
GOLDEN_KP = 40.0
GOLDEN_KD = 1.5
GOLDEN_MOTOR_IDS = tuple(range(12, 29))
GOLDEN_RAMP_S = 2.0


def fill_golden_frame(message, upper_q, weight):
    q = vector(upper_q, 17)
    if not np.isfinite(weight) or not 0 <= weight <= 1:
        raise ValueError('invalid Golden weight')
    if len({id(m) for m in message.motor_cmd}) != len(message.motor_cmd):
        raise ValueError('aliased motor slots')
    for offset, motor_id in enumerate(GOLDEN_MOTOR_IDS):
        motor = message.motor_cmd[motor_id]
        motor.q, motor.dq = float(q[offset]), 0.0
        motor.kp, motor.kd, motor.tau = GOLDEN_KP, GOLDEN_KD, 0.0
    message.motor_cmd[29].q = float(weight)


def build_golden_frame(message_factory, crc, upper_q, weight, label='FIRST_FRAME', sequence=0):
    message = message_factory()
    fill_golden_frame(message, upper_q, weight)
    message.crc = crc.Crc(message)
    return message, lowcmd_snapshot(message, label, sequence)


def compare_golden_phase0(golden, phase0):
    differences = []
    for motor_id in range(30):
        for field in ('q', 'dq', 'kp', 'kd', 'tau', 'mode', 'reserve'):
            if golden['motors'][motor_id].get(field) != phase0['motors'][motor_id].get(field):
                differences.append(f'motor{motor_id}.{field}')
    for field in ('mode_pr', 'mode_machine', 'reserve'):
        if golden.get(field) != phase0.get(field):
            differences.append(field)
    return differences
