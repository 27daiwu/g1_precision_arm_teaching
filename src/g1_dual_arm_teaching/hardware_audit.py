"""Read-only lowstate audit. Frequency is based on actual DDS callbacks."""
import json
import time
from pathlib import Path
from .utils.timing import ControlLoop


def run_audit(client, duration, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    loop = ControlLoop(client.config['control']['frequency_hz'])
    deadline = time.monotonic() + duration
    with path.open('x', encoding='utf-8') as stream:
        while True:
            state = client.get_joint_state()
            diagnostics = client.transport.diagnostics() if client.real else dict(
                LOWSTATE_RECEIVED=False, LOWSTATE_FREQUENCY=None, MODE_MACHINE=None)
            row = dict(timestamp=time.monotonic(), BACKEND='hardware' if client.real else 'simulation',
                       DDS_INTERFACE=client.interface, **diagnostics,
                       MOTOR_12_Q=float(state.waist_q[0]), MOTOR_13_Q=float(state.waist_q[1]), MOTOR_14_Q=float(state.waist_q[2]),
                       MOTOR_15_28_Q=state.q.tolist(), MOTOR_15_28_DQ=state.dq.tolist(),
                       ROBOT_DOF_CONFIG=client.config['robot']['dof'],
                       WAIST_CONFIGURATION_VERIFIED='YES' if client.config['waist']['configuration_verified'] else 'NO',
                       WAIST_CONFIGURATION=client.config['waist']['configuration'])
            stream.write(json.dumps(row, allow_nan=False) + '\n')
            stream.flush()
            if time.monotonic() >= deadline:
                print(json.dumps(row, allow_nan=False))
                return row
            loop.wait()
