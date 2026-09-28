"""Validation shared by acquire, hold and every trajectory command."""
import time
import numpy as np
from ..sdk.arm_sdk_types import vector, ArmJointCommand


class SafetyGuard:
    def __init__(self, joint_min, joint_max, max_joint_step_rad=0.05,
                 state_timeout_s=0.2, max_velocity_rad_s=1.0, command_timeout_s=0.2):
        self.joint_min = vector(joint_min, 14)
        self.joint_max = vector(joint_max, 14)
        if np.any(self.joint_min >= self.joint_max):
            raise ValueError('invalid limits')
        for name, value in locals().copy().items():
            if name in ('max_joint_step_rad', 'state_timeout_s', 'max_velocity_rad_s', 'command_timeout_s'):
                if not np.isfinite(value) or value <= 0:
                    raise ValueError(f'invalid {name}')
                setattr(self, name, float(value))

    def validate(self, q, previous_q=None, state_timestamp=None, now=None):
        q = vector(q, 14)
        if np.any(q < self.joint_min) or np.any(q > self.joint_max):
            raise ValueError('joint position limit')
        if previous_q is not None and np.any(np.abs(q - vector(previous_q, 14)) > self.max_joint_step_rad + 1e-12):
            raise ValueError('command step limit')
        if state_timestamp is not None:
            self.check_age(state_timestamp, self.state_timeout_s, 'state', now)

    @staticmethod
    def check_age(timestamp, timeout, name, now=None):
        now = time.monotonic() if now is None else now
        if not np.isfinite(timestamp) or not np.isfinite(now) or not 0 <= now - timestamp <= timeout:
            raise TimeoutError(f'{name} timeout or invalid timestamp')

    def command(self, command, previous_q, state, last_command, period, now=None):
        # Revalidate mutable input arrays at the publishing boundary.
        command = ArmJointCommand(command.q, command.dq, command.kp, command.kd, command.tau_ff)
        self.validate(state.q, state_timestamp=state.timestamp, now=now)
        self.validate(command.q, previous_q)
        if state.dq is not None and np.any(np.abs(state.dq) > self.max_velocity_rad_s):
            raise ValueError('measured velocity limit')
        if np.any(np.abs(command.dq) > self.max_velocity_rad_s):
            raise ValueError('command velocity limit')
        if np.any(np.abs(command.q - previous_q) > self.max_velocity_rad_s * period + 1e-12):
            raise ValueError('reference velocity limit')
        self.check_age(last_command, self.command_timeout_s, 'command', now)
        return command
