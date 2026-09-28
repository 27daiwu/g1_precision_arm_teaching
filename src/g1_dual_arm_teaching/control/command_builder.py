"""Construct validated arm-only commands; gains come from controller config."""
import numpy as np
from ..sdk.arm_sdk_types import ArmJointCommand


def build_arm_command(q, dq=None, kp=None, kd=None, tau_ff=None):
    zero = np.zeros(14)
    return ArmJointCommand(q, zero if dq is None else dq,
                           zero if kp is None else kp, zero if kd is None else kd,
                           zero if tau_ff is None else tau_ff)
