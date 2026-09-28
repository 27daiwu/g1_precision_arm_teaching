import time
import numpy as np
import pytest
from g1_dual_arm_teaching.control.safety_guard import SafetyGuard
from g1_dual_arm_teaching.control.command_builder import build_arm_command
from g1_dual_arm_teaching.sdk.arm_sdk_types import JointState


def guard():
    return SafetyGuard([-1] * 14, [1] * 14)


@pytest.mark.parametrize('value', [np.nan, np.inf, -np.inf, 1.01, -1.01])
def test_invalid_q(value):
    with pytest.raises(ValueError):
        guard().validate(np.full(14, value))


def test_step_limit():
    with pytest.raises(ValueError, match='step'):
        guard().validate(np.full(14, 0.06), np.zeros(14))


@pytest.mark.parametrize('timestamp', [0, float('nan'), float('inf')])
def test_state_timeout(timestamp):
    with pytest.raises(TimeoutError):
        guard().validate(np.zeros(14), state_timestamp=timestamp)


def test_velocity_and_command_timeout():
    now = time.monotonic()
    state = JointState(np.zeros(14), now)
    with pytest.raises(ValueError, match='velocity'):
        guard().command(build_arm_command(np.zeros(14), dq=np.full(14, 2)), np.zeros(14), state, now, .02)
    with pytest.raises(TimeoutError, match='command'):
        guard().command(build_arm_command(np.zeros(14)), np.zeros(14), state, now - 1, .02)
