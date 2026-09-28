import numpy as np
import pytest
from g1_dual_arm_teaching.state.robot_state import RobotState
from g1_dual_arm_teaching.utils.timing import ControlLoop


@pytest.mark.parametrize('q', [[0] * 13, [np.nan] * 14, [np.inf] * 14])
def test_invalid_state(q):
    with pytest.raises(ValueError):
        RobotState(q)


def test_state_does_not_refresh_explicit_old_timestamp():
    state = RobotState(np.zeros(14), timestamp=0.)
    assert state.timestamp == 0.


@pytest.mark.parametrize('frequency', [0, -1, np.nan, np.inf])
def test_invalid_frequency(frequency):
    with pytest.raises(ValueError):
        ControlLoop(frequency)
