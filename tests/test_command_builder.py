import numpy as np
import pytest
from g1_dual_arm_teaching.control.command_builder import build_arm_command


@pytest.mark.parametrize('size', [0, 13, 15])
def test_dimensions(size):
    with pytest.raises(ValueError):
        build_arm_command(np.zeros(size))


@pytest.mark.parametrize('field', ['q', 'dq', 'kp', 'kd', 'tau_ff'])
@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
def test_nonfinite(field, bad):
    args = {'q': np.zeros(14), field: np.full(14, bad)}
    with pytest.raises(ValueError):
        build_arm_command(**args)


def test_feedforward_rejected():
    with pytest.raises(ValueError):
        build_arm_command(np.zeros(14), tau_ff=np.ones(14))
