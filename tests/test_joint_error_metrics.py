import numpy as np
import pytest
from g1_dual_arm_teaching.evaluation.joint_error_metrics import joint_error_metrics, repeatability_metrics


def test_error_and_repeatability():
    actual = np.array([[1] * 14, [3] * 14])
    metrics = joint_error_metrics(actual, np.zeros((2, 14)))
    np.testing.assert_allclose(metrics['mae_rad'], 2)
    np.testing.assert_allclose(metrics['rmse_rad'], np.sqrt(5))
    np.testing.assert_allclose(metrics['max_abs_rad'], 3)
    np.testing.assert_allclose(repeatability_metrics(actual)['std_rad'], np.sqrt(2))
    np.testing.assert_allclose(repeatability_metrics(actual)['peak_to_peak_rad'], 2)


@pytest.mark.parametrize('samples', [[], [[0] * 13], [[np.nan] * 14]])
def test_invalid_metrics(samples):
    with pytest.raises(ValueError):
        joint_error_metrics(samples, samples)
