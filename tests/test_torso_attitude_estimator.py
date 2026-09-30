import numpy as np
import pytest

from g1_dual_arm_teaching.control.torso_attitude_estimator import TorsoAttitudeEstimator
from scripts.analyze_torso_attitude_estimator import reference_matrix


def test_zero_waist_preserves_pelvis():
    q = np.array([0.9, 0.1, -0.2, 0.3])
    result = TorsoAttitudeEstimator().estimate(q, 0, 0, 0)
    np.testing.assert_allclose(result.torso_quaternion_wxyz, q / np.linalg.norm(q), atol=1e-15)


@pytest.mark.parametrize('joint,axis', [(1, 0), (2, 1)])
def test_single_waist_axis(joint, axis):
    waist = np.zeros(3)
    waist[joint] = 0.4
    result = TorsoAttitudeEstimator().estimate([1, 0, 0, 0], *waist)
    expected = np.zeros(4)
    expected[0] = np.cos(0.2)
    expected[axis + 1] = np.sin(0.2)
    np.testing.assert_allclose(result.torso_quaternion_wxyz, expected, atol=1e-15)


def test_combined_chain_against_independent_matrices():
    q = np.array([0.7, -0.3, 0.2, 0.4])
    waist = [0.35, -0.42, 0.29]
    result = TorsoAttitudeEstimator().estimate(q, *waist)
    np.testing.assert_allclose(result.rotation_matrix, reference_matrix(q, waist), atol=1e-14)


def test_quaternion_sign_equivalence_and_continuity():
    q = np.array([0.7, -0.3, 0.2, 0.4])
    estimator = TorsoAttitudeEstimator()
    a = estimator.estimate(q, 0.3, -0.2, 0.4)
    b = estimator.estimate(-q, 0.3, -0.2, 0.4)
    np.testing.assert_allclose(a.rotation_matrix, b.rotation_matrix, atol=1e-14)
    np.testing.assert_allclose(a.torso_quaternion_wxyz, b.torso_quaternion_wxyz, atol=1e-14)
    np.testing.assert_allclose([a.torso_roll, a.torso_pitch, a.torso_yaw],
                               [b.torso_roll, b.torso_pitch, b.torso_yaw], atol=1e-14)


@pytest.mark.parametrize('quaternion,joints', [
    ([0, 0, 0, 0], [0, 0, 0]),
    ([1e-15, 0, 0, 0], [0, 0, 0]),
    ([np.nan, 0, 0, 0], [0, 0, 0]),
    ([np.inf, 0, 0, 0], [0, 0, 0]),
    ([1, 0, 0, 0], [np.nan, 0, 0]),
    ([1, 0, 0, 0], [0, np.inf, 0]),
    ([1, 0, 0, 0], [0, 0, np.nan]),
])
def test_invalid_frame_does_not_change_continuity_state(quaternion, joints):
    estimator = TorsoAttitudeEstimator()
    before = estimator.estimate([1, 0, 0, 0], 0, 0, 0)
    with pytest.raises(ValueError):
        estimator.estimate(quaternion, *joints)
    after = estimator.estimate([-1, 0, 0, 0], 0, 0, 0)
    np.testing.assert_array_equal(before.torso_quaternion_wxyz, after.torso_quaternion_wxyz)
