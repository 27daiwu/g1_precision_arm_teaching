import numpy as np
import pytest

from g1_dual_arm_teaching.kinematics import (G1ArmFK, compare_shadow,
    compare_relative_plan_to_recording)
from g1_dual_arm_teaching.kinematics.fk import analyze_trajectory
from scripts.capture_fk_readonly import lowstate_row
from g1_dual_arm_teaching.trajectory.replay import ReplayLimits
from g1_dual_arm_teaching.trajectory.replay_c2 import RelativeC2ReplayPlan


@pytest.fixture(scope="module")
def fk():
    return G1ArmFK("mujoco")


def test_model_mapping_and_named_frames(fk):
    assert fk.model.nq == 36
    assert fk.model.njnt == 30
    assert fk._qpos["left_shoulder_pitch_joint"] == 22
    assert fk._qpos["right_shoulder_pitch_joint"] == 29
    poses = fk.compute(np.zeros(14), np.zeros(3), "pelvis")
    assert set(poses) == {"left", "right"}
    assert poses["left"].frame == "left_wrist_yaw_link"
    assert poses["right"].frame == "right_wrist_yaw_link"
    assert poses["left"].position_m.shape == (3,)
    assert poses["left"].quaternion_wxyz.shape == (4,)


def test_waist_and_relative_frames_are_explicit(fk):
    q = np.zeros(14)
    pelvis = fk.compute(q, np.zeros(3), "pelvis")
    torso_position = fk.data.xpos[fk._torso_id].copy()
    torso_rotation = fk.data.xmat[fk._torso_id].reshape(3, 3).copy()
    torso = fk.compute(q, np.zeros(3), "torso")
    np.testing.assert_allclose(pelvis["left"].position_m,
                               torso_position + torso_rotation @ torso["left"].position_m,
                               atol=1e-12)
    np.testing.assert_allclose(pelvis["left"].rotation,
                               torso_rotation @ torso["left"].rotation, atol=1e-12)
    changed = fk.compute(q, np.array([0.2, 0.0, 0.0]), "pelvis")
    assert np.linalg.norm(changed["left"].position_m - pelvis["left"].position_m) > 1e-4


def test_input_validation_and_world_frame_rejected(fk):
    with pytest.raises(ValueError):
        fk.compute(np.zeros(13))
    with pytest.raises(ValueError):
        fk.compute(np.full(14, np.nan))
    with pytest.raises(ValueError):
        fk.compute(np.zeros(14), frame="world")


def test_shadow_uses_geometric_measured_and_reference_inputs(fk):
    q_measured = np.zeros(14)
    q_ref = np.zeros(14)
    q_ref[3] = 0.2
    comparison = compare_shadow(fk, q_measured, q_ref, np.zeros(3))
    assert comparison.position_error_m["left"] > 0
    assert comparison.provenance.startswith("geometric FK")


def test_trajectory_analysis_uses_measured_q_and_real_timestamps(fk):
    arrays = {"timestamp": np.array([10.0, 10.03, 10.09]),
              "q_measured": np.zeros((3, 14)),
              "q_ref": np.full((3, 14), 0.3),
              "waist_q": np.zeros((3, 3))}
    report = analyze_trajectory(arrays, fk)
    assert report["source_joint_array"] == "q_measured"
    np.testing.assert_allclose(report["time_s"], [0.0, 0.03, 0.09])
    assert report["sampling"]["max_interval_s"] == pytest.approx(0.06)
    assert len(report["joint_change"]["arm_max_abs_step_rad_by_motor15_28"]) == 14
    assert report["live_tracking_validation"] is False
    with pytest.raises(ValueError):
        analyze_trajectory({**arrays, "timestamp": np.array([1.0, 1.0, 2.0])}, fk)


def test_readonly_capture_extracts_sdk_indices_without_command_fields():
    class Motor:
        def __init__(self, q):
            self.q = q
            self.dq = -q

    class Message:
        motor_state = [Motor(float(i)) for i in range(29)]
        imu_state = type("Imu", (), {"quaternion": [1.0, 0.0, 0.0, 0.0]})()

    timestamp, q, dq, waist, imu = lowstate_row(Message(), 4.5)
    assert timestamp == 4.5
    np.testing.assert_array_equal(q, np.arange(15, 29))
    np.testing.assert_array_equal(dq, -np.arange(15, 29))
    np.testing.assert_array_equal(waist, [12, 13, 14])
    np.testing.assert_array_equal(imu, [1, 0, 0, 0])


def test_ikpy_urdf_fk_when_optional_backend_is_installed():
    pytest.importorskip("ikpy.chain")
    urdf_fk = G1ArmFK("ikpy")
    zero = np.zeros(14)
    pelvis = urdf_fk.compute(zero, np.zeros(3), "pelvis")
    torso = urdf_fk.compute(zero, np.zeros(3), "torso")
    assert pelvis["left"].frame == "left_wrist_yaw_link"
    assert np.linalg.norm(pelvis["left"].position_m - torso["left"].position_m) > 0.04
    shifted = urdf_fk.compute(zero, np.array([0.2, 0.0, 0.0]), "pelvis")
    assert np.linalg.norm(shifted["left"].position_m - pelvis["left"].position_m) > 1e-4


def test_relative_fk_shadow_reports_that_tcp_path_is_not_preserved(fk):
    timestamp=np.array([0.,.5,1.])
    q=np.zeros((3,14));q[1,0]=.005;q[2,0]=.01
    arrays={"timestamp":timestamp,"cycle_timestamp":timestamp+.001,
        "q_measured":q,"dq_measured":np.zeros_like(q),"q_ref":q.copy(),
        "waist_q":np.zeros((3,3)),"torso_roll_pitch":np.zeros((3,2)),
        "arm_motor_ids":np.arange(15,29),"waist_motor_ids":np.arange(12,15)}
    metadata={"sample_count":3,"duration_s":1.,"data_integrity":"PASS",
        "start_q":q[0].tolist(),"end_q":q[-1].tolist()}
    limits=ReplayLimits((-2.,)*14,(2.,)*14,2.,100.,.1,.6,.1,.1,.1,.1,5.,.1)
    plan=RelativeC2ReplayPlan(metadata,arrays,limits,1.,jerk_limit=1e6,
        jerk_jump_limit=1e6,acquire_q=np.full(14,.1),motion_scale=.5)
    report=compare_relative_plan_to_recording(fk,plan,np.array([.1,0.,0.]))
    assert report["sample_count"]==3
    assert "does not preserve" in report["provenance"]
    assert report["position_error_m"]["left"]["max"]>0
