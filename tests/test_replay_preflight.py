import json
from pathlib import Path

import numpy as np
import pytest

from g1_dual_arm_teaching.replay_preflight import (
    ReadOnlyWindow, build_preflight_report, extract_lowstate,
)
from g1_dual_arm_teaching.kinematics import G1ArmFK
from g1_dual_arm_teaching.kinematics.fk import DEFAULT_URDF


ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY = ROOT / "data/trajectories/replay_candidates/g1_first_replay_candidate.npz"
REFERENCE = ROOT / "configs/g1_29dof_published_joint_reference.json"


class FakeLowState:
    def __init__(self, offset=0.0):
        self.motor_state = [type("Motor", (), {"q": offset + i * 0.001,
                                                "dq": i * 0.0001})()
                            for i in range(29)]
        self.imu_state = type("Imu", (), {"quaternion": [1.0, 0.0, 0.0, 0.0]})()


def test_extract_same_snapshot_motor12_through28():
    row = extract_lowstate(FakeLowState(), 9.5)
    assert row["lowstate_receive_time_monotonic"] == 9.5
    np.testing.assert_allclose(row["q_motor12_28"], np.arange(12, 29) * 0.001)
    np.testing.assert_allclose(row["dq_motor12_28"], np.arange(12, 29) * 0.0001)
    assert row["pelvis_imu_quaternion_wxyz"].tolist() == [1, 0, 0, 0]


def test_extract_rejects_incomplete_or_nonfinite_state():
    msg = FakeLowState()
    msg.motor_state = msg.motor_state[:28]
    with pytest.raises(ValueError):
        extract_lowstate(msg, 1.0)
    msg = FakeLowState()
    msg.motor_state[15].q = float("nan")
    with pytest.raises(ValueError):
        extract_lowstate(msg, 1.0)


def test_readonly_window_is_bounded_and_only_holds_numeric_state():
    capture = ReadOnlyWindow(capacity=2)
    capture.callback(FakeLowState())
    capture.callback(FakeLowState(1.0))
    capture.callback(FakeLowState(2.0))
    rows, dropped, error = capture.snapshot()
    assert len(rows) == 2 and dropped == 1 and error is None
    assert rows[-1]["q_motor12_28"][3] == pytest.approx(2.015)


def test_preflight_report_is_diagnostic_only_and_contains_start_connector():
    fk = G1ArmFK("mujoco")
    q = np.zeros(17)
    dq = np.zeros(17)
    rows = []
    for i in range(3):
        rows.append({"lowstate_receive_time_monotonic": 10.0 + i * 0.01,
                     "q_motor12_28": q.copy(), "dq_motor12_28": dq.copy(),
                     "pelvis_imu_quaternion_wxyz": np.array([1., 0., 0., 0.])})
    report = build_preflight_report(rows, TRAJECTORY,
        model_path=fk.model_path, reference_path=REFERENCE, backend="mujoco",
        limits_model_path=DEFAULT_URDF,
        connector_duration_s=5.0)
    assert report["hardware_execute_enabled"] is False
    assert report["publisher_created"] is False
    assert report["execution_authorized"] is False
    assert report["start_comparison"]["static_status"] == "UNASSESSED"
    assert report["start_comparison"]["max_abs_error_rad"] > 0
    assert report["connector"]["status"] == "OFFLINE_DIAGNOSTIC_ONLY"
    assert report["connector"]["assumptions"]["mechanical_space_checked"] is False
    assert report["connector"]["within_urdf_position_range"]
    assert len(report["model_limit_margins"]) == 14
    assert all(x["hardware_limit_status"] == "UNKNOWN"
               for x in report["model_limit_margins"].values())
    assert report["future_replay_log_fields"]["write_duration_s"] is None
    assert report["future_replay_log_fields"]["tracking_error_rad"] is None
    assert report["timing"]["lowstate_age_at_diagnostic_snapshot_s"] >= 0
    assert report["timing"]["fk_diagnostic_processing_duration_s"] >= 0


def test_diagnostic_static_threshold_does_not_authorize_execution():
    fk = G1ArmFK("mujoco")
    rows = [{"lowstate_receive_time_monotonic": 1.0 + i * 0.01,
             "q_motor12_28": np.zeros(17), "dq_motor12_28": np.zeros(17),
             "pelvis_imu_quaternion_wxyz": np.array([1., 0., 0., 0.])} for i in range(3)]
    report = build_preflight_report(rows, TRAJECTORY, model_path=fk.model_path,
        reference_path=REFERENCE, backend="mujoco",
        limits_model_path=DEFAULT_URDF,
        static_dq_threshold_rad_s=0.01, static_q_drift_threshold_rad=0.001)
    assert report["start_comparison"]["static_status"] == "BELOW_DIAGNOSTIC_THRESHOLDS"
    assert report["start_comparison"]["static_threshold_is_execution_permission"] is False
    assert report["execution_authorized"] is False


def test_preflight_age_uses_supplied_snapshot_time_not_report_completion():
    fk = G1ArmFK("mujoco")
    receipt = 20.0
    rows = [{"lowstate_receive_time_monotonic": receipt + i * .01,
             "q_motor12_28": np.zeros(17), "dq_motor12_28": np.zeros(17),
             "pelvis_imu_quaternion_wxyz": np.array([1., 0., 0., 0.])} for i in range(3)]
    snapshot = receipt + .025
    report = build_preflight_report(rows, TRAJECTORY, model_path=fk.model_path,
        reference_path=REFERENCE, backend="mujoco", diagnostic_snapshot_time_monotonic=snapshot)
    assert report["timing"]["lowstate_age_at_diagnostic_snapshot_s"] == pytest.approx(.005)


def test_readonly_script_has_no_command_publisher_or_transport_imports():
    source = (ROOT / "scripts/replay_preflight_readonly.py").read_text()
    assert "ChannelPublisher" not in source
    assert "UnitreeTransport" not in source
    assert '"rt/lowstate"' in source

