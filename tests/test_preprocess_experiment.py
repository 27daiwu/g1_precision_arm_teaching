from pathlib import Path

import numpy as np
import pytest

from g1_dual_arm_teaching.kinematics.fk import DEFAULT_URDF
from g1_dual_arm_teaching.trajectory.preprocess_experiment import (
    compare_candidate_preprocessing, septic_time_warp,
)


ROOT = Path(__file__).resolve().parents[1]
CANDIDATE = ROOT / "data/trajectories/replay_candidates/g1_first_replay_candidate.npz"
PREFLIGHT = ROOT / "logs/replay_preflight/preflight.json"


def test_septic_warp_has_flat_c3_endpoints_and_is_monotone():
    u = np.linspace(0, 1, 1001)
    w = septic_time_warp(u)
    assert w[0] == 0 and w[-1] == 1
    assert np.all(np.diff(w) >= 0)


def test_preprocess_experiment_reports_real_seams_and_does_not_write_candidate():
    before = CANDIDATE.read_bytes()
    report = compare_candidate_preprocessing(CANDIDATE, PREFLIGHT,
        urdf_path=DEFAULT_URDF, duration_scales=(1., 5.),
        connector_duration_s=5., sample_period_s=.01)
    assert CANDIDATE.read_bytes() == before
    assert report["hardware_execute_enabled"] is False
    assert report["source"]["generated_trajectory_written"] is False
    assert report["source"]["candidate_role"] == "FROZEN_REGRESSION_BASELINE_ONLY"
    assert report["preflight_start_max_abs_error_rad"] == pytest.approx(.5925719738)
    assert report["original_c2"]["connector_matching_actual_start_jerk_peak_abs_velocity_rad_s"] > 1000
    assert report["experiments"][0]["start_seam_max_abs_q_v_a_j"] == pytest.approx([0]*4, abs=1e-8)
    assert report["experiments"][0]["end_seam_max_abs_q_v_a_j"] == pytest.approx([0]*4, abs=1e-8)
    assert report["experiments"][0]["trajectory_internal_jerk_jump_rad_s3"] > \
        report["original_c2"]["internal_jerk_jump_rad_s3"]
    assert report["experiments"][1]["trajectory_duration_s"] == pytest.approx(
        5 * report["source"]["duration_s"])
    assert report["limits"]["hardware_limit_status"] == "UNKNOWN"


def test_experiment_rejects_invalid_scales():
    with pytest.raises(ValueError):
        compare_candidate_preprocessing(CANDIDATE, PREFLIGHT, urdf_path=DEFAULT_URDF,
            duration_scales=(0.5,))
