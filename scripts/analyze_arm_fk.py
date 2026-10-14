#!/usr/bin/env python3
"""Offline FK analysis of a recorded G1 arm trajectory; never opens DDS."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from g1_dual_arm_teaching.kinematics import G1ArmFK
from g1_dual_arm_teaching.trajectory.storage import load_trajectory
from g1_dual_arm_teaching.kinematics.fk import analyze_trajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, help="trajectory NPZ path")
    parser.add_argument("--backend", choices=("ikpy", "mujoco", "pinocchio"), default="ikpy")
    parser.add_argument("--model", type=Path, help="optional MJCF or URDF path")
    parser.add_argument("--frame", choices=("pelvis", "torso"), default="pelvis")
    parser.add_argument("--q-ref", action="store_true", help="analyze command geometry, not measured motion")
    parser.add_argument("--output", type=Path, help="write JSON analysis")
    parser.add_argument("--compare-mjcf", action="store_true",
                        help="also compute the same LowState samples with the G1 MJCF and report model deltas")
    args = parser.parse_args()

    metadata, arrays = load_trajectory(args.trajectory)
    fk = G1ArmFK(args.backend, args.model)
    report = analyze_trajectory(arrays, fk, frame=args.frame, use_q_ref=args.q_ref)
    summary = {key: value for key, value in report.items() if key not in
               ("time_s", "position_m", "quaternion_wxyz", "linear_velocity_m_s", "linear_speed_m_s",
                "angular_step_rad", "angular_speed_rad_s", "sample_interval_s")}
    summary.update(trajectory_id=metadata["trajectory_id"], sample_count=len(arrays["timestamp"]),
                   duration_s=float(arrays["timestamp"][-1] - arrays["timestamp"][0]),
                   backend=args.backend, model=str(fk.model_path), model_sha256=fk.model_hash_sha256,
                   metadata_sha256=hashlib.sha256(
                       args.trajectory.with_suffix(".json").read_bytes()).hexdigest())
    if args.compare_mjcf:
        other = G1ArmFK("mujoco")
        position_error = {side: [] for side in ("left", "right")}
        orientation_error = {side: [] for side in ("left", "right")}
        q_key = "q_ref" if args.q_ref else "q_measured"
        for q_arm, q_waist in zip(arrays[q_key], arrays["waist_q"]):
            base = fk.compute(q_arm, q_waist, args.frame)
            comparison = other.compute(q_arm, q_waist, args.frame)
            for side in ("left", "right"):
                position_error[side].append(float(np.linalg.norm(
                    base[side].position_m - comparison[side].position_m)))
                relative = base[side].rotation.T @ comparison[side].rotation
                orientation_error[side].append(float(np.arccos(np.clip(
                    (np.trace(relative) - 1) / 2, -1, 1))))
        summary["model_comparison"] = dict(
            primary_model=str(fk.model_path), primary_sha256=fk.model_hash_sha256,
            comparison_model=str(other.model_path), comparison_sha256=other.model_hash_sha256,
            position_error_m={side: dict(max=float(np.max(values)), mean=float(np.mean(values)))
                              for side, values in position_error.items()},
            orientation_error_rad={side: dict(max=float(np.max(values)), mean=float(np.mean(values)))
                                   for side, values in orientation_error.items()},
            same_samples=True, sample_count=len(arrays["timestamp"]))
    if args.output:
        payload = dict(summary, time_s=report["time_s"].tolist(),
                       sample_interval_s=report["sample_interval_s"].tolist(),
                       position_m={s: v.tolist() for s, v in report["position_m"].items()},
                       quaternion_wxyz={s: v.tolist() for s, v in report["quaternion_wxyz"].items()},
                       linear_velocity_m_s={s: v.tolist() for s, v in report["linear_velocity_m_s"].items()},
                       linear_speed_m_s={s: v.tolist() for s, v in report["linear_speed_m_s"].items()},
                       angular_step_rad={s: v.tolist() for s, v in report["angular_step_rad"].items()},
                       angular_speed_rad_s={s: v.tolist() for s, v in report["angular_speed_rad_s"].items()})
        args.output.write_text(json.dumps(payload, allow_nan=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, allow_nan=False, indent=2))


if __name__ == "__main__":
    main()
