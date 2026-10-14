#!/usr/bin/env python3
"""Offline-only relative Replay planner preview. Never initializes DDS."""
import argparse
import json
from pathlib import Path

import numpy as np

from g1_dual_arm_teaching.kinematics import G1ArmFK, compare_relative_plan_to_recording
from g1_dual_arm_teaching.trajectory.replay import ReplayLimits
from g1_dual_arm_teaching.trajectory.replay_c2 import RelativeReplaySpec


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("preflight", type=Path,
                        help="read-only Preflight JSON used only as an offline acquire snapshot")
    parser.add_argument("--constraints", type=Path, required=True,
                        help="explicit JSON: limits, jerk_limit, jerk_jump_limit, period_s")
    parser.add_argument("--start-index", type=int, required=True)
    parser.add_argument("--end-index", type=int, required=True,
                        help="exclusive sample index")
    parser.add_argument("--motion-scale", type=float, required=True)
    parser.add_argument("--speed-scale", type=float, required=True)
    parser.add_argument("--backend", choices=("ikpy", "mujoco", "pinocchio"), default="ikpy")
    parser.add_argument("--model", type=Path)
    args = parser.parse_args(argv)

    preflight = json.loads(args.preflight.read_text(encoding="utf-8"))
    if preflight.get("read_only") is not True or preflight.get("publisher_created") is not False:
        raise ValueError("input must be a subscriber-only read-only Preflight report")
    acquire_q = np.asarray(preflight["start_comparison"]["q_measured_rad"], dtype=float)
    acquire_waist = np.asarray(preflight["waist_measured"]["q_rad"], dtype=float)
    cfg = json.loads(args.constraints.read_text(encoding="utf-8"))
    limits = ReplayLimits(**cfg["limits"])
    period = float(cfg["period_s"])
    if not np.isfinite(period) or period <= 0:
        raise ValueError("period_s must be explicit and positive")
    spec = RelativeReplaySpec.load(
        args.trajectory, limits, args.speed_scale,
        jerk_limit=cfg["jerk_limit"], jerk_jump_limit=cfg["jerk_jump_limit"],
        motion_scale=args.motion_scale, start_index=args.start_index,
        end_index=args.end_index)
    plan = spec.bind(acquire_q)
    # C2ReplayPlan validates all 14 axes against the supplied explicit limits.
    sampled_steps = []
    sample_times = np.append(np.arange(0.0, plan.duration, period), plan.duration)
    for t0, t1 in zip(sample_times[:-1], sample_times[1:]):
        sampled_steps.append(float(np.max(np.abs(plan.sample(float(t1))[0] -
                                                 plan.sample(float(t0))[0]))))
    fk = G1ArmFK(args.backend, args.model)
    report = {
        "report_type": "OFFLINE_RELATIVE_REPLAY_PREVIEW",
        "hardware_execute_enabled": False,
        "dds_initialized": False,
        "publisher_created": False,
        "source_trajectory": str(args.trajectory),
        "trajectory_id": spec.metadata.get("trajectory_id"),
        "segment_indices_half_open": [args.start_index, args.end_index],
        "acquire_snapshot_provenance": "read-only Preflight; execution must bind a new same-LowState ACQUIRE snapshot",
        "acquire_q_motor15_28": plan.acquire_q.tolist(),
        "first_reference_exact_match": bool(np.array_equal(plan.sample(0.0)[0], plan.acquire_q)),
        "motion_scale": plan.motion_scale,
        "speed_scale": plan.speed_scale,
        "sample_count": int(len(plan.q)),
        "duration_s": plan.duration,
        "position_min_rad": plan.position_min.tolist(),
        "position_max_rad": plan.position_max.tolist(),
        "peak_abs_velocity_rad_s": plan.peak_velocity,
        "peak_abs_acceleration_rad_s2": plan.peak_acceleration,
        "peak_abs_jerk_rad_s3": plan.peak_jerk,
        "peak_abs_jerk_jump_rad_s3": plan.peak_jerk_jump,
        "max_sampled_write_step_rad": max(sampled_steps, default=0.0),
        "write_step_bound_at_cycle_timeout_rad": plan.peak_velocity * limits.cycle_timeout,
        "configured_write_step_limit_rad": limits.step,
        "checks_against_supplied_offline_limits": {
            "position": bool(np.all(plan.position_min >= limits.q_min) and
                              np.all(plan.position_max <= limits.q_max)),
            "velocity": bool(plan.peak_velocity <= limits.velocity),
            "acceleration": bool(plan.peak_acceleration <= limits.acceleration),
            "jerk": bool(plan.peak_jerk <= plan.jerk_limit),
            "jerk_jump": bool(plan.peak_jerk_jump <= plan.jerk_jump_limit),
            "cycle_timeout_write_step_budget": bool(plan.peak_velocity * limits.cycle_timeout <= limits.step),
            "actual_dds_write": False,
        },
        "fk_shadow": compare_relative_plan_to_recording(fk, plan, acquire_waist),
        "mechanical_space_collision_checked": False,
        "thresholds_are_hardware_approved": False,
    }
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
