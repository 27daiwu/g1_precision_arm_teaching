#!/usr/bin/env python3
"""Read-only LowState Replay preflight. Creates a subscriber, never a publisher."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from g1_dual_arm_teaching.replay_preflight import ReadOnlyWindow, build_preflight_report
from g1_dual_arm_teaching.kinematics.fk import DEFAULT_MJCF, DEFAULT_URDF


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="DDS interface, e.g. eth0")
    parser.add_argument("--trajectory", type=Path,
                        default=Path("data/trajectories/replay_candidates/g1_first_replay_candidate.npz"))
    parser.add_argument("--duration", type=float, default=10.0,
                        help="read-only observation duration, seconds (0, 300]")
    parser.add_argument("--capacity", type=int, default=30000)
    parser.add_argument("--backend", choices=("ikpy", "pinocchio", "mujoco"), default="ikpy")
    parser.add_argument("--model", type=Path, default=None,
                        help="FK model; defaults to official URDF except for MuJoCo backend")
    parser.add_argument("--limits-model", type=Path, default=DEFAULT_URDF,
                        help="URDF used only to report model position limits")
    parser.add_argument("--connector-duration", type=float, default=5.0,
                        help="offline connector diagnostic duration; not an execution plan")
    parser.add_argument("--static-dq-threshold", type=float, default=None,
                        help="optional diagnostic threshold only; does not grant execution permission")
    parser.add_argument("--static-q-drift-threshold", type=float, default=None,
                        help="optional diagnostic window drift threshold; not execution permission")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if not 0 < args.duration <= 300:
        parser.error("--duration must be in (0, 300]")
    model_path = args.model or (DEFAULT_MJCF if args.backend == "mujoco" else DEFAULT_URDF)
    capture = ReadOnlyWindow(args.capacity)
    subscriber = None
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    try:
        ChannelFactoryInitialize(0, args.interface)
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(capture.callback, 10)
        print("READ_ONLY: subscribed to rt/lowstate; no command publisher is created", flush=True)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        print("Preflight interrupted; analyzing samples collected so far", flush=True)
    finally:
        if subscriber is not None:
            subscriber.Close()
    rows, dropped, callback_error = capture.snapshot()
    diagnostic_snapshot_time = time.monotonic()
    if len(rows) < 2:
        raise RuntimeError(f"need at least two valid LowState samples; received {len(rows)}")
    report = build_preflight_report(rows, args.trajectory, model_path=model_path,
        reference_path=Path("configs/g1_29dof_published_joint_reference.json"),
        backend=args.backend, limits_model_path=args.limits_model,
        connector_duration_s=args.connector_duration,
        static_dq_threshold_rad_s=args.static_dq_threshold,
        static_q_drift_threshold_rad=args.static_q_drift_threshold,
        diagnostic_snapshot_time_monotonic=diagnostic_snapshot_time)
    report["sampling"]["dropped_oldest_due_to_capacity"] = dropped
    report["callback_error"] = callback_error
    report["captured_at_utc"] = datetime.now(timezone.utc).isoformat()
    rendered = json.dumps(report, allow_nan=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered, flush=True)
    if callback_error:
        raise RuntimeError(f"LowState callback error: {callback_error}")


if __name__ == "__main__":
    main()
