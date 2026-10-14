#!/usr/bin/env python3
"""Compare offline endpoint-smoothing experiments; does not write trajectories."""
import argparse
import json
from pathlib import Path

from g1_dual_arm_teaching.kinematics.fk import DEFAULT_URDF
from g1_dual_arm_teaching.trajectory.preprocess_experiment import compare_candidate_preprocessing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("preflight", type=Path)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--duration-scales", type=float, nargs="+", default=(1., 5., 10.))
    parser.add_argument("--connector-duration", type=float, default=5.)
    parser.add_argument("--sample-period", type=float, default=.01)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = compare_candidate_preprocessing(args.trajectory, args.preflight,
        urdf_path=args.urdf, duration_scales=args.duration_scales,
        connector_duration_s=args.connector_duration,
        sample_period_s=args.sample_period)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rendered = json.dumps(report, allow_nan=False, indent=2) + "\n"
    args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
