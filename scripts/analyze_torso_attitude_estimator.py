#!/usr/bin/env python3
"""Replay saved LowState captures without importing robot SDK or opening DDS."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from g1_dual_arm_teaching.control.torso_attitude_estimator import TorsoAttitudeEstimator


def reference_matrix(pelvis, waist):
    w, x, y, z = pelvis / np.linalg.norm(pelvis)
    root = np.array(((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)),
                     (2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)),
                     (2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y))))
    a, b, c = waist
    rz = np.array(((np.cos(a), -np.sin(a), 0), (np.sin(a), np.cos(a), 0), (0, 0, 1)))
    rx = np.array(((1, 0, 0), (0, np.cos(b), -np.sin(b)), (0, np.sin(b), np.cos(b))))
    ry = np.array(((np.cos(c), 0, np.sin(c)), (0, 1, 0), (-np.sin(c), 0, np.cos(c))))
    return root @ rz @ rx @ ry


def analyze(path):
    with np.load(path, allow_pickle=False) as data:
        quaternion = data['quaternion']
        waist = data['waist_q']
        raw_rpy = data['rpy']
    if quaternion.ndim != 2 or quaternion.shape[1] != 4 or waist.shape != (len(quaternion), 3) or raw_rpy.shape != (len(quaternion), 3) or len(quaternion) == 0:
        raise ValueError(f'{path}: invalid capture shapes')
    estimator = TorsoAttitudeEstimator()
    estimated = np.empty((len(quaternion), 3))
    errors = np.empty(len(quaternion))
    for i, (pelvis, joints) in enumerate(zip(quaternion, waist)):
        attitude = estimator.estimate(pelvis, *joints)
        estimated[i] = (attitude.torso_roll, attitude.torso_pitch, attitude.torso_yaw)
        delta = attitude.rotation_matrix.T @ reference_matrix(pelvis, joints)
        errors[i] = np.arccos(np.clip((np.trace(delta)-1)/2, -1, 1))
    if not np.isfinite(raw_rpy).all():
        raise ValueError(f'{path}: nonfinite recorded RPY')
    raw_range = np.ptp(np.unwrap(raw_rpy, axis=0), axis=0)
    estimated_range = np.ptp(np.unwrap(estimated, axis=0), axis=0)
    return dict(file=str(path), sample_count=len(quaternion),
                raw_imu_rpy_p2p_rad=raw_range.tolist(),
                estimated_torso_rpy_p2p_rad=estimated_range.tolist(),
                max_orientation_error_rad=float(np.max(errors)),
                mean_orientation_error_rad=float(np.mean(errors)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs='+', type=Path)
    args = parser.parse_args()
    for path in args.captures:
        print(json.dumps(analyze(path), allow_nan=False))


if __name__ == '__main__':
    main()
