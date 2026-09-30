#!/usr/bin/env python3
"""Offline analysis of raw body IMU LowState captures; never connects to DDS."""

import argparse
import json
from pathlib import Path

import numpy as np

REQUIRED = ('monotonic_receipt_time', 'quaternion', 'rpy', 'gyroscope',
            'accelerometer', 'tick', 'waist_q', 'waist_dq')
PELVIS_FIXED_LABELS = {'pelvis_fixed_torso_roll': (1, 0),
                       'pelvis_fixed_torso_pitch': (2, 1)}


def vector_stats(values, percentiles=True):
    if not len(values):
        return None
    result = dict(mean=np.mean(values, axis=0).tolist(), std=np.std(values, axis=0).tolist())
    if percentiles:
        result.update(p95=np.percentile(values, 95, axis=0).tolist(),
                      p99=np.percentile(values, 99, axis=0).tolist())
    return result


def correlation(a, b):
    if len(a) < 3 or np.std(a) < 1e-10 or np.std(b) < 1e-10:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def motion_series(t, rpy, gyro, waist):
    angle = np.unwrap(rpy, axis=0)
    vectors = {'rpy': angle, 'gyro': gyro, 'waist_q': waist}
    ranges = {key: {'min': np.min(value, axis=0).tolist(),
                    'max': np.max(value, axis=0).tolist(),
                    'peak_to_peak': np.ptp(value, axis=0).tolist()}
              for key, value in vectors.items()}
    baseline_count = max(1, min(len(t) // 10, int(0.5 / np.median(np.diff(t)))))
    baseline = np.median(angle[:baseline_count], axis=0)
    excursion = angle - baseline
    joint_corr = [[correlation(waist[:, i], angle[:, j]) for j in range(3)] for i in range(3)]
    # Resample to 20 ms bins and differentiate across 100 ms to suppress repeated DDS frames.
    edges = np.arange(t[0], t[-1], 0.02)
    if len(edges) >= 7:
        sampled_angle = np.column_stack([np.interp(edges, t, angle[:, i]) for i in range(3)])
        sampled_gyro = np.column_stack([np.interp(edges, t, gyro[:, i]) for i in range(3)])
        rate = (sampled_angle[5:] - sampled_angle[:-5]) / 0.1
        gyro_aligned = sampled_gyro[2:-3]
        rate_corr = [[correlation(rate[:, i], gyro_aligned[:, j]) for j in range(3)] for i in range(3)]
    else:
        rate_corr = [[None] * 3 for _ in range(3)]
    return dict(range=ranges, rpy_excursion_from_initial_median={
                    'positive': np.max(excursion, axis=0).tolist(),
                    'negative': np.min(excursion, axis=0).tolist()},
                waist_q_vs_rpy_correlation=joint_corr,
                waist_q_correlation_rows=['q12', 'q13', 'q14'],
                waist_q_correlation_columns=['roll', 'pitch', 'yaw'],
                smoothed_euler_rate_vs_gyro_correlation=rate_corr,
                smoothed_rate_window_s=0.1)


def external_pelvis_attitude(path, capture_t):
    with np.load(path, allow_pickle=False) as source:
        if not {'monotonic_receipt_time', 'rpy'} <= set(source.files):
            raise ValueError('external pelvis .npz requires monotonic_receipt_time and rpy')
        t, rpy = source['monotonic_receipt_time'], source['rpy']
    if t.ndim != 1 or rpy.shape != (len(t), 3) or len(t) < 2:
        raise ValueError('invalid external pelvis array shapes')
    if not np.isfinite(t).all() or not np.isfinite(rpy).all() or np.any(np.diff(t) <= 0):
        raise ValueError('invalid external pelvis samples')
    if t[0] > capture_t[0] or t[-1] < capture_t[-1]:
        raise ValueError('external pelvis attitude must cover the full capture on the same monotonic clock')
    angle = np.unwrap(rpy, axis=0)
    aligned = np.column_stack([np.interp(capture_t, t, angle[:, i]) for i in range(3)])
    return dict(source=str(path), aligned_rpy_peak_to_peak_rad=np.ptp(aligned, axis=0).tolist(),
                timestamp_basis='same-host monotonic clock required; check sensor calibration independently')


def analyze(path, pelvis_attitude=None):
    with np.load(path, allow_pickle=False) as source:
        missing = set(REQUIRED) - set(source.files)
        if missing:
            raise ValueError(f'missing fields: {sorted(missing)}')
        data = {key: source[key] for key in REQUIRED}
    t, rpy, gyro = (data[key] for key in ('monotonic_receipt_time', 'rpy', 'gyroscope'))
    n = len(t)
    if any(len(value) != n for value in data.values()):
        raise ValueError('inconsistent sample counts')
    if any(data[key].shape != (n, width) for key, width in
           (('quaternion', 4), ('rpy', 3), ('gyroscope', 3),
            ('accelerometer', 3), ('waist_q', 3), ('waist_dq', 3))):
        raise ValueError('invalid vector shape')
    if n < 2:
        raise ValueError('at least two callbacks required')
    if not all(np.isfinite(value).all() for value in data.values()):
        raise ValueError('nonfinite raw data')
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError('receipt times must increase')
    label = path.stem.split('_', 1)[-1]
    metadata_path = path.with_suffix('.json')
    metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    label = metadata.get('label', label)
    duplicate = {key: float(np.mean(np.all(data[key][1:] == data[key][:-1], axis=1)))
                 for key in ('quaternion', 'rpy', 'gyroscope')}
    norm = np.linalg.norm(data['quaternion'], axis=1)
    result = dict(file=str(path), label=label, sample_count=n,
                  duration_s=float(t[-1] - t[0]),
                  callback_interval_s=vector_stats(dt),
                  maximum_callback_gap_s=float(np.max(dt)),
                  duplicate_sample_ratio=duplicate,
                  tick_repeat_ratio=float(np.mean(data['tick'][1:] == data['tick'][:-1])),
                  quaternion_norm=vector_stats(norm))
    result['event_markers'] = metadata.get('event_markers', [])
    if label == 'static':
        result['static'] = dict(rpy=vector_stats(rpy), gyro=vector_stats(gyro),
                                accelerometer=vector_stats(data['accelerometer'], False))
    else:
        result['static'] = None
        result['static_note'] = 'Only explicitly labeled static field captures qualify as static noise measurements.'
    unwrapped = np.unwrap(rpy, axis=0)
    rate = np.diff(unwrapped, axis=0) / dt[:, None]
    gyro_mid = (gyro[1:] + gyro[:-1]) / 2
    correlations = np.full((3, 3), np.nan)
    for i in range(3):
        for j in range(3):
            if n > 2 and np.std(rate[:, i]) > 0 and np.std(gyro_mid[:, j]) > 0:
                correlations[i, j] = np.corrcoef(rate[:, i], gyro_mid[:, j])[0, 1]
    result['motion'] = dict(delta_rpy_rad=(unwrapped[-1] - unwrapped[0]).tolist(),
                            delta_waist_q_rad=(data['waist_q'][-1] - data['waist_q'][0]).tolist(),
                            peak_abs_gyro_xyz=np.max(np.abs(gyro), axis=0).tolist(),
                            euler_derivative_vs_gyro_xyz_correlation=[
                                [float(x) if np.isfinite(x) else None for x in row] for row in correlations],
                            correlation_rows=['roll_rate', 'pitch_rate', 'yaw_rate'],
                            correlation_columns=['gyro_x', 'gyro_y', 'gyro_z'],
                            limitation='Body angular velocity is not generally Euler angle derivative. Direction comparison is only approximate at small roll/pitch and low yaw/pitch excursions.',
                            **motion_series(t, rpy, gyro, data['waist_q']))
    result['update_rate_note'] = 'DDS callback rate and exact repeats cannot identify independent IMU generation rate or distinguish held from stale samples without a sensor timestamp or controlled response.'
    if label in PELVIS_FIXED_LABELS:
        joint, axis = PELVIS_FIXED_LABELS[label]
        motion = result['motion']
        q_range = motion['range']['waist_q']['peak_to_peak']
        imu_range = motion['range']['rpy']['peak_to_peak']
        other_joints = [i for i in range(3) if i != joint]
        result['pelvis_fixed_experiment'] = dict(
            target_joint=f'q{joint + 12}', target_imu_axis=('roll', 'pitch')[axis],
            target_joint_peak_to_peak_rad=q_range[joint],
            target_imu_peak_to_peak_rad=imu_range[axis],
            target_joint_vs_imu_correlation=motion['waist_q_vs_rpy_correlation'][joint][axis],
            coupling_joint_peak_to_peak_rad={f'q{i + 12}': q_range[i] for i in other_joints},
            gyro_xyz_range=motion['range']['gyro'],
            pelvis_world_attitude=(external_pelvis_attitude(pelvis_attitude, t)
                                   if pelvis_attitude else None),
            hardware_imu_link='UNCONFIRMED',
            interpretation='Independent pelvis fixation and clear target joint motion are required. Compare pelvis-corrected IMU orientation with joint motion; label and correlation alone do not establish the hardware IMU link.')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs='+', type=Path, help='raw .npz capture files')
    parser.add_argument('--pelvis-attitude', type=Path,
                        help='external .npz with monotonic_receipt_time and rpy on the same host clock; one capture only')
    args = parser.parse_args()
    if args.pelvis_attitude and len(args.captures) != 1:
        parser.error('--pelvis-attitude requires exactly one capture')
    for path in args.captures:
        if path.suffix != '.npz':
            parser.error('only .npz captures are accepted')
        print(json.dumps(analyze(path, args.pelvis_attitude), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
