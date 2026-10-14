"""Offline-only trajectory preprocessing experiments; never used by Replay."""
from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .replay import load_replay_trajectory
from .replay_c2 import C2Curve, extrema
from .start_transition import septic_hermite


def septic_time_warp(u):
    """Monotone C3 endpoint-flat map [0,1] -> [0,1]."""
    u = np.asarray(u, dtype=float)
    if not np.isfinite(u).all() or np.any(u < 0) or np.any(u > 1):
        raise ValueError("normalized time must be finite and in [0, 1]")
    return 35*u**4 - 84*u**5 + 70*u**6 - 20*u**7


def _warp_derivatives(u):
    w1 = 140*u**3 - 420*u**4 + 420*u**5 - 140*u**6
    w2 = 420*u**2 - 1680*u**3 + 2100*u**4 - 840*u**5
    w3 = 840*u - 5040*u**2 + 8400*u**3 - 4200*u**4
    return w1, w2, w3


def _curve_derivatives(curve, elapsed):
    if elapsed >= curve.duration:
        return curve.q[-1].copy(), np.zeros(14), np.zeros(14), np.zeros(14)
    i = max(0, min(len(curve.time)-2,
                   int(np.searchsorted(curve.time, elapsed, side="right") - 1)))
    h = curve.time[i+1] - curve.time[i]
    u = (elapsed - curve.time[i]) / h
    out = []
    for order in range(4):
        out.append(np.asarray([
            np.polynomial.polynomial.polyval(
                u, np.polynomial.polynomial.polyder(p, order)) / h**order
            for p in curve.coefficients[i]]))
    return tuple(out)


def _retimed_state(curve, elapsed, duration):
    u = elapsed / duration
    warp = float(septic_time_warp(u))
    w1, w2, w3 = (float(x) for x in _warp_derivatives(u))
    source_t = curve.duration * warp
    source_q, source_v, source_a, source_j = _curve_derivatives(curve, source_t)
    td = curve.duration / duration * w1
    tdd = curve.duration / duration**2 * w2
    tj = curve.duration / duration**3 * w3
    qd = source_v * td
    qdd = source_a * td**2 + source_v * tdd
    qj = source_j * td**3 + 3 * source_a * td * tdd + source_v * tj
    return source_q, qd, qdd, qj


def _peak_polynomial(coeff, duration):
    peaks = []
    for order in (1, 2, 3):
        peak = 0.0
        for p in coeff:
            lo, hi = extrema(p, order)
            peak = max(peak, abs(lo) / duration**order, abs(hi) / duration**order)
        peaks.append(float(peak))
    return peaks


def _source_jerk_jumps(curve):
    jumps = []
    for i in range(1, len(curve.coefficients)):
        hl = curve.time[i] - curve.time[i-1]
        hr = curve.time[i+1] - curve.time[i]
        jl = np.asarray([np.polynomial.polynomial.polyval(
            1., np.polynomial.polynomial.polyder(p, 3)) / hl**3
            for p in curve.coefficients[i-1]])
        jr = np.asarray([np.polynomial.polynomial.polyval(
            0., np.polynomial.polynomial.polyder(p, 3)) / hr**3
            for p in curve.coefficients[i]])
        jumps.append((float(curve.time[i]), jr-jl))
    return jumps


def _urdf_limits(path):
    from ..kinematics.fk import ARM_JOINTS
    root = ET.parse(path).getroot()
    result = []
    for name in ARM_JOINTS:
        joint = next((node for node in root.findall("joint") if node.get("name") == name), None)
        limit = None if joint is None else joint.find("limit")
        if limit is None or limit.get("lower") is None or limit.get("upper") is None:
            raise ValueError(f"URDF limit missing for {name}")
        result.append((float(limit.get("lower")), float(limit.get("upper"))))
    return np.asarray(result)


def _connector(start, end, end_jet, duration):
    zero = np.zeros(14)
    coeff = septic_hermite(start, zero, zero, zero, end,
                           end_jet[1], end_jet[2], end_jet[3], duration)
    qmin, qmax = np.empty(14), np.empty(14)
    for axis, p in enumerate(coeff):
        qmin[axis], qmax[axis] = extrema(p)
    peaks = _peak_polynomial(coeff, duration)
    return coeff, qmin, qmax, peaks


def _connector_samples(coeff, duration, sample_period):
    times = np.linspace(0., duration, int(np.ceil(duration / sample_period)) + 1)
    q = np.asarray([[np.polynomial.polynomial.polyval(t / duration, p)
                     for p in coeff] for t in times])
    return q


def _sampled_step(states):
    return np.max(np.abs(np.diff(states, axis=0)), axis=0)


def _summary(curve, duration_scale, connector_start,
             connector_duration, urdf_limits, sample_period):
    duration = curve.duration * duration_scale
    count = int(np.ceil(duration / sample_period)) + 1
    output_t = np.linspace(0., duration, count)
    states = [_retimed_state(curve, t, duration) for t in output_t]
    q, v, a, j = (np.asarray([state[k] for state in states]) for k in range(4))
    candidate_peaks = [float(np.max(np.abs(x))) for x in (v, a, j)]
    candidate_jerk_jump = 0.0
    for source_t, delta_j in _source_jerk_jumps(curve):
        u = source_t / curve.duration
        w1, _, _ = _warp_derivatives(u)
        output_jump = delta_j * (curve.duration / duration * w1)**3
        candidate_jerk_jump = max(candidate_jerk_jump, float(np.max(np.abs(output_jump))))

    endpoint_start = _retimed_state(curve, 0., duration)
    endpoint_end = _retimed_state(curve, duration, duration)
    connector_coeff, connector_min, connector_max, connector_peaks = _connector(
        connector_start, endpoint_start[0], endpoint_start, connector_duration)
    seam_start_left = np.zeros((4, 14))
    seam_start_right = np.asarray(endpoint_start)
    connector_end = np.asarray([
        np.asarray([np.polynomial.polynomial.polyval(
            1., np.polynomial.polynomial.polyder(p, order))
            for p in connector_coeff]) / connector_duration**order
        for order in range(4)])
    candidate_end = np.asarray(endpoint_end)
    seam_end_left = candidate_end
    seam_end_right = candidate_end.copy()
    seam_end_right[1:] = 0.0
    start_seam = np.max(np.abs(connector_end - seam_start_right), axis=1)
    end_seam = np.max(np.abs(seam_end_left - seam_end_right), axis=1)
    trajectory_limits = np.column_stack((
        np.minimum(curve.position_min, connector_min),
        np.maximum(curve.position_max, connector_max)))
    margins_low = trajectory_limits[:, 0] - urdf_limits[:, 0]
    margins_high = urdf_limits[:, 1] - trajectory_limits[:, 1]
    connector_t = np.linspace(0., connector_duration,
        int(np.ceil(connector_duration / sample_period)) + 1)
    connector_q = np.asarray([[np.polynomial.polynomial.polyval(
        t / connector_duration, p) for p in connector_coeff] for t in connector_t])
    joined_q = np.concatenate((connector_q, q[1:]), axis=0)
    return {
        "duration_scale": duration_scale,
        "trajectory_duration_s": duration,
        "connector_duration_s": connector_duration,
        "connector_plus_trajectory_duration_s": duration + connector_duration,
        "connector_peak_abs_velocity_rad_s": connector_peaks[0],
        "connector_peak_abs_acceleration_rad_s2": connector_peaks[1],
        "connector_peak_abs_jerk_rad_s3": connector_peaks[2],
        "trajectory_peak_abs_velocity_rad_s_sampled": candidate_peaks[0],
        "trajectory_peak_abs_acceleration_rad_s2_sampled": candidate_peaks[1],
        "trajectory_peak_abs_jerk_rad_s3_sampled": candidate_peaks[2],
        "trajectory_internal_jerk_jump_rad_s3": candidate_jerk_jump,
        "start_seam_max_abs_q_v_a_j": start_seam.tolist(),
        "end_seam_max_abs_q_v_a_j": end_seam.tolist(),
        "combined_peak_abs_velocity_rad_s": max(connector_peaks[0], candidate_peaks[0]),
        "combined_peak_abs_acceleration_rad_s2": max(connector_peaks[1], candidate_peaks[1]),
        "combined_peak_abs_jerk_rad_s3": max(connector_peaks[2], candidate_peaks[2]),
        "combined_max_sampled_write_step_rad": float(np.max(_sampled_step(joined_q))),
        "write_step_sample_period_s": sample_period,
        "position_range_min_rad_by_axis": trajectory_limits[:, 0].tolist(),
        "position_range_max_rad_by_axis": trajectory_limits[:, 1].tolist(),
        "minimum_urdf_lower_margin_rad": float(np.min(margins_low)),
        "minimum_urdf_upper_margin_rad": float(np.min(margins_high)),
        "within_urdf_position_limits": bool(np.all(margins_low >= 0) and np.all(margins_high >= 0)),
        "endpoint_start_q_v_a_j_max_abs": [float(np.max(np.abs(x))) for x in endpoint_start],
        "endpoint_end_q_v_a_j_max_abs": [float(np.max(np.abs(x))) for x in endpoint_end],
        "trajectory_internal_jerk_jump_method": "source polynomial one-sided jerk limits mapped through monotone time warp",
    }


def compare_candidate_preprocessing(trajectory_path, preflight_path, *,
                                    urdf_path, duration_scales=(1., 5., 10.),
                                    connector_duration_s=5., sample_period_s=.01):
    """Compare frozen C2 data with an endpoint-flat time-warp experiment.

    The returned prepared curve is diagnostic only and no trajectory artifact is
    written. Measured q geometry is not offset or rewritten.
    """
    if (not np.isfinite(connector_duration_s) or connector_duration_s <= 0 or
            not np.isfinite(sample_period_s) or sample_period_s <= 0):
        raise ValueError("connector duration and sample period must be positive")
    meta, arrays = load_replay_trajectory(trajectory_path)
    t = np.asarray(arrays["timestamp"], dtype=float)
    q = np.asarray(arrays["q_measured"], dtype=float)
    curve = C2Curve(t-t[0], q)
    report = json.loads(Path(preflight_path).read_text(encoding="utf-8"))
    start = np.asarray(report["start_comparison"]["q_measured_rad"], dtype=float)
    if start.shape != (14,) or not np.isfinite(start).all():
        raise ValueError("preflight report lacks valid measured start q")
    urdf = _urdf_limits(urdf_path)
    scales = tuple(float(scale) for scale in duration_scales)
    if not scales or any(not np.isfinite(scale) or scale < 1 for scale in scales):
        raise ValueError("duration scales must be finite values >= 1")
    rows = [_summary(curve, scale, start, connector_duration_s,
                     urdf, sample_period_s) for scale in scales]
    zero = np.zeros(14)
    raw_connector, raw_connector_min, raw_connector_max, raw_connector_peaks = _connector(
        start, curve.q[0], (curve.q[0], np.zeros(14), np.zeros(14), curve.start_jerk),
        connector_duration_s)
    raw_connection_q = _connector_samples(raw_connector, connector_duration_s, sample_period_s)
    raw_curve_t = np.linspace(0., curve.duration,
        int(np.ceil(curve.duration / sample_period_s)) + 1)
    raw_curve_q = np.asarray([curve.sample(float(t))[0] for t in raw_curve_t])
    raw_joined = np.concatenate((raw_connection_q, raw_curve_q[1:]), axis=0)
    raw_position_min = np.minimum(curve.position_min, raw_connector_min)
    raw_position_max = np.maximum(curve.position_max, raw_connector_max)
    raw_seam_end = np.asarray([curve.q[0], zero, zero, curve.start_jerk])
    raw_connector_end = np.asarray([
        np.asarray([np.polynomial.polynomial.polyval(
            1., np.polynomial.polynomial.polyder(p, order))
            for p in raw_connector]) / connector_duration_s**order
        for order in range(4)])
    raw_start_seam = np.max(np.abs(raw_connector_end - raw_seam_end), axis=1)
    raw_hold_seam = np.asarray([curve.q[-1], zero, zero, curve.end_jerk])
    raw_hold = raw_hold_seam.copy()
    raw_hold[1:] = zero
    raw_limits_ok = bool(np.all(raw_position_min >= urdf[:, 0]) and
                         np.all(raw_position_max <= urdf[:, 1]))
    return {
        "report_type": "OFFLINE_CANDIDATE_PREPROCESSING_EXPERIMENT",
        "hardware_execute_enabled": False,
        "source": {"trajectory_id": meta.get("trajectory_id"),
            "source_path": str(Path(trajectory_path)), "sample_count": int(len(q)),
            "duration_s": float(t[-1]-t[0]),
            "source_data_unchanged": True, "generated_trajectory_written": False,
            "candidate_role": "FROZEN_REGRESSION_BASELINE_ONLY"},
        "preflight_start_max_abs_error_rad": float(np.max(np.abs(start-curve.q[0]))),
        "preflight_start_error_rad_by_motor15_28": (start-curve.q[0]).tolist(),
        "original_c2": {"peak_abs_velocity_rad_s": curve.peak_velocity,
            "peak_abs_acceleration_rad_s2": curve.peak_acceleration,
            "peak_abs_jerk_rad_s3": curve.peak_jerk,
            "internal_jerk_jump_rad_s3": curve.peak_jerk_jump,
            "start_jerk_rad_s3_by_motor15_28": curve.start_jerk.tolist(),
            "peak_abs_start_jerk_motor_id": int(15+np.argmax(np.abs(curve.start_jerk))),
            "recorded_q_max_sample_step_rad": float(np.max(np.abs(np.diff(q, axis=0)))),
            "splice_start_seam_max_abs_q_v_a_j": raw_start_seam.tolist(),
            "splice_hold_seam_max_abs_q_v_a_j": np.max(np.abs(raw_hold_seam-raw_hold), axis=1).tolist(),
            "connector_matching_actual_start_jerk_peak_abs_velocity_rad_s": raw_connector_peaks[0],
            "connector_matching_actual_start_jerk_peak_abs_acceleration_rad_s2": raw_connector_peaks[1],
            "connector_matching_actual_start_jerk_peak_abs_jerk_rad_s3": raw_connector_peaks[2],
            "connector_plus_c2_max_sampled_write_step_rad": float(np.max(_sampled_step(raw_joined))),
            "connector_plus_c2_position_min_rad_by_axis": raw_position_min.tolist(),
            "connector_plus_c2_position_max_rad_by_axis": raw_position_max.tolist(),
            "connector_plus_c2_within_urdf_position_limits": raw_limits_ok},
        "preprocessing_method": {"type": "offline septic C3 time warp of existing C2 geometric path",
            "polynomial": "35u^4 - 84u^5 + 70u^6 - 20u^7",
            "properties": "monotonic; endpoint time derivatives 1..3 are zero; preserves q path but changes time profile",
            "internal_c2_jerk_jumps_remain": True,
            "connector_endpoint_jet": "matched to computed prepared trajectory start q/v/a/jerk",
            "hold_end_jet": "prepared trajectory end q/v/a/jerk joined to constant pose",
            "candidate_is_measured_recording": False},
        "experiments": rows,
        "limits": {"urdf_path": str(Path(urdf_path)),
            "hardware_limit_status": "UNKNOWN", "no_production_dynamic_thresholds_applied": True,
            "mechanical_workspace_checked": False,
            "sampled_write_step_is_offline_estimate": True,
            "connector_start_jet_assumption": "current observed q, zero velocity/acceleration/jerk; stationary status was not established"},
    }
