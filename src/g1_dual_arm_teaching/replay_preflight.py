"""Read-only Replay preflight diagnostics. This module has no DDS dependency."""
from __future__ import annotations

import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from .kinematics import G1ArmFK
from .trajectory.replay import load_replay_trajectory
from .trajectory.replay_c2 import extrema
from .trajectory.start_transition import septic_hermite


ARM_IDS = tuple(range(15, 29))
WAIST_IDS = tuple(range(12, 15))


def extract_lowstate(message, receipt_time=None):
    """Take one immutable numeric snapshot of motor12..28 from LowState."""
    received = time.monotonic() if receipt_time is None else float(receipt_time)
    if not np.isfinite(received):
        raise ValueError("invalid LowState receipt time")
    motors = message.motor_state
    if len(motors) < 29:
        raise ValueError("LowState motor_state has fewer than 29 motors")
    q = np.asarray([motors[i].q for i in range(12, 29)], dtype=float)
    dq = np.asarray([motors[i].dq for i in range(12, 29)], dtype=float)
    if q.shape != (17,) or dq.shape != (17,) or not np.isfinite([q, dq]).all():
        raise ValueError("LowState motor12..28 q/dq is incomplete or nonfinite")
    imu = np.asarray(message.imu_state.quaternion, dtype=float)
    if imu.shape != (4,) or not np.isfinite(imu).all():
        raise ValueError("LowState IMU quaternion is incomplete or nonfinite")
    return {"lowstate_receive_time_monotonic": received,
            "q_motor12_28": q, "dq_motor12_28": dq,
            "pelvis_imu_quaternion_wxyz": imu}


class ReadOnlyWindow:
    """Bounded thread-safe holder for the latest LowState samples."""
    def __init__(self, capacity=4096):
        if type(capacity) is not int or not 2 <= capacity <= 30000:
            raise ValueError("capacity must be in [2, 30000]")
        import threading
        self.capacity = capacity
        self._lock = threading.Lock()
        self._rows = []
        self.dropped = 0
        self.error = None

    def callback(self, message):
        try:
            row = extract_lowstate(message)
            with self._lock:
                if len(self._rows) == self.capacity:
                    self._rows.pop(0)
                    self.dropped += 1
                self._rows.append(row)
        except Exception as exc:
            with self._lock:
                self.error = repr(exc)

    def snapshot(self):
        with self._lock:
            return list(self._rows), self.dropped, self.error


def _model_limits(urdf_path, reference_path):
    root = ET.parse(urdf_path).getroot()
    result = {}
    from .kinematics.fk import ARM_JOINTS
    for motor_id, joint_name in zip(ARM_IDS, ARM_JOINTS):
        joint = next((x for x in root.findall("joint") if x.get("name") == joint_name), None)
        limit = None if joint is None else joint.find("limit")
        if limit is None or limit.get("lower") is None or limit.get("upper") is None:
            raise ValueError(f"URDF position limit unavailable for motor{motor_id}/{joint_name}")
        result[motor_id] = {"joint": joint_name, "min": float(limit.get("lower")),
                            "max": float(limit.get("upper")), "source": "URDF model only"}
    reference = json.loads(Path(reference_path).read_text(encoding="utf-8"))
    public = {int(row["motor_id"]): row for row in reference["joints"]}
    for motor_id in ARM_IDS:
        result[motor_id]["published_reference"] = {
            "min": public[motor_id]["position_min"], "max": public[motor_id]["position_max"],
            "source_url": reference["source"]["article_url"]}
        result[motor_id]["hardware_limit_status"] = "UNKNOWN"
    return result


def _connector_diagnostics(start, target, duration, model_limits, fk, waist_q):
    zero = np.zeros(14)
    coeff = septic_hermite(start, zero, zero, zero, target, zero, zero, zero, duration)
    mins = np.empty(14)
    maxs = np.empty(14)
    peaks = {name: np.zeros(14) for name in ("velocity", "acceleration", "jerk")}
    for axis, p in enumerate(coeff):
        mins[axis], maxs[axis] = extrema(p)
        for order, name in ((1, "velocity"), (2, "acceleration"), (3, "jerk")):
            lo, hi = extrema(p, order)
            peaks[name][axis] = max(abs(lo), abs(hi)) / duration**order
    times = np.linspace(0.0, duration, max(3, int(np.ceil(duration / 0.01)) + 1))
    samples = np.asarray([
        [np.polynomial.polynomial.polyval(t / duration, p) for p in coeff]
        for t in times])
    steps = np.max(np.abs(np.diff(samples, axis=0)), axis=0)
    tcp_paths = {side: {"position_m": [], "quaternion_wxyz": []} for side in ("left", "right")}
    for q in samples:
        poses = fk.compute(q, waist_q, frame="pelvis")
        for side in ("left", "right"):
            tcp_paths[side]["position_m"].append(poses[side].position_m)
            tcp_paths[side]["quaternion_wxyz"].append(poses[side].quaternion_wxyz)
    tcp = {}
    for side, data in tcp_paths.items():
        positions = np.asarray(data["position_m"])
        quaternions = np.asarray(data["quaternion_wxyz"])
        position_steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        position_speeds = position_steps / np.diff(times)
        dots = np.clip(np.abs(np.sum(quaternions[1:] * quaternions[:-1], axis=1)), 0.0, 1.0)
        tcp[side] = {"start_position_m": positions[0].tolist(),
            "end_position_m": positions[-1].tolist(),
            "position_min_m": np.min(positions, axis=0).tolist(),
            "position_max_m": np.max(positions, axis=0).tolist(),
            "max_sampled_position_step_m": float(np.max(position_steps)),
            "max_sampled_linear_speed_m_s": float(np.max(position_speeds)),
            "max_sampled_orientation_step_rad": float(np.max(2.0 * np.arccos(dots)))}
    limits_ok = all(mins[i] >= model_limits[mid]["min"] and maxs[i] <= model_limits[mid]["max"]
                    for i, mid in enumerate(ARM_IDS))
    published_ok = all(mins[i] >= model_limits[mid]["published_reference"]["min"] and
                       maxs[i] <= model_limits[mid]["published_reference"]["max"]
                       for i, mid in enumerate(ARM_IDS))
    return {"status": "OFFLINE_DIAGNOSTIC_ONLY",
            "assumptions": {"start_velocity_acceleration_jerk": "zero",
                            "target_velocity_acceleration_jerk": "zero",
                            "duration_s": duration,
                            "step_estimate": "10 ms diagnostic sampling; not actual Write schedule",
                            "mechanical_space_checked": False},
            "position_min_rad": mins.tolist(), "position_max_rad": maxs.tolist(),
            "peak_abs_velocity_rad_s": peaks["velocity"].tolist(),
            "peak_abs_acceleration_rad_s2": peaks["acceleration"].tolist(),
            "peak_abs_jerk_rad_s3": peaks["jerk"].tolist(),
            "max_sampled_step_rad_at_10ms": steps.tolist(),
            "distance_to_urdf_lower_rad": [float(mins[i] - model_limits[mid]["min"])
                for i, mid in enumerate(ARM_IDS)],
            "distance_to_urdf_upper_rad": [float(model_limits[mid]["max"] - maxs[i])
                for i, mid in enumerate(ARM_IDS)],
            "distance_to_published_lower_rad": [float(mins[i] - model_limits[mid]["published_reference"]["min"])
                for i, mid in enumerate(ARM_IDS)],
            "distance_to_published_upper_rad": [float(model_limits[mid]["published_reference"]["max"] - maxs[i])
                for i, mid in enumerate(ARM_IDS)],
            "within_urdf_position_range": bool(limits_ok),
            "within_published_reference_position_range": bool(published_ok),
            "tcp_proxy_pelvis_frame_m": tcp,
            "tcp_definition": "uncalibrated wrist-yaw link origin; not tool center or collision proof"}


def build_preflight_report(rows, trajectory_path, *, model_path, reference_path,
                           backend="ikpy", limits_model_path=None, connector_duration_s=5.0,
                           static_dq_threshold_rad_s=None,
                           static_q_drift_threshold_rad=None,
                           diagnostic_snapshot_time_monotonic=None):
    """Build diagnostic report from captured rows; never grants execution permission."""
    if not np.isfinite(connector_duration_s) or not 0 < connector_duration_s <= 300:
        raise ValueError("connector diagnostic duration must be in (0, 300] seconds")
    metadata, arrays = load_replay_trajectory(trajectory_path)
    if len(rows) < 2:
        raise ValueError("at least two LowState samples are required")
    receipt = np.asarray([r["lowstate_receive_time_monotonic"] for r in rows], dtype=float)
    q17 = np.asarray([r["q_motor12_28"] for r in rows], dtype=float)
    dq17 = np.asarray([r["dq_motor12_28"] for r in rows], dtype=float)
    if (q17.shape != (len(rows), 17) or dq17.shape != q17.shape or
            not all(np.isfinite(x).all() for x in (receipt, q17, dq17))):
        raise ValueError("captured sample shape/data invalid")
    dt = np.diff(receipt)
    strictly_increasing = bool(np.all(dt > 0))
    if not strictly_increasing:
        raise ValueError("LowState receipt times are not strictly increasing")
    q_arm, dq_arm = q17[-1, 3:], dq17[-1, 3:]
    candidate_start = np.asarray(arrays["q_measured"][0], dtype=float)
    start_error = q_arm - candidate_start
    from .kinematics.fk import DEFAULT_URDF
    limits_path = Path(limits_model_path or DEFAULT_URDF)
    limits = _model_limits(limits_path, reference_path)
    margins = {}
    for i, motor_id in enumerate(ARM_IDS):
        lim = limits[motor_id]
        margins[str(motor_id)] = {"joint": lim["joint"],
            "distance_to_urdf_lower_rad": float(q_arm[i] - lim["min"]),
            "distance_to_urdf_upper_rad": float(lim["max"] - q_arm[i]),
            "distance_to_published_lower_rad": float(q_arm[i] - lim["published_reference"]["min"]),
            "distance_to_published_upper_rad": float(lim["published_reference"]["max"] - q_arm[i]),
            "hardware_limit_status": "UNKNOWN"}
    fk_started = time.monotonic()
    fk = G1ArmFK(backend=backend, model_path=model_path)
    poses = fk.compute(q_arm, q17[-1, :3], frame="pelvis")
    diag_threshold = static_dq_threshold_rad_s
    if diag_threshold is not None and (not np.isfinite(diag_threshold) or diag_threshold <= 0):
        raise ValueError("diagnostic dq threshold must be positive finite")
    drift_threshold = static_q_drift_threshold_rad
    if drift_threshold is not None and (not np.isfinite(drift_threshold) or drift_threshold <= 0):
        raise ValueError("diagnostic q drift threshold must be positive finite")
    max_abs_dq = np.max(np.abs(dq_arm), axis=0)
    arm_q_samples = q17[:, 3:]
    q_drift = np.max(np.abs(arm_q_samples - arm_q_samples[0]), axis=0)
    if diag_threshold is None or drift_threshold is None:
        static = "UNASSESSED"
    else:
        static = ("BELOW_DIAGNOSTIC_THRESHOLDS"
                  if np.all(max_abs_dq <= diag_threshold) and np.all(q_drift <= drift_threshold)
                  else "ABOVE_DIAGNOSTIC_THRESHOLDS")
    connector = _connector_diagnostics(q_arm, candidate_start, connector_duration_s,
                                       limits, fk, q17[-1, :3])
    fk_finished = time.monotonic()
    snapshot_time = (time.monotonic() if diagnostic_snapshot_time_monotonic is None
                     else float(diagnostic_snapshot_time_monotonic))
    if not np.isfinite(snapshot_time) or snapshot_time < receipt[-1]:
        raise ValueError("diagnostic snapshot time must be finite and not before latest LowState receipt")
    return {"report_type": "READ_ONLY_REPLAY_PREFLIGHT",
        "read_only": True, "subscriber_only": True, "publisher_created": False,
        "hardware_execute_enabled": False,
        "trajectory": {"path": str(Path(trajectory_path)),
            "trajectory_id": metadata.get("trajectory_id"), "sample_count": int(len(arrays["q_measured"])),
            "start_q_motor15_28": candidate_start.tolist()},
        "sampling": {"sample_count": len(rows), "duration_s": float(receipt[-1]-receipt[0]),
            "strictly_increasing": strictly_increasing, "min_interval_s": float(np.min(dt)),
            "median_interval_s": float(np.median(dt)), "max_interval_s": float(np.max(dt)),
            "dropped_oldest_due_to_capacity": 0},
        "start_comparison": {"motor_ids": list(ARM_IDS), "error_q_minus_candidate_rad": start_error.tolist(),
            "max_abs_error_rad": float(np.max(np.abs(start_error))),
            "max_abs_error_motor_id": int(ARM_IDS[int(np.argmax(np.abs(start_error)))]),
            "q_measured_rad": q_arm.tolist(), "dq_measured_rad_s": dq_arm.tolist(),
            "max_abs_dq_over_window_rad_s": max_abs_dq.tolist(),
            "arm_q_drift_over_window_rad": q_drift.tolist(),
            "static_status": static,
            "static_thresholds": {"max_abs_dq_rad_s": diag_threshold,
                                   "max_q_drift_rad": drift_threshold},
            "static_threshold_is_execution_permission": False},
        "waist_measured": {"motor_ids": list(WAIST_IDS),
            "q_rad": q17[-1, :3].tolist(), "dq_rad_s": dq17[-1, :3].tolist()},
        "model": {"backend": backend, "path": str(Path(model_path)),
            "sha256": fk.model_hash_sha256, "frame": "pelvis", "tcp_definition":
            "wrist-yaw link origin plus configured offset; uncalibrated proxy",
            "hardware_limits_verified": False},
        "model_limit_source": {"type": "URDF position limits", "path": str(limits_path)},
        "model_limit_margins": margins,
        "fk_tcp_proxy": {side: {"position_m": pose.position_m.tolist(),
            "quaternion_wxyz": pose.quaternion_wxyz.tolist(), "frame": pose.frame}
            for side, pose in poses.items()},
        "connector": connector,
        "timing": {"latest_lowstate_receive_time_monotonic": float(receipt[-1]),
            "diagnostic_snapshot_time_monotonic": snapshot_time,
            "lowstate_age_at_diagnostic_snapshot_s": float(snapshot_time - receipt[-1]),
            "fk_diagnostic_processing_duration_s": float(fk_finished - fk_started)},
        "future_replay_log_fields": {"q_ref_motor15_28": None,
            "q_measured_motor15_28": q_arm.tolist(), "dq_measured_motor15_28": dq_arm.tolist(),
            "control_time_monotonic": None, "lowstate_receive_time_monotonic": float(receipt[-1]),
            "lowstate_age_s": float(snapshot_time - receipt[-1]),
            "write_duration_s": None, "tracking_error_rad": None},
        "not_assessed": ["mechanical workspace/collision", "actual connector execution",
            "hardware limits", "dynamic gains", "dynamic speed/acceleration/jerk permissions",
            "independent stop condition", "DDS Write duration and bound"],
        "execution_authorized": False}
