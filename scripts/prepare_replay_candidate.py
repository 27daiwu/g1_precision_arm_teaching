#!/usr/bin/env python3
"""Create a provenance-bound candidate slice without modifying its recording."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import uuid

import numpy as np


ARRAY_FIELDS = ("timestamp", "cycle_timestamp", "q_measured", "dq_measured",
                "q_ref", "waist_q", "torso_roll_pitch")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--start-s", type=float, required=True,
                        help="source time relative to first recording sample")
    parser.add_argument("--end-s", type=float, default=None,
                        help="exclusive time bound; defaults to recording end")
    parser.add_argument("--output-dir", type=Path,
                        default=Path("data/trajectories/replay_candidates"))
    parser.add_argument("--display-name", default=None)
    args = parser.parse_args()
    recording = args.recording.resolve()
    source_json = recording.with_suffix(".json")
    if not recording.is_file() or not source_json.is_file():
        parser.error("recording NPZ and matching JSON are both required")
    if not np.isfinite(args.start_s) or args.start_s < 0:
        parser.error("--start-s must be finite and nonnegative")
    source_meta = json.loads(source_json.read_text(encoding="utf-8"))
    with np.load(recording, allow_pickle=False) as archive:
        source = {key: archive[key].copy() for key in archive.files}
    if not np.array_equal(source.get("arm_motor_ids"), np.arange(15, 29)):
        raise ValueError("source arm motor mapping must be 15..28")
    if not np.array_equal(source.get("waist_motor_ids"), np.arange(12, 15)):
        raise ValueError("source waist motor mapping must be 12..14")
    timestamps = np.asarray(source.get("timestamp"), dtype=float)
    if timestamps.ndim != 1 or len(timestamps) < 2 or not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
        raise ValueError("source timestamps must be finite and strictly increasing")
    relative = timestamps - timestamps[0]
    end_s = float(relative[-1]) if args.end_s is None else args.end_s
    if not np.isfinite(end_s) or end_s <= args.start_s or end_s > relative[-1]:
        parser.error("--end-s must be greater than start and no later than recording end")
    first = int(np.searchsorted(relative, args.start_s, side="left"))
    last = int(np.searchsorted(relative, end_s, side="right"))
    if last - first < 3:
        raise ValueError("selected window must contain at least three samples")
    arrays = {key: source[key][first:last].copy() for key in ARRAY_FIELDS}
    arrays["arm_motor_ids"] = source["arm_motor_ids"].copy()
    arrays["waist_motor_ids"] = source["waist_motor_ids"].copy()
    n = len(arrays["timestamp"])
    if any(not np.isfinite(value).all() for value in arrays.values()):
        raise ValueError("candidate slice contains nonfinite data")
    for key, width in (("q_measured", 14), ("dq_measured", 14), ("q_ref", 14),
                       ("waist_q", 3), ("torso_roll_pitch", 2)):
        if arrays[key].shape != (n, width):
            raise ValueError(f"source field {key} has invalid shape")
    dt = np.diff(arrays["timestamp"])
    duration = float(arrays["timestamp"][-1] - arrays["timestamp"][0])
    trajectory_id = uuid.uuid4().hex
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.output_dir / f"replay_candidate_{trajectory_id}"
    npz_path, json_path, manifest_path = (stem.with_suffix(s) for s in (".npz", ".json", ".manifest.json"))
    metadata = {
        "schema_version": 2,
        "trajectory_id": trajectory_id,
        "session_id": source_meta.get("session_id"),
        "segment_index": 1,
        "previous_segment_id": None,
        "start_q": arrays["q_measured"][0].tolist(),
        "end_q": arrays["q_measured"][-1].tolist(),
        "connection_delta_q": None,
        "connection_max_abs_delta_q": None,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "recording_started_at": source_meta.get("recording_started_at"),
        "display_name": args.display_name or f"Replay candidate {args.start_s:.3f}-{end_s:.3f}s",
        "termination_reason": "OFFLINE_SOURCE_SLICE",
        "sample_count": n,
        "duration_s": duration,
        "actual_frequency_hz": (n-1)/duration,
        "max_sample_interval_s": float(np.max(dt)),
        "min_sample_interval_s": float(np.min(dt)),
        "sampling_interval_count": int(len(dt)),
        "gap_threshold_s": source_meta.get("gap_threshold_s", 0.1),
        "sampling_gap_count": int(np.count_nonzero(dt > source_meta.get("gap_threshold_s", 0.1))),
        "duplicate_timestamp_count": 0,
        "nonmonotonic_timestamp_count": 0,
        "invalid_sample_count": 0,
        "data_integrity": "PASS",
        "arm_motor_ids": list(range(15, 29)),
        "waist_motor_ids": [12, 13, 14],
        "timestamp_basis": source_meta.get("timestamp_basis"),
        "measured_source": source_meta.get("measured_source"),
        "q_ref_source": source_meta.get("q_ref_source"),
        "candidate_source_trajectory_id": source_meta.get("trajectory_id"),
        "candidate_status": "OFFLINE_NOT_AUTHORIZED",
        "replay_implemented": False,
        "replay_hardware_authorized": False,
    }
    with npz_path.open("xb") as stream:
        np.savez(stream, **arrays)
    with json_path.open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")
    manifest = {
        "manifest_version": 1,
        "candidate_id": trajectory_id,
        "status": "OFFLINE_CANDIDATE_NOT_AUTHORIZED",
        "hardware_execute_enabled": False,
        "candidate": {"npz": npz_path.name, "npz_sha256": sha256(npz_path),
                      "metadata": json_path.name, "metadata_sha256": sha256(json_path)},
        "source": {"npz_path": str(recording), "npz_sha256": sha256(recording),
                   "metadata_path": str(source_json), "metadata_sha256": sha256(source_json),
                   "trajectory_id": source_meta.get("trajectory_id"),
                   "source_sample_indices_half_open": [first, last],
                   "requested_relative_time_s": [args.start_s, end_s],
                   "actual_relative_time_s": [float(relative[first]), float(relative[last-1])],
                   "slice_fields": list(ARRAY_FIELDS), "original_untouched": True},
        "authorization": {"dynamic_replay_approved": False,
                          "reason": "Candidate slice requires live Preflight, hardware parameter approval, mechanical-space review, independent stop confirmation, and explicit user authorization."},
    }
    with manifest_path.open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")
    print(json.dumps({"npz": str(npz_path), "metadata": str(json_path),
                      "manifest": str(manifest_path), "candidate_id": trajectory_id,
                      "sample_count": n, "actual_duration_s": duration}, indent=2))


if __name__ == "__main__":
    main()
