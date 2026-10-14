#!/usr/bin/env python3
"""Bounded read-only rt/lowstate capture for offline G1 arm FK."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time

import numpy as np


def lowstate_row(message, receipt_time):
    """Extract FK inputs from one LowState message without command access."""
    return (float(receipt_time),
            np.asarray([message.motor_state[i].q for i in range(15, 29)], dtype=float),
            np.asarray([message.motor_state[i].dq for i in range(15, 29)], dtype=float),
            np.asarray([message.motor_state[i].q for i in (12, 13, 14)], dtype=float),
            np.asarray(message.imu_state.quaternion, dtype=float))


class Capture:
    def __init__(self, capacity):
        self.capacity = capacity
        self.rows = []
        self.lock = threading.Lock()
        self.dropped = 0
        self.error = None

    def callback(self, message):
        try:
            row = lowstate_row(message, time.monotonic())
            if not all(np.isfinite(value).all() for value in row):
                raise ValueError("nonfinite LowState FK input")
            with self.lock:
                if len(self.rows) < self.capacity:
                    self.rows.append(row)
                else:
                    self.dropped += 1
        except Exception as exc:
            with self.lock:
                self.error = repr(exc)

    def snapshot(self):
        with self.lock:
            rows = list(self.rows)
            return rows, self.dropped, self.error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="DDS interface, e.g. eth0")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output-dir", type=Path, default=Path("logs/fk_readonly"))
    parser.add_argument("--max-samples", type=int, default=30000)
    args = parser.parse_args()
    if not np.isfinite(args.duration) or not 0 < args.duration <= 300:
        parser.error("--duration must be finite and in (0, 300] seconds")
    if not 1 <= args.max_samples <= 30000:
        parser.error("--max-samples must be in [1, 30000]")

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    capture = Capture(args.max_samples)
    subscriber = None
    started = time.monotonic()
    try:
        ChannelFactoryInitialize(0, args.interface)
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(capture.callback, 10)
        print("READ_ONLY: subscribed to rt/lowstate; no command publisher is created", flush=True)
        while time.monotonic() - started < args.duration:
            time.sleep(max(0.0, min(0.05, args.duration - (time.monotonic() - started))))
    except KeyboardInterrupt:
        print("Capture interrupted; retaining received samples", flush=True)
    finally:
        if subscriber is not None:
            subscriber.Close()
    rows, dropped, error = capture.snapshot()
    timestamp = np.asarray([r[0] for r in rows], dtype=float)
    arrays = {
        "timestamp": timestamp,
        "monotonic_receipt_time": timestamp,
        "q_measured": np.asarray([r[1] for r in rows], dtype=float).reshape((-1, 14)),
        "dq_measured": np.asarray([r[2] for r in rows], dtype=float).reshape((-1, 14)),
        "waist_q": np.asarray([r[3] for r in rows], dtype=float).reshape((-1, 3)),
        "pelvis_imu_quaternion_wxyz": np.asarray([r[4] for r in rows], dtype=float).reshape((-1, 4)),
        "arm_motor_ids": np.arange(15, 29, dtype=np.int32),
        "waist_motor_ids": np.arange(12, 15, dtype=np.int32),
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.output_dir / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_fk_readonly")
    suffix = 0
    while stem.with_suffix(".npz").exists() or stem.with_suffix(".json").exists():
        suffix += 1
        stem = args.output_dir / f"{datetime.now(timezone.utc):%Y%m%dT%H%M%S}_fk_readonly_{suffix}"
    np.savez(stem.with_suffix(".npz"), **arrays)
    gaps = np.diff(timestamp)
    metadata = dict(read_only=True, publisher_created=False, topic="rt/lowstate",
                    interface=args.interface, sample_count=len(timestamp), dropped_after_capacity=dropped,
                    callback_error=error, capture_duration_s=float(timestamp[-1] - timestamp[0]) if len(timestamp) > 1 else 0.0,
                    callback_rate_hz=float((len(timestamp) - 1) / (timestamp[-1] - timestamp[0])) if len(timestamp) > 1 else None,
                    max_callback_gap_s=float(np.max(gaps)) if len(gaps) else None,
                    arm_motor_ids=list(range(15, 29)), waist_motor_ids=[12, 13, 14],
                    fields={"q_measured": "LowState motor_state.q", "dq_measured": "LowState motor_state.dq",
                            "waist_q": "same LowState motor_state.q", "pelvis_imu_quaternion_wxyz": "same LowState imu_state.quaternion"},
                    captured_at_utc=datetime.now(timezone.utc).isoformat())
    stem.with_suffix(".json").write_text(json.dumps(metadata, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {len(timestamp)} read-only LowState samples to {stem}.npz", flush=True)
    if error:
        raise RuntimeError(error)


if __name__ == "__main__":
    main()
