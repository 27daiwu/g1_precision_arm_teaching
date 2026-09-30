#!/usr/bin/env python3
"""Read-only rt/lowstate capture for onsite IMU semantics experiments."""

import argparse
import json
import select
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

LABELS = ('static', 'whole_body_roll', 'whole_body_pitch',
          'torso_relative_roll', 'torso_relative_pitch',
          'pelvis_fixed_torso_roll', 'pelvis_fixed_torso_pitch')
EVENTS = frozenset(('LEFT', 'RIGHT', 'FORWARD', 'BACKWARD', 'CENTER'))


def parse_marker(line, timestamp):
    name = line.strip().upper()
    if name not in EVENTS:
        raise ValueError(f'unknown marker {name!r}; expected {", ".join(sorted(EVENTS))}')
    return dict(monotonic_timestamp=float(timestamp), event=name)


def summarize(data, label):
    t = data['monotonic_receipt_time']
    n = len(t)
    gaps = np.diff(t)
    duration = float(t[-1] - t[0]) if n > 1 else 0.0
    quat = data['quaternion']
    finite = {key: bool(np.isfinite(value).all()) for key, value in data.items()
              if np.issubdtype(value.dtype, np.number)}
    norms = np.linalg.norm(quat, axis=1) if n else np.array([])
    def stats(values):
        return dict(mean=float(np.mean(values)), std=float(np.std(values)),
                    p95=float(np.percentile(values, 95)),
                    p99=float(np.percentile(values, 99)), max=float(np.max(values))) if len(values) else None
    def repeats(key):
        values = data[key]
        return int(np.count_nonzero(np.all(values[1:] == values[:-1], axis=1)))
    return dict(label=label, sample_count=n, duration_s=duration,
                dds_callback_rate_hz=(n - 1) / duration if duration > 0 else None,
                finite_check=finite, all_finite=all(finite.values()),
                quaternion_norm_statistics=stats(norms), callback_gap_statistics_s=stats(gaps),
                exact_repeated_quaternion_count=repeats('quaternion'),
                exact_repeated_rpy_count=repeats('rpy'),
                exact_repeated_gyro_count=repeats('gyroscope'),
                tick_repeat_count=int(np.count_nonzero(data['tick'][1:] == data['tick'][:-1])),
                tick_repeat_ratio=float(np.mean(data['tick'][1:] == data['tick'][:-1])) if n > 1 else None,
                note='Repeated values do not establish independent IMU update rate or stale frames.')


class Capture:
    def __init__(self):
        self.lock = threading.Lock()
        self.rows = []
        self.error = None

    def callback(self, message):
        try:
            receipt = time.monotonic()
            imu = message.imu_state
            row = (receipt, tuple(imu.quaternion), tuple(imu.rpy),
                   tuple(imu.gyroscope), tuple(imu.accelerometer), int(message.tick),
                   tuple(message.motor_state[i].q for i in (12, 13, 14)),
                   tuple(message.motor_state[i].dq for i in (12, 13, 14)),
                   int(message.mode_pr), int(message.mode_machine))
            with self.lock:
                self.rows.append(row)
        except Exception as exc:
            with self.lock:
                self.error = repr(exc)

    def snapshot(self):
        with self.lock:
            return list(self.rows), self.error


def arrays(rows):
    fields = ('monotonic_receipt_time', 'quaternion', 'rpy', 'gyroscope',
              'accelerometer', 'tick', 'waist_q', 'waist_dq', 'mode_pr', 'mode_machine')
    widths = (None, 4, 3, 3, 3, None, 3, 3, None, None)
    result = {}
    for i, (field, width) in enumerate(zip(fields, widths)):
        dtype = np.uint32 if field == 'tick' else np.uint8 if field.startswith('mode_') else np.float64
        result[field] = np.asarray([row[i] for row in rows], dtype=dtype).reshape((-1, width) if width else (-1,))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('interface', help='DDS network interface, e.g. eth0')
    parser.add_argument('--label', required=True, choices=LABELS)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--output-dir', type=Path, default=Path('logs/body_imu_audit'))
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be a positive finite number')

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    capture = Capture()
    markers = []
    subscriber = None
    started = time.monotonic()
    try:
        ChannelFactoryInitialize(0, args.interface)
        subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        subscriber.Init(capture.callback, 10)
        print('Type LEFT, RIGHT, FORWARD, BACKWARD or CENTER then Enter to mark an event.', flush=True)
        next_display = started
        while time.monotonic() - started < args.duration:
            time.sleep(0.02)
            now = time.monotonic()
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line:
                    try:
                        marker = parse_marker(line, time.monotonic())
                        markers.append(marker)
                        print(f'MARKER {marker["event"]} at {marker["monotonic_timestamp"]:.6f}', flush=True)
                    except ValueError as exc:
                        print(exc, flush=True)
            if now >= next_display:
                rows, error = capture.snapshot()
                if error:
                    raise RuntimeError(f'LowState callback failed: {error}')
                if rows:
                    latest = rows[-1]
                    elapsed = rows[-1][0] - rows[0][0]
                    hz = (len(rows) - 1) / elapsed if elapsed > 0 else 0.0
                    print(f'RPY={latest[2]} gyro={latest[3]} q12/13/14={latest[6]} callback={hz:.1f} Hz', flush=True)
                next_display = now + 0.1
    except KeyboardInterrupt:
        print('Capture interrupted; saving received samples.', flush=True)
    finally:
        if subscriber is not None:
            subscriber.Close()
        rows, error = capture.snapshot()
        data = arrays(rows)
        metadata = summarize(data, args.label)
        metadata.update(interface=args.interface, callback_error=error,
                        captured_at_utc=datetime.now(timezone.utc).isoformat(),
                        source_topic='rt/lowstate', field_order='raw SDK order',
                        event_markers=markers,
                        marker_timestamp_note='Terminal line receipt on the capture host; may lag the physical action.')
        args.output_dir.mkdir(parents=True, exist_ok=True)
        stem = f'{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_{args.label}'
        path = args.output_dir / stem
        np.savez(path.with_suffix('.npz'), **data)
        path.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
        print(f'Saved {len(rows)} callbacks to {path.with_suffix(".npz")}', flush=True)
    if error:
        raise RuntimeError(f'LowState callback failed: {error}')


if __name__ == '__main__':
    main()
