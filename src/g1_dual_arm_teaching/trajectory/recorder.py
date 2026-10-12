"""Read-only, bounded measured trajectories. No command or replay interface."""
import json
from pathlib import Path
import queue
import threading
import uuid

import numpy as np

from ..control.torso_attitude_estimator import TorsoAttitudeEstimator
from ..utils.naming import filename_timestamp, wall_now


class TrajectoryRecorder:
    """A session-wide fixed sample budget; stopped slices are immutable.

    Persistence runs on one worker, never on the command/release thread.
    Schema version is the extension point for future offline consumers.
    """
    def __init__(self, directory='data/trajectories', max_samples=30000,
                 max_segments=128, gap_threshold_s=.1, context=None):
        if max_samples < 1 or max_segments < 1 or not np.isfinite(gap_threshold_s) or gap_threshold_s <= 0:
            raise ValueError('invalid recording limits')
        self.directory = Path(directory)
        self.buffer = np.empty((max_samples, 49), dtype=np.float64)
        self.max_segments = max_segments
        self.gap_threshold = gap_threshold_s
        self.context = dict(context or {})
        self.session_id = uuid.uuid4().hex
        self.session = self.session_id
        self.used = self.segments = 0
        self.active = None
        self.results = []
        self.previous_segment = None
        self.jobs = queue.Queue(maxsize=max_segments)
        self.worker = None
        self.closed = False
        self.estimator = TorsoAttitudeEstimator()

    def start(self, mode):
        if mode != 'DRAG' or self.active is not None or self.closed:
            return False
        if self.used == len(self.buffer) or self.segments == self.max_segments:
            print(f'RECORDING_REJECTED = session limit reached: samples={self.used}/{len(self.buffer)}, '
                  f'segments={self.segments}/{self.max_segments}; existing data retained, no overwrite', flush=True)
            return False
        if self.worker is None:
            self.worker = threading.Thread(target=self._persist, daemon=True)
            self.worker.start()
        self.segments += 1
        started_at = wall_now()
        previous_id = None if self.previous_segment is None else self.previous_segment['trajectory_id']
        self.active = dict(start=self.used, segment_index=self.segments,
                           trajectory_id=uuid.uuid4().hex,
                           previous_segment_id=previous_id,
                           start_q=None,
                           recording_started_at=started_at.isoformat(),
                           display_name=started_at.strftime('Trajectory %Y-%m-%d %H-%M-%S %z'),
                           duplicate_count=0,
                           gap_count=0, invalid_count=0, nonmonotonic_count=0)
        print(f'RECORDING_STARTED = {self.segments}; trajectory_id={self.active["trajectory_id"]}', flush=True)
        return True

    def sample(self, state, q_ref, cycle_timestamp):
        if self.active is None:
            return
        try:
            q, dq, ref, waist = (np.asarray(v, dtype=float) for v in
                                  (state.q, state.dq, q_ref, state.waist_q))
            if any(v.shape != shape or not np.isfinite(v).all() for v, shape in
                   ((q, (14,)), (dq, (14,)), (ref, (14,)), (waist, (3,)))):
                raise ValueError('missing/nonfinite joint data')
            if not np.isfinite(state.timestamp) or not np.isfinite(cycle_timestamp) or cycle_timestamp < state.timestamp:
                raise ValueError('invalid monotonic timestamp')
            attitude = self.estimator.estimate(state.imu_quaternion_wxyz, *waist)
            row = np.r_[state.timestamp, cycle_timestamp, q, dq, ref, waist,
                        attitude.torso_roll, attitude.torso_pitch]
            if not np.isfinite(row).all():
                raise ValueError('nonfinite recording data')
        except Exception:
            self.active['invalid_count'] += 1
            self.stop('INVALID_DATA')
            return
        if self.used > self.active['start']:
            dt = state.timestamp - self.buffer[self.used-1, 0]
            if dt == 0:
                self.active['duplicate_count'] += 1
                return
            if dt < 0 or cycle_timestamp < self.buffer[self.used-1, 1]:
                self.active['nonmonotonic_count'] += 1
                self.stop('NONMONOTONIC_TIME')
                return
            if dt > self.gap_threshold:
                self.active['gap_count'] += 1
        self.buffer[self.used] = row
        self.used += 1
        if self.active['start_q'] is None:
            self.active['start_q'] = q.copy().tolist()
        if self.used == len(self.buffer):
            self.stop('BUFFER_FULL')

    def stop(self, reason):
        if self.active is None:
            return
        segment = self.active
        self.active = None
        end_q = (self.buffer[self.used-1, 2:16].copy().tolist()
                 if self.used > segment['start'] else segment['start_q'])
        segment.update(end=self.used, end_q=end_q, termination_reason=reason)
        if (self.previous_segment is not None and segment['start_q'] is not None
                and self.previous_segment['end_q'] is not None):
            delta = (np.asarray(segment['start_q']) - np.asarray(self.previous_segment['end_q'])).tolist()
            segment['connection_delta_q'] = delta
            segment['connection_max_abs_delta_q'] = float(np.max(np.abs(delta)))
            print(f'RECORDING_CONNECTION = previous={segment["previous_segment_id"]}; '
                  f'max_abs_delta={segment["connection_max_abs_delta_q"]:.6f} rad (diagnostic only)', flush=True)
        else:
            segment['connection_delta_q'] = None
            segment['connection_max_abs_delta_q'] = None
        self.previous_segment = dict(trajectory_id=segment['trajectory_id'], end_q=end_q)
        self.jobs.put_nowait(segment)  # At most max_segments jobs in the session.
        print(f'RECORDING_STOPPED = {segment["segment_index"]}; reason={reason}; saving asynchronously; '
              f'samples={self.used}/{len(self.buffer)}, segments={self.segments}/{self.max_segments}; '
              'recorded data retained', flush=True)

    def _persist(self):
        while True:
            segment = self.jobs.get()
            try:
                if segment is None:
                    return
                data = self.buffer[segment['start']:segment['end']]
                n = len(data)
                dt = np.diff(data[:, 0])
                duration = float(data[-1, 0]-data[0, 0]) if n > 1 else 0.
                created_at = wall_now()
                meta = dict(schema_version=2, trajectory_id=segment['trajectory_id'],
                            session_id=self.session_id, segment_index=segment['segment_index'],
                            previous_segment_id=segment['previous_segment_id'],
                            start_q=segment['start_q'], end_q=segment['end_q'],
                            connection_delta_q=segment['connection_delta_q'],
                            connection_max_abs_delta_q=segment['connection_max_abs_delta_q'],
                            created_at=created_at.isoformat(),
                            recording_started_at=segment['recording_started_at'],
                            display_name=segment['display_name'],
                            termination_reason=segment['termination_reason'],
                            sample_count=n, duration_s=duration,
                            actual_frequency_hz=(n-1)/duration if duration > 0 else None,
                            max_sample_interval_s=float(dt.max()) if len(dt) else None,
                            min_sample_interval_s=float(dt.min()) if len(dt) else None,
                            sampling_interval_count=len(dt),
                            gap_threshold_s=self.gap_threshold,
                            sampling_gap_count=segment['gap_count'],
                            duplicate_timestamp_count=segment['duplicate_count'],
                            nonmonotonic_timestamp_count=segment['nonmonotonic_count'],
                            invalid_sample_count=segment['invalid_count'],
                            data_integrity=('PASS' if not any(segment[k] for k in
                                ('duplicate_count', 'nonmonotonic_count', 'invalid_count', 'gap_count')) else 'WARN'),
                            session_sample_capacity=len(self.buffer), context=self.context,
                            session_segment_capacity=self.max_segments,
                            arm_motor_ids=list(range(15, 29)), waist_motor_ids=[12, 13, 14],
                            array_fields=dict(timestamp='LowState monotonic receive time',
                                cycle_timestamp='host monotonic time when control iteration sampled state',
                                q_measured='measured LowState q, motors 15..28',
                                dq_measured='measured LowState dq, motors 15..28',
                                q_ref='same-cycle arm controller reference, motors 15..28',
                                waist_q='measured LowState q, motors 12..14',
                                torso_roll_pitch='estimated from this LowState IMU quaternion and waist_q'),
                            timestamp_basis='host monotonic LowState receive timestamp',
                            measured_source='same LowState snapshot: motor_state.q/dq',
                            q_ref_source='same-cycle arm teaching reference; not measured data',
                            units=dict(q='rad', dq='rad/s', time='s'), replay_implemented=False)
                self.directory.mkdir(parents=True, exist_ok=True)
                stamp = filename_timestamp(created_at)
                stem = self.directory / f'trajectory_{segment["trajectory_id"]}_{stamp}'
                npz_path, json_path = self._unique_pair(stem)
                with npz_path.open('xb') as stream:
                    np.savez(stream, timestamp=data[:, 0], cycle_timestamp=data[:, 1],
                             q_measured=data[:, 2:16], dq_measured=data[:, 16:30],
                             q_ref=data[:, 30:44], waist_q=data[:, 44:47],
                             torso_roll_pitch=data[:, 47:49],
                             arm_motor_ids=np.arange(15, 29), waist_motor_ids=np.arange(12, 15))
                with json_path.open('x', encoding='utf-8') as stream:
                    json.dump(meta, stream, ensure_ascii=False, allow_nan=False, indent=2)
                self.results.append(dict(trajectory_id=segment['trajectory_id'], npz=str(npz_path),
                                         metadata=str(json_path), status='SAVED'))
                print(f'TRAJECTORY_SAVED = {npz_path}', flush=True)
            except Exception as exc:
                self.results.append(dict(status='SAVE_FAILED', error=str(exc)))
                print(f'TRAJECTORY_SAVE_FAILED = {exc}', flush=True)
            finally:
                self.jobs.task_done()

    def finish(self, reason):
        self.stop(reason)
        self.closed = True
        if self.worker is not None:
            self.jobs.join()  # Called only after release.
            self.jobs.put(None)
            self.worker.join()

    @staticmethod
    def _unique_pair(stem):
        candidate, suffix = stem, 0
        while True:
            npz_path, json_path = Path(f'{candidate}.npz'), Path(f'{candidate}.json')
            if not npz_path.exists() and not json_path.exists():
                return npz_path, json_path
            suffix += 1
            candidate = Path(f'{stem}_{suffix}')

