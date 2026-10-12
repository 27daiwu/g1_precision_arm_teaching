import json
import queue
import time
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from g1_dual_arm_teaching.dual_arm_teach import WAIST_KP, WAIST_PITCH_HOLD_BIAS, run_teach
from g1_dual_arm_teaching.sdk.arm_sdk_types import JointState
from g1_dual_arm_teaching.sdk.golden_diagnostics import SafetyAbort
from g1_dual_arm_teaching.trajectory.recorder import TrajectoryRecorder
from g1_dual_arm_teaching.trajectory.storage import load_trajectory
from g1_dual_arm_teaching.utils.naming import read_log_timestamp, timestamped_path


def make_state(timestamp, offset):
    return JointState(np.full(14, .2 + offset), timestamp, np.full(14, .3 + offset),
                      np.array([.01, .02, .03]), np.zeros(3),
                      imu_quaternion_wxyz=np.array([1., 0., 0., 0.]))


class RecordingController:
    period_s = .02
    waist_kp_by_axis = WAIST_KP
    waist_pitch_hold_bias = WAIST_PITCH_HOLD_BIAS
    static_hold_waist_kp = None

    def __init__(self, inbox):
        self.inbox, self.now, self.reads = inbox, 10., 0
        self.q_hold = np.zeros(17)
        self.max_delta = np.zeros(17)
        self.diagnostics = SimpleNamespace(writes=[], first_trigger=None)
        self.frames, self.events, self.released = [], [], False

    def clock(self): return self.now
    def _profiled_sleep(self, duration): self.now += duration
    def capture_upper_body_pose(self): pass
    def acquire_current_pose(self): pass
    def hold_current_pose(self, duration): pass
    def _state(self):
        self.reads += 1
        return make_state(self.now - .001, self.reads * .01)
    def set_teach_command(self, reference, kp): self.command = reference.copy(), kp.copy()
    def _frame(self, weight, phase):
        self.frames.append((weight, phase, *self.command))
        self.diagnostics.writes.append(dict(phase=phase, weight=weight, outcome='SUCCESS'))
        actions = {1: ['r'], 3: ['r'], 4: ['r'], 6: ['q']}
        for action in actions.get(len(self.frames), []): self.inbox.put(action)
    def emit(self, event): self.events.append(event)
    def release(self): self.released = True


def wait_saved(directory, count):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        files = list(Path(directory).glob('*.json'))
        if len(files) == count: return sorted(files)
        time.sleep(.01)
    raise AssertionError(f'expected {count} metadata files in {directory}')


def test_r_recording_saves_multiple_measured_drag_segments_without_replay(tmp_path):
    inbox = queue.Queue()
    inbox.put('f')
    controller = RecordingController(inbox)
    recorder = TrajectoryRecorder(tmp_path, max_samples=16,
                                  context={'body_attitude_closed_loop': True})
    run_teach(controller, inbox, recorder=recorder)
    files = wait_saved(tmp_path, 2)
    assert controller.released
    assert len(controller.frames) == 7
    assert all(frame[1] == 'TEACH' for frame in controller.frames)
    for metadata_path in files:
        metadata = json.loads(metadata_path.read_text())
        arrays = np.load(metadata_path.with_suffix('.npz'))
        assert metadata['schema_version'] == 2
        assert metadata['termination_reason'] in ('OPERATOR_STOP', 'OPERATOR_QUIT')
        assert metadata['sample_count'] > 0
        assert metadata['actual_frequency_hz'] > 0
        assert metadata['data_integrity'] == 'PASS'
        assert metadata['replay_implemented'] is False
        assert arrays['q_measured'].shape == arrays['dq_measured'].shape == (metadata['sample_count'], 14)
        assert arrays['q_ref'].shape == (metadata['sample_count'], 14)
        assert arrays['waist_q'].shape == (metadata['sample_count'], 3)
        assert arrays['torso_roll_pitch'].shape == (metadata['sample_count'], 2)
        np.testing.assert_array_equal(arrays['q_measured'], arrays['q_ref'])
        assert not np.array_equal(arrays['q_measured'], controller.frames[0][2])
        assert metadata['measured_source'].startswith('same LowState')
        assert metadata['q_ref_source'].startswith('same-cycle')
        assert metadata['context']['body_attitude_closed_loop'] is True
        assert metadata['trajectory_id'] in metadata_path.stem
        assert metadata['session_id'] == recorder.session_id
        assert metadata['display_name'].startswith('Trajectory ')
        assert datetime.fromisoformat(metadata['created_at']).utcoffset() is not None
        assert datetime.fromisoformat(metadata['recording_started_at']).utcoffset() is not None
        assert ':' not in metadata_path.name
        assert read_log_timestamp(metadata_path) is not None


def test_recorder_marks_duplicate_gap_and_retains_valid_samples(tmp_path):
    recorder = TrajectoryRecorder(tmp_path, max_samples=8, gap_threshold_s=.05)
    assert recorder.start('DRAG')
    q = np.zeros(14)
    recorder.sample(make_state(1., 0), q, 1.01)
    recorder.sample(make_state(1., 1), q, 1.02)
    recorder.sample(make_state(1.2, 2), q, 1.21)
    recorder.stop('OPERATOR_STOP')
    recorder.finish('TEST_END')
    path = wait_saved(tmp_path, 1)[0]
    meta = json.loads(path.read_text())
    assert meta['sample_count'] == 2
    assert meta['duplicate_timestamp_count'] == 1
    assert meta['sampling_gap_count'] == 1
    assert meta['max_sample_interval_s'] == pytest.approx(.2)
    assert meta['data_integrity'] == 'WARN'


def test_recorder_stops_on_nonmonotonic_or_invalid_sample(tmp_path):
    recorder = TrajectoryRecorder(tmp_path, max_samples=8)
    assert not recorder.start('LOCKED')
    assert recorder.start('DRAG')
    recorder.sample(make_state(2., 0), np.zeros(14), 2.01)
    recorder.sample(make_state(1.9, 0), np.zeros(14), 2.02)
    assert recorder.active is None
    recorder.finish('TEST_END')
    meta = json.loads(wait_saved(tmp_path, 1)[0].read_text())
    assert meta['termination_reason'] == 'NONMONOTONIC_TIME'
    assert meta['sample_count'] == 1
    assert meta['nonmonotonic_timestamp_count'] == 1


def test_recorder_invalid_sample_is_flagged_and_valid_prefix_saved(tmp_path):
    recorder = TrajectoryRecorder(tmp_path, max_samples=4)
    recorder.start('DRAG')
    recorder.sample(make_state(1., 0), np.zeros(14), 1.01)
    state = make_state(1.02, 0)
    state.dq = None
    recorder.sample(state, np.zeros(14), 1.03)
    recorder.finish('TEST_END')
    meta = json.loads(wait_saved(tmp_path, 1)[0].read_text())
    assert meta['termination_reason'] == 'INVALID_DATA'
    assert meta['sample_count'] == 1
    assert meta['invalid_sample_count'] == 1


def test_measured_and_reference_are_independent_copies_and_capacity_is_bounded(tmp_path):
    recorder = TrajectoryRecorder(tmp_path, max_samples=2)
    recorder.start('DRAG')
    state = make_state(1., 0)
    ref = np.full(14, -1.)
    recorder.sample(state, ref, 1.01)
    state.q[:] = 99.
    ref[:] = 88.
    recorder.sample(make_state(1.03, 1), np.full(14, -2.), 1.04)
    assert recorder.active is None
    assert not recorder.start('DRAG')
    recorder.finish('TEST_END')
    path = wait_saved(tmp_path, 1)[0]
    meta = json.loads(path.read_text())
    assert meta['termination_reason'] == 'BUFFER_FULL'
    assert meta['actual_frequency_hz'] == pytest.approx(1/.03)
    with np.load(path.with_suffix('.npz'), allow_pickle=False) as data:
        np.testing.assert_array_equal(data['q_measured'][0], np.full(14, .2))
        np.testing.assert_array_equal(data['dq_measured'][0], np.full(14, .3))
        np.testing.assert_array_equal(data['q_ref'][0], np.full(14, -1.))


@pytest.mark.parametrize('action', ['m', 'f', 'q'])
def test_commands_stop_recording_and_waypoints_are_preserved(tmp_path, action):
    inbox = queue.Queue()
    inbox.put('f')
    inbox.put('r')
    controller = RecordingController(inbox)
    original = controller._frame
    def frame(weight, phase):
        original(weight, phase)
        while not inbox.empty(): inbox.get_nowait()
        inbox.put(action if len(controller.frames) == 1 else 'q')
    controller._frame = frame
    recorder = TrajectoryRecorder(tmp_path, max_samples=10)
    points = run_teach(controller, inbox, recorder=recorder)
    assert controller.released
    meta = json.loads(wait_saved(tmp_path, 1)[0].read_text())
    assert meta['sample_count'] == 1
    assert meta['termination_reason'] == ('OPERATOR_QUIT' if action == 'q' else 'EXIT_DRAG')
    assert len(points) == (1 if action == 'm' else 0)


def test_safety_abort_releases_before_finalizing_and_preserves_data(tmp_path):
    recorder = TrajectoryRecorder(tmp_path, max_samples=10)
    inbox = queue.Queue()
    inbox.put('f')
    inbox.put('r')
    controller = RecordingController(inbox)
    def abort(weight, phase): raise SafetyAbort('offline injected')
    controller._frame = abort
    finish = recorder.finish
    def checked_finish(reason):
        assert controller.released
        finish(reason)
    recorder.finish = checked_finish
    with pytest.raises(SafetyAbort):
        run_teach(controller, inbox, recorder=recorder)
    meta = json.loads(wait_saved(tmp_path, 1)[0].read_text())
    assert meta['termination_reason'] == 'SAFETY_ABORT'
    assert meta['sample_count'] == 1


def test_save_failure_does_not_affect_release_or_control_frames(tmp_path):
    directory = tmp_path / 'not_a_directory'
    directory.write_text('offline failure fixture')
    recorder = TrajectoryRecorder(directory, max_samples=10)
    inbox = queue.Queue()
    inbox.put('f')
    controller = RecordingController(inbox)
    run_teach(controller, inbox, recorder=recorder)
    assert controller.released
    assert len(controller.frames) == 7
    assert all(r['status'] == 'SAVE_FAILED' for r in recorder.results)


def test_recording_does_not_change_commands_or_state_read_count(tmp_path):
    runs = []
    for enabled in (False, True):
        inbox = queue.Queue()
        inbox.put('f')
        controller = RecordingController(inbox)
        original = controller._frame
        def frame(weight, phase):
            original(weight, phase)
            while not inbox.empty(): inbox.get_nowait()
            if len(controller.frames) == 1 and enabled: inbox.put('r')
            if len(controller.frames) == 5: inbox.put('q')
        controller._frame = frame
        run_teach(controller, inbox, recorder=TrajectoryRecorder(tmp_path / str(enabled), max_samples=10))
        runs.append(controller)
    assert runs[0].reads == runs[1].reads
    assert len(runs[0].frames) == len(runs[1].frames)
    for a, b in zip(runs[0].frames, runs[1].frames):
        assert a[:2] == b[:2]
        np.testing.assert_array_equal(a[2], b[2])
        np.testing.assert_array_equal(a[3], b[3])


def test_segment_connections_use_measured_end_to_start_and_stable_ids(tmp_path):
    recorder = TrajectoryRecorder(tmp_path, max_samples=8)
    recorder.start('DRAG')
    first_id = recorder.active['trajectory_id']
    recorder.sample(make_state(1., 0), np.zeros(14), 1.01)
    recorder.sample(make_state(1.02, 1), np.zeros(14), 1.03)
    first_end = recorder.buffer[recorder.used-1, 2:16].copy()
    recorder.stop('OPERATOR_STOP')
    recorder.start('DRAG')
    second_id = recorder.active['trajectory_id']
    recorder.sample(make_state(1.04, 2), np.ones(14), 1.05)
    second_start = recorder.buffer[recorder.active['start'], 2:16].copy()
    recorder.sample(make_state(1.06, 3), np.ones(14), 1.07)
    recorder.stop('OPERATOR_STOP')
    recorder.finish('TEST_END')
    files = wait_saved(tmp_path, 2)
    metas = [json.loads(path.read_text()) for path in files]
    first = next(meta for meta in metas if meta['trajectory_id'] == first_id)
    second = next(meta for meta in metas if meta['trajectory_id'] == second_id)
    assert first['segment_index'] == 1 and first['previous_segment_id'] is None
    assert second['segment_index'] == 2 and second['previous_segment_id'] == first_id
    np.testing.assert_array_equal(first['end_q'], first_end)
    np.testing.assert_array_equal(second['start_q'], second_start)
    np.testing.assert_array_equal(second['connection_delta_q'], second_start-first_end)
    assert second['connection_max_abs_delta_q'] == pytest.approx(np.max(np.abs(second_start-first_end)))
    assert second['connection_max_abs_delta_q'] > 0
    assert first['session_id'] == second['session_id'] == recorder.session_id


def test_timestamped_paths_are_zoned_windows_safe_deduplicated_and_legacy_readable(tmp_path):
    instant = datetime(2026, 10, 12, 12, 34, 56, 123456, tzinfo=timezone.utc)
    first = timestamped_path(tmp_path, 'dual_arm_teach', '.jsonl', instant)
    first.touch()
    second = timestamped_path(tmp_path, 'dual_arm_teach', '.jsonl', instant)
    assert first != second
    assert ':' not in first.name
    assert read_log_timestamp(first) == instant
    assert read_log_timestamp(second) == instant
    legacy = tmp_path / 'dual_arm_teach_1791638375433505367.jsonl'
    assert read_log_timestamp(legacy) is not None
    assert read_log_timestamp(tmp_path / 'unknown.jsonl') is None


def test_display_name_is_mutable_without_renaming_or_changing_npz(tmp_path):
    recorder = TrajectoryRecorder(tmp_path, max_samples=4)
    recorder.start('DRAG')
    recorder.sample(make_state(1., 0), np.zeros(14), 1.01)
    recorder.stop('OPERATOR_STOP')
    recorder.finish('TEST_END')
    meta_path = wait_saved(tmp_path, 1)[0]
    npz_path = meta_path.with_suffix('.npz')
    trajectory_id = json.loads(meta_path.read_text())['trajectory_id']
    old_npz = npz_path.read_bytes()
    meta = json.loads(meta_path.read_text())
    meta['display_name'] = 'Windows renamed trajectory'
    meta_path.write_text(json.dumps(meta))
    reread = json.loads(meta_path.read_text())
    assert reread['trajectory_id'] == trajectory_id
    assert reread['display_name'] == 'Windows renamed trajectory'
    assert npz_path.exists() and npz_path.read_bytes() == old_npz
    loaded, arrays = load_trajectory(meta_path)
    assert loaded['display_name'] == 'Windows renamed trajectory'
    assert loaded['trajectory_id'] == trajectory_id


def test_legacy_schema_reader_keeps_files_and_normalizes_missing_metadata(tmp_path):
    path = tmp_path / 'trajectory_1791638375433505367_abcd_001.json'
    path.write_text(json.dumps(dict(schema_version=1, session='old_session', index=1)))
    np.savez(path.with_suffix('.npz'), q_measured=np.ones((2, 14)))
    original = path.read_bytes()
    meta, arrays = load_trajectory(path)
    assert meta['session_id'] == 'old_session' and meta['segment_index'] == 1
    assert meta['previous_segment_id'] is None and meta['recording_started_at'] is None
    assert meta['created_at'].startswith('2026-10-10')
    assert meta['start_q'] == meta['end_q'] == [1.] * 14
    assert load_trajectory(path)[0]['trajectory_id'] == meta['trajectory_id']
    assert path.read_bytes() == original


def test_empty_segment_has_no_invented_connection_or_endpoints(tmp_path):
    recorder = TrajectoryRecorder(tmp_path, max_samples=8)
    for n in range(3):
        recorder.start('DRAG')
        if n != 1:
            recorder.sample(make_state(1.+n, n), np.zeros(14), 1.01+n)
        recorder.stop('OPERATOR_STOP')
    recorder.finish('TEST_END')
    metas = sorted((json.loads(p.read_text()) for p in wait_saved(tmp_path, 3)), key=lambda m:m['segment_index'])
    assert metas[1]['start_q'] is None and metas[1]['end_q'] is None
    assert metas[2]['previous_segment_id'] == metas[1]['trajectory_id']
    assert metas[2]['connection_delta_q'] is None


def test_pair_collision_preserves_complete_timestamp_and_existing_files(tmp_path):
    stem = tmp_path / 'trajectory_id_2026-10-12T10-11-12.123456+0800'
    npz, meta = TrajectoryRecorder._unique_pair(stem)
    meta.write_text('existing')
    next_npz, next_meta = TrajectoryRecorder._unique_pair(stem)
    assert next_npz != npz and next_meta != meta
    assert '.123456+0800_1.npz' in next_npz.name
    assert meta.read_text() == 'existing'


def test_segment_limit_rejects_new_recordings_without_losing_previous(tmp_path, capsys):
    recorder = TrajectoryRecorder(tmp_path, max_samples=8, max_segments=2)
    for n in range(2):
        assert recorder.start('DRAG')
        recorder.sample(make_state(1.+n, n), np.zeros(14), 1.01+n)
        recorder.stop('OPERATOR_STOP')
    assert not recorder.start('DRAG')
    recorder.finish('TEST_END')
    assert len(wait_saved(tmp_path, 2)) == 2
    assert 'segments=2/2' in capsys.readouterr().out
