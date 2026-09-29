import queue
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from g1_dual_arm_teaching.dual_arm_teach import ArmTeachMode, DRAG_KP, WAIST_KP, WAIST_PITCH_HOLD_BIAS, run_teach
from g1_dual_arm_teaching.sdk.arm_sdk_types import JointState
from g1_dual_arm_teaching.sdk.golden_diagnostics import GoldenDiagnostics
from g1_dual_arm_teaching.sdk.golden_controller import ArmSdkGoldenController
from g1_dual_arm_teaching.sdk.runtime_guard import RuntimeWireGuard
from g1_dual_arm_teaching.utils.config import load_config


def test_offline_config_load_keeps_acquire_guard_available():
    config = load_config(Path(__file__).resolve().parents[1] / 'configs')
    assert config['arm_sdk']['ownership_weight'] == 1.


def test_only_explicit_commands_change_global_mode():
    mode = ArmTeachMode(np.zeros(14))
    assert mode.mode == 'LOCKED'
    np.testing.assert_array_equal(mode.gains(0), np.full(14, 40.))
    for q in (np.ones(14), -np.ones(14)):
        np.testing.assert_array_equal(mode.update(q), np.zeros(14))
        assert mode.mode == 'LOCKED'

    mode.drag(np.ones(14))
    assert mode.mode == 'DRAG'
    np.testing.assert_array_equal(mode.gains(0), DRAG_KP)
    np.testing.assert_array_equal(mode.update(np.full(14, 2.)), np.full(14, 2.))
    np.testing.assert_array_equal(mode.update(np.full(14, 3.)), np.full(14, 3.))
    assert mode.mode == 'DRAG'

    captured = mode.lock(np.full(14, 4.), 1.)
    np.testing.assert_array_equal(captured, mode.reference)
    assert mode.mode == 'LOCKED'
    np.testing.assert_array_equal(mode.gains(1.), DRAG_KP)
    np.testing.assert_array_equal(mode.gains(1.4), np.full(14, 40.))
    np.testing.assert_array_equal(mode.update(np.full(14, 9.)), captured)
    assert mode.mode == 'LOCKED'


def test_invalid_capture_does_not_change_mode():
    mode = ArmTeachMode(np.zeros(14))
    with pytest.raises(ValueError):
        mode.drag(np.full(14, np.nan))
    assert mode.mode == 'LOCKED'
    with pytest.raises(ValueError):
        mode.lock(np.zeros(13), 0.)
    assert mode.mode == 'LOCKED'


class FakeController:
    period_s = .02
    waist_kp_by_axis = WAIST_KP
    waist_pitch_hold_bias = WAIST_PITCH_HOLD_BIAS
    static_hold_waist_kp = None

    def __init__(self, inbox):
        self.inbox = inbox
        self.time = 1.
        self.q_hold = np.zeros(17)
        self.max_delta = np.zeros(17)
        self.diagnostics = SimpleNamespace(writes=[], first_trigger=None)
        self.events = []
        self.frames = []
        self.reads = 0
        self.reads_at_mark = None
        self.released = False

    def clock(self):
        return self.time

    def _profiled_sleep(self, duration):
        self.time += duration

    def capture_upper_body_pose(self):
        pass

    def acquire_current_pose(self):
        pass

    def hold_current_pose(self, duration):
        pass

    def _state(self):
        self.reads += 1
        return SimpleNamespace(q=np.full(14, .1 * self.reads), dq=np.full(14, 9.))

    def set_teach_command(self, reference, kp):
        self.command = (reference.copy(), kp.copy())

    def _frame(self, weight, phase):
        self.frames.append((weight, phase, *self.command))
        self.diagnostics.writes.append(dict(phase=phase, weight=weight, outcome='SUCCESS'))
        if len(self.frames) == 1:
            self.inbox.put('m')
        elif len(self.frames) == 2:
            self.inbox.put('q')

    def emit(self, event):
        self.events.append(event)
        if event.get('event') == 'JOINT_WAYPOINT_MARK':
            self.reads_at_mark = self.reads

    def release(self):
        self.released = True
        self.diagnostics.writes.append(dict(phase='RELEASE', weight=0., outcome='SUCCESS'))


def test_f_then_m_uses_single_capture_and_keeps_ownership():
    inbox = queue.Queue()
    inbox.put('f')
    controller = FakeController(inbox)
    points = run_teach(controller, inbox)
    assert len(points) == 1
    assert controller.reads_at_mark == 3  # entry, F frame, M frame
    captured = np.full(14, .3)
    np.testing.assert_allclose(points[0]['q_measured_at_mark'], captured)
    np.testing.assert_allclose(points[0]['q_command_reference'], captured)
    assert controller.frames[0][0] == controller.frames[1][0] == 1.
    np.testing.assert_array_equal(controller.frames[0][3], DRAG_KP)
    np.testing.assert_allclose(controller.frames[1][2], captured)
    assert controller.released
    assert controller.diagnostics.writes[-1]['weight'] == 0.
    assert [e['mode'] for e in controller.events if e.get('event') == 'ARM_TEACH_MODE'] == ['DRAG', 'LOCKED']


def test_teach_arm_motion_is_not_acquire_delta_warning():
    config = dict(phase0_hold=dict(waist_watchdog=dict(warning_abs_delta_rad=.05,
        max_abs_delta_rad=.08, max_abs_velocity_rad_s=.5), pre_acquire_motion_threshold=.02),
        safety=dict(state_timeout_s=.2, max_joint_step_rad=.05),
        limits=dict(max_velocity_rad_s=1.))
    diagnostics = GoldenDiagnostics(config, lambda: 1., lambda event: None)
    state = JointState(np.full(14, .2), 1., np.full(14, 2.), np.zeros(3), np.zeros(3))
    teach, *_ = diagnostics.inspect(state, np.zeros(17), 'TEACH', 1., False)
    acquire, *_ = diagnostics.inspect(state, np.zeros(17), 'ACQUIRE', .5, False)
    assert not any(v['abort_code'] == 'ARM_DIAGNOSTIC_WARNING' for v in teach)
    assert any(v['reason'] == 'ARM_DELTA' for v in acquire)
    unsafe_waist = JointState(np.full(14, .2), 1., np.full(14, 2.),
                              np.array([.1, 0., 0.]), np.zeros(3))
    violations, *_ = diagnostics.inspect(unsafe_waist, np.zeros(17), 'TEACH', 1., False)
    assert any(v['abort_code'] == 'WAIST_SAFETY_ABORT' for v in violations)


def test_pitch_bias_ramps_with_acquire_and_persists_through_release():
    config = load_config(Path(__file__).resolve().parents[1] / 'configs')
    motor = lambda: SimpleNamespace(q=0., dq=0., kp=0., kd=0., tau=0., mode=0, reserve=0)
    message = lambda: SimpleNamespace(motor_cmd=[motor() for _ in range(30)],
                                      mode_pr=0, mode_machine=0, reserve=[], crc=0)
    transport = SimpleNamespace(_message=message, _crc=SimpleNamespace(Crc=lambda cmd: 0),
                                publisher=None, state=lambda: JointState(np.zeros(14), 1.,
                                    np.zeros(14), np.array([.11, -.12, .13]), np.zeros(3)))
    controller = ArmSdkGoldenController(transport, config, clock=lambda: 1.,
        waist_kp_by_axis=WAIST_KP, teach_waist=True,
        waist_pitch_hold_bias=WAIST_PITCH_HOLD_BIAS)
    controller.q_hold = np.r_[np.array([.11, -.12, .13]), np.zeros(14)]
    controller.runtime_guard = RuntimeWireGuard(controller.defaults, controller.q_hold,
        controller.motor14_kp, teach_waist=True, waist_kp_by_axis=controller.waist_kp_by_axis,
        waist_pitch_hold_bias=WAIST_PITCH_HOLD_BIAS)
    controller.sequence = 1
    controller.weight = 1.
    controller.set_teach_command(np.zeros(14), np.full(14, 40.))

    def waist_frame(weight, phase):
        controller._frame(weight, phase, publish=False)
        return [tuple(getattr(controller.cmd.motor_cmd[i], field)
                      for field in ('q', 'kp', 'kd', 'tau')) for i in range(12, 15)]

    acquire_start = waist_frame(0., 'ACQUIRE')
    acquire_middle = waist_frame(.5, 'ACQUIRE')
    interrupted_release = waist_frame(.25, 'RELEASE')
    acquire_complete = waist_frame(1., 'ACQUIRE_COMPLETE')
    hold = waist_frame(1., 'HOLD')
    locked_before = waist_frame(1., 'TEACH')
    controller.set_teach_command(np.ones(14), DRAG_KP)
    drag = waist_frame(1., 'TEACH')
    assert [controller.cmd.motor_cmd[i].kp for i in range(15, 29)] == DRAG_KP.tolist()
    assert all(controller.cmd.motor_cmd[i].kd == 1.5 and controller.cmd.motor_cmd[i].tau == 0.
               for i in range(12, 29))
    release = waist_frame(.5, 'RELEASE')
    final_zero = waist_frame(0., 'RELEASE')
    controller.set_teach_command(np.ones(14), np.full(14, 40.))
    locked_after = waist_frame(1., 'TEACH')
    assert all(controller.cmd.motor_cmd[i].kp == 40. for i in range(15, 29))

    expected = [(.11, 80., 1.5, 0.), (-.12, 80., 1.5, 0.), (.14, 120., 1.5, 0.)]
    assert acquire_start[2][0] == pytest.approx(.13)
    assert acquire_middle[2][0] == pytest.approx(.135)
    assert interrupted_release[2][0] == pytest.approx(.135)
    for frame in (acquire_complete, hold, locked_before, drag, locked_after, release, final_zero):
        for actual, target in zip(frame, expected):
            assert actual == pytest.approx(target)
    for frame in (acquire_start, acquire_middle):
        assert frame[:2] == expected[:2]
    assert controller.q_hold[:3].tolist() == [.11, -.12, .13]
    assert config['phase0_hold']['waist_watchdog'] == dict(
        warning_abs_delta_rad=.05, max_abs_delta_rad=.08, max_abs_velocity_rad_s=.5)
    assert controller.post_run_audit()['result'] == 'FIELDS_PASS_CDR_UNAVAILABLE'
    assert controller.cmd.motor_cmd[29].q == 1.
