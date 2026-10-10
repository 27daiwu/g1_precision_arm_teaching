import queue
from types import SimpleNamespace

import numpy as np
import pytest

from g1_dual_arm_teaching.control.body_attitude_shadow import BodyAttitudeShadow
from g1_dual_arm_teaching.dual_arm_teach import run_teach
from test_explicit_teach_mode import FakeController


def state(timestamp, roll=0., pitch=0., waist=None):
    q = np.array([np.cos(roll/2), np.sin(roll/2), 0., 0.])
    return SimpleNamespace(timestamp=timestamp, imu_quaternion_wxyz=q,
                           waist_q=np.zeros(3) if waist is None else np.asarray(waist))


def test_reference_p_clamp_and_rate_limit():
    shadow = BodyAttitudeShadow(kp_roll=1., kp_pitch=1., clamp_rad=.05,
                                rate_limit_rad_s=.1)
    first = shadow.update(state(1.))
    assert first.roll_ref == first.pitch_ref == first.motor13_candidate == 0.
    second = shadow.update(state(1.2, roll=-.4, waist=[0., 0., -.2]))
    assert second.roll_error == pytest.approx(-second.torso_roll)
    assert second.roll_error > .4
    assert second.motor13_candidate == pytest.approx(.02)
    assert second.motor14_candidate == pytest.approx(.02)
    assert (second.q12, second.q13, second.q14) == (0., 0., -.2)
    third = shadow.update(state(1.5, roll=-.4, waist=[0., 0., -.2]))
    assert third.motor13_candidate == pytest.approx(.05)
    assert third.motor14_candidate == pytest.approx(.05)


def test_invalid_frame_does_not_advance_candidate():
    shadow = BodyAttitudeShadow()
    shadow.update(state(1.))
    with pytest.raises(ValueError):
        shadow.update(state(.9))
    assert shadow.previous_time == 1.
    with pytest.raises(ValueError):
        shadow.update(SimpleNamespace(timestamp=1.1, waist_q=np.zeros(3),
                                      imu_quaternion_wxyz=np.array([0., 0., 0., 0.])))
    assert shadow.previous_time == 1.


class ShadowFakeController(FakeController):
    correction = (0., 0.)

    def set_body_attitude_correction(self, roll, pitch):
        self.correction = (roll, pitch)

    def _frame(self, weight, phase):
        super()._frame(weight, phase)
        self.events.append(dict(event='TEST_WAIST_COMMAND',
                                motor13_ref=self.q_hold[1] + self.correction[0],
                                motor14_ref=self.q_hold[2] + self.waist_pitch_hold_bias + self.correction[1]))

    def _state(self):
        measured = super()._state()
        measured.timestamp = self.time
        measured.imu_quaternion_wxyz = np.array([1., 0., 0., 0.])
        measured.waist_q = np.array([0., -.01 * self.reads, .02 * self.reads])
        return measured


def test_teach_shadow_logs_without_changing_commands():
    inbox = queue.Queue()
    inbox.put('f')
    baseline = FakeController(queue.Queue())
    baseline.inbox.put('f')
    run_teach(baseline, baseline.inbox)
    controller = ShadowFakeController(inbox)
    run_teach(controller, inbox, body_attitude_shadow=True)
    assert len(controller.frames) == len(baseline.frames)
    for shadow_frame, baseline_frame in zip(controller.frames, baseline.frames):
        assert shadow_frame[:2] == baseline_frame[:2]
        np.testing.assert_array_equal(shadow_frame[2], baseline_frame[2])
        np.testing.assert_array_equal(shadow_frame[3], baseline_frame[3])
    events = [event for event in controller.events if event['event'].startswith('BODY_ATTITUDE_SHADOW')]
    assert events[0]['event'] == 'BODY_ATTITUDE_SHADOW_REFERENCE'
    assert any(event['event'] == 'BODY_ATTITUDE_SHADOW_V1' for event in events)
    for event in events:
        assert {'torso_roll', 'torso_pitch', 'roll_ref', 'pitch_ref', 'roll_error',
                'pitch_error', 'motor13_candidate', 'motor14_candidate',
                'q12', 'q13', 'q14'} <= event.keys()


def test_closed_loop_uses_acquire_references_and_preserves_arm_frames():
    inbox = queue.Queue()
    inbox.put('f')
    controller = ShadowFakeController(inbox)
    run_teach(controller, inbox, body_attitude_closed_loop=True)
    logs = [event for event in controller.events if event['event'] == 'BODY_ATTITUDE_SHADOW_V1']
    assert logs and all(event['closed_loop'] for event in logs)
    assert all(event['motor13_command_ref'] == pytest.approx(controller.q_hold[1] + event['motor13_candidate'])
               and event['motor14_command_ref'] == pytest.approx(controller.q_hold[2] + .010 + event['motor14_candidate'])
               for event in logs)
    assert all(abs(event['motor13_candidate']) <= .05 and abs(event['motor14_candidate']) <= .05
               for event in logs)
    assert len(controller.frames) == 3
