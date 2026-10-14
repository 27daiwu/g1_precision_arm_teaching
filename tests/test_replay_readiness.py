"""Replay readiness state-transition/fault checks using fake transport only."""

from types import SimpleNamespace

import numpy as np
import pytest
import json

from g1_dual_arm_teaching.sdk.replay_controller import ReplayController
from g1_dual_arm_teaching.sdk.replay_fake_adapter import FakeReplayTransport
from g1_dual_arm_teaching.trajectory.replay import ReplayGains, ReplayLimits
from g1_dual_arm_teaching.trajectory.replay_c2 import C2ReplayPlan, RelativeReplaySpec
from g1_dual_arm_teaching.utils.config import load_config


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class FakePermission:
    def __init__(self, limits, gains):
        self.limits = limits
        self.gains = gains
        self.period = 0.02
        self.gain_ramp = 0.04
        self.hold = 0.04
        self.stop = 0.5
        self.fault_actions = {key: "freeze_then_bounded_release" for key in
                              ("preflight", "feedback", "tracking", "motion", "timing", "write", "operator")}

    def check_plan(self, plan):
        assert isinstance(plan, C2ReplayPlan)

    def make_start_transition(self, start_q, plan):
        return None


def setup_controller():
    clock = FakeClock()
    limits = ReplayLimits(
        (-2.0,) * 14, (2.0,) * 14,
        velocity=2.0, acceleration=100.0, step=0.1, gap=0.6,
        start_error=0.01, tracking_error=0.1, state_timeout=0.1,
        cycle_timeout=0.1, max_duration=5.0, start_velocity=0.02,
    )
    gains = ReplayGains((12.0,) * 14, (1.5,) * 14, (5.0,) * 14,
                        (40.0,) * 14, (0.5,) * 14, (3.0,) * 14)
    permission = FakePermission(limits, gains)
    transport = FakeReplayTransport(clock)
    config = load_config("configs")
    controller = ReplayController(transport, config, permission,
                                  clock=clock, sleep=clock.sleep)
    time = np.array([0.0, 0.5, 1.0])
    q = np.zeros((3, 14))
    q[:, 0] = (0.0, 0.001, 0.002)
    metadata = dict(sample_count=3, duration_s=1.0, data_integrity="PASS")
    arrays = dict(timestamp=time, q_measured=q, cycle_timestamp=time + 0.001,
                  dq_measured=np.zeros_like(q), q_ref=np.zeros_like(q),
                  waist_q=np.zeros((3, 3)), torso_roll_pitch=np.zeros((3, 2)),
                  arm_motor_ids=np.arange(15, 29), waist_motor_ids=np.arange(12, 15))
    plan = C2ReplayPlan(metadata, arrays, limits, 1.0,
                        jerk_limit=100.0, jerk_jump_limit=100.0)
    controller.acquire(plan)
    return clock, transport, controller


def test_fake_hold_end_holds_last_reference_then_releases():
    clock, transport, controller = setup_controller()
    for _ in range(100):
        if not controller.replay_active:
            break
        clock.sleep(controller.period_s)
        controller.tick()
    assert "HOLD_END" in controller.transitions
    assert controller.transitions[-1] == "RELEASE"
    assert controller.released
    assert transport.frames[-1].motor_cmd[29].q == 0.0
    np.testing.assert_allclose([m.q for m in transport.frames[-1].motor_cmd[15:29]],
                               controller.plan.q[-1], atol=1e-12)


def test_fake_operator_controlled_stop_completes_and_releases():
    clock, transport, controller = setup_controller()
    controller.request_stop(0.5)
    for _ in range(30):
        if not controller.replay_active:
            break
        clock.sleep(controller.period_s)
        controller.tick()
    assert "CONTROLLED_STOP" in controller.transitions
    assert controller.transitions[-1] == "RELEASE"
    assert controller.released
    assert transport.frames[-1].motor_cmd[29].q == 0.0


def test_fake_operator_hard_abort_releases_without_another_motion_frame():
    _, transport, controller = setup_controller()
    writes_before_abort = len(transport.frames)
    controller.stop()
    with pytest.raises(InterruptedError):
        controller.tick()
    assert controller.transitions[-1] == "RELEASE"
    assert controller.released
    assert len(transport.frames) > writes_before_abort
    assert transport.frames[-1].motor_cmd[29].q == 0.0


def test_fake_write_failure_is_reported_and_release_is_best_effort():
    _, transport, controller = setup_controller()
    transport.fail = True
    with pytest.raises(RuntimeError, match="injected fake write failure"):
        controller.tick()
    assert controller.transitions[-1] == "RELEASE"
    assert not controller.released
    assert controller.release_failures > 0


def test_fake_relative_replay_binds_acquire_snapshot_and_completes_hold_release(tmp_path):
    clock=FakeClock()
    limits=ReplayLimits((-2.0,)*14,(2.0,)*14,velocity=2.0,acceleration=100.0,
        step=.1,gap=.6,start_error=.01,tracking_error=.1,state_timeout=.1,
        cycle_timeout=.1,max_duration=5.,start_velocity=.02)
    gains=ReplayGains((12.,)*14,(1.5,)*14,(5.,)*14,(40.,)*14,(.5,)*14,(3.,)*14)
    permission=FakePermission(limits,gains)
    acquire_q=np.linspace(-.1,.1,14)
    transport=FakeReplayTransport(clock,acquire_q)
    config=load_config("configs")
    controller=ReplayController(transport,config,permission,clock=clock,sleep=clock.sleep)
    timestamp=np.array([0.,.5,1.])
    q=np.array([np.zeros(14),np.full(14,.001),np.full(14,.002)])
    arrays=dict(timestamp=timestamp,cycle_timestamp=timestamp+.001,q_measured=q,
        dq_measured=np.zeros_like(q),q_ref=q.copy(),waist_q=np.zeros((3,3)),
        torso_roll_pitch=np.zeros((3,2)),arm_motor_ids=np.arange(15,29),
        waist_motor_ids=np.arange(12,15))
    path=tmp_path/'relative.npz'
    np.savez(path,**arrays)
    path.with_suffix('.json').write_text(json.dumps(dict(sample_count=3,duration_s=1.,
        data_integrity='PASS',start_q=q[0].tolist(),end_q=q[-1].tolist())))
    spec=RelativeReplaySpec.load(path,limits,1.,jerk_limit=1e6,
        jerk_jump_limit=1e6,motion_scale=.5)

    controller.acquire(spec,hold_end_s=.04)
    np.testing.assert_array_equal(controller.plan.acquire_q,acquire_q)
    np.testing.assert_array_equal(controller.plan.sample(0.)[0],acquire_q)
    clock.sleep(controller.period_s)
    controller.tick()
    expected=controller.plan.sample(clock()-controller.started)[0]
    np.testing.assert_allclose([m.q for m in transport.frames[-1].motor_cmd[15:29]],expected)
    for _ in range(100):
        if not controller.replay_active:
            break
        clock.sleep(controller.period_s)
        controller.tick()
    assert controller.transitions[-1]=='RELEASE'
    assert 'HOLD_END' in controller.transitions
    assert controller.released
    np.testing.assert_allclose([m.q for m in transport.frames[-1].motor_cmd[15:29]],
                               controller.plan.q[-1],atol=1e-12)
