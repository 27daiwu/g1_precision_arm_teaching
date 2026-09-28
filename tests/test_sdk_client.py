import time
import numpy as np
import pytest
from g1_dual_arm_teaching.sdk.arm_sdk_client import ArmSdkClient
from g1_dual_arm_teaching.sdk.transport import SimulationTransport
from g1_dual_arm_teaching.sdk.arm_sdk_types import JointState
from g1_dual_arm_teaching.control.arm_controller import ArmController
from g1_dual_arm_teaching.control.command_builder import build_arm_command
from g1_dual_arm_teaching.waypoint import JointWaypoint


class RecordingTransport(SimulationTransport):
    def __init__(self):
        super().__init__(np.full(14, .1), [.1, .2, .3])
        self.frames = []
        self.stale = False
        self.fail = False

    def state(self):
        s = super().state()
        if self.stale:
            s.timestamp = time.monotonic() - 10
        return s

    def write(self, command, waist, weight):
        if self.fail:
            raise RuntimeError('DDS failed')
        self.frames.append((command, waist.copy() if waist is not None else None, weight))
        super().write(command, waist, weight)


def test_acquire_hold_release_and_waist(config):
    transport = RecordingTransport()
    with ArmSdkClient(config, transport=transport) as client:
        state = client.acquire()
        client.hold()
        client.release()
    np.testing.assert_array_equal(transport.frames[0][0].q, state.q)
    assert transport.frames[0][2] == 0
    assert transport.frames[-1][2] == 0
    for cmd, waist, weight in transport.frames:
        np.testing.assert_array_equal(waist, [.1, .2, .3])
        np.testing.assert_array_equal(cmd.q, state.q)
        assert np.all(cmd.tau_ff == 0)
    assert transport.closed


def test_no_preset_acquire(config):
    with ArmSdkClient(config, transport=RecordingTransport()) as client:
        with pytest.raises(ValueError, match='measured'):
            client.acquire(np.zeros(14))


def test_single_publisher(config):
    with ArmSdkClient(config):
        with pytest.raises(BlockingIOError):
            ArmSdkClient(config).initialize()
    with ArmSdkClient(config):
        pass


@pytest.mark.parametrize('failure', ['stale', 'fail', 'mutated', 'step'])
def test_abort_closes_and_prevents_further_commands(config, failure):
    transport = RecordingTransport()
    with ArmSdkClient(config, transport=transport) as client:
        client.acquire()
        command = build_arm_command(client.acquire_q)
        if failure == 'mutated':
            command.tau_ff[0] = 1
        elif failure == 'step':
            command.q[0] += 1
        else:
            setattr(transport, failure, True)
        with pytest.raises((ValueError, RuntimeError, TimeoutError)):
            client.send_joint_command(command)
        assert transport.closed
        count = len(transport.frames)
        with pytest.raises(RuntimeError):
            client.send_joint_command(command)
        assert len(transport.frames) == count


def test_watchdog_acts_while_caller_is_blocked(config):
    transport = RecordingTransport()
    with ArmSdkClient(config, transport=transport) as client:
        client.acquire()
        assert client._stop.wait(timeout=1.0)
        # Closing completes under the same lock as the abort.
        with client._mutex:
            assert transport.closed
        assert transport.frames[-1][2] == 0


def test_waypoint_execution_records_exact_measured_start_and_endpoint(config):
    transport = RecordingTransport()
    with ArmSdkClient(config, transport=transport) as client:
        controller = ArmController(client)
        controller.acquire()
        begin = len(transport.frames)
        start = client.get_joint_state().q
        target = np.full(14, .11)
        controller.move(JointWaypoint('A', target, .08))
        np.testing.assert_array_equal(transport.frames[begin][0].q, start)
        np.testing.assert_array_equal(transport.frames[-1][0].q, target)
        controller.move(JointWaypoint('B', start, .08))
        np.testing.assert_array_equal(transport.frames[-1][0].q, start)


def test_waypoint_cannot_move_waist(config):
    with ArmSdkClient(config, transport=RecordingTransport()) as client:
        controller = ArmController(client)
        controller.acquire()
        with pytest.raises(ValueError, match='waist'):
            controller.move(JointWaypoint('bad', client.acquire_q, waist_q_reference=np.zeros(3)))
        assert client._closed


def test_real_control_requires_review_before_initializing_transport(config):
    transport = RecordingTransport()
    with pytest.raises(RuntimeError, match='reviewed'):
        ArmSdkClient(config, real=True, transport=transport).initialize()
    assert not hasattr(transport, 'config')
    config['robot']['hardware_reviewed'] = True
    with pytest.raises(RuntimeError, match='Arm Action'):
        ArmSdkClient(config, real=True, transport=transport).initialize()


def test_ctrl_c_disables(config):
    transport = RecordingTransport()
    with pytest.raises(KeyboardInterrupt):
        with ArmSdkClient(config, transport=transport) as client:
            client.acquire()
            raise KeyboardInterrupt()
    assert transport.closed and transport.frames[-1][2] == 0


def test_waist_omission_experiment(config):
    config['waist']['send_commands'] = False
    transport = RecordingTransport()
    with ArmSdkClient(config, transport=transport) as client:
        client.acquire()
        client.hold()
    assert all(frame[1] is None for frame in transport.frames)


def test_acquire_rejects_fast_moving_state_before_first_frame(config):
    class Moving(RecordingTransport):
        def state(self):
            state = super().state()
            state.dq[:] = 2.0
            return state
    transport = Moving()
    with ArmSdkClient(config, transport=transport) as client:
        with pytest.raises(ValueError, match='velocity'):
            client.acquire()
    assert transport.frames == []
