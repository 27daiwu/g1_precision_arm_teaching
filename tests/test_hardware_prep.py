import json
import time
from types import SimpleNamespace
import numpy as np
import pytest
from g1_dual_arm_teaching.sdk.arm_sdk_client import ArmSdkClient
from g1_dual_arm_teaching.sdk.transport import SimulationTransport, UnitreeTransport
from g1_dual_arm_teaching.control.command_builder import build_arm_command
from g1_dual_arm_teaching.control.arm_controller import ArmController
from g1_dual_arm_teaching.waypoint import JointWaypoint
from g1_dual_arm_teaching.utils.hardware import validate_variant, require_real_acquire, weight_target
from g1_dual_arm_teaching.evaluation.hardware_telemetry import HardwareTelemetry
from g1_dual_arm_teaching.cli import main


def reviewed(config):
    config['robot'].update(hardware_reviewed=True, model_confirmed=True, arm_joint_mapping_verified=True)
    return config


@pytest.mark.parametrize('kind,active,locked', [
    ('ACTIVE_3DOF', [12,13,14], []), ('YAW_ONLY', [12], [13,14]),
    ('LOCKED_VARIANT', [12], [13,14]), ('LOCKED_VARIANT', [], [12,13,14])])
@pytest.mark.parametrize('experiment', ['SEND_ACQUIRE_REFERENCE', 'ZERO_GAIN_NO_COMMAND'])
def test_variant_wire_slots(config, kind, active, locked, experiment):
    config['waist'].update(configuration=kind, active_joints=active, locked_joints=locked, experiment=experiment)
    validate_variant(config)
    transport = UnitreeTransport()
    frames = []
    transport.config = config
    transport.publisher = SimpleNamespace(Write=lambda msg: frames.append(msg) or True)
    transport._message = lambda: SimpleNamespace(motor_cmd=[SimpleNamespace(q=0.,dq=0.,kp=0.,kd=0.,tau=0.) for _ in range(35)])
    transport._crc = SimpleNamespace(Crc=lambda message: 0)
    transport.write(build_arm_command(np.zeros(14)), np.array([.1,.2,.3]), .1)
    for i in (12,13,14):
        m = frames[0].motor_cmd[i]
        if i in active and experiment == 'SEND_ACQUIRE_REFERENCE':
            assert m.kp == config['waist']['kp'][i-12]
            assert m.q == [.1,.2,.3][i-12]
        else:
            assert m.q == m.kp == m.kd == m.tau == 0


@pytest.mark.parametrize('value', [0, -1, 1.01, np.inf, np.nan, True])
def test_invalid_weight(value):
    with pytest.raises(ValueError):
        weight_target(value)


@pytest.mark.parametrize('value', [.1,.25,.5,1.])
def test_each_weight_level(config, value):
    with ArmSdkClient(config, current_pose_only=True, weight=value) as client:
        client.acquire()
        assert client._weight == value
        with pytest.raises(ValueError, match='weight'):
            client.send_joint_command(client._command(client.acquire_q), weight=value+.01)


def test_unverified_limits_block_real_motion_but_not_reviewed_hold(config):
    reviewed(config)
    require_real_acquire(config, True)
    with pytest.raises(RuntimeError, match='unverified'):
        require_real_acquire(config, False)
    config['robot']['ready_for_phase1_real_joint_motion'] = True
    with pytest.raises(RuntimeError, match='unverified'):
        require_real_acquire(config, False)


@pytest.mark.parametrize('field', ['model_confirmed', 'arm_joint_mapping_verified', 'hardware_reviewed'])
def test_invalid_real_config_never_initializes(config, field):
    reviewed(config)["robot"][field] = False
    transport = SimulationTransport()
    with pytest.raises(RuntimeError):
        ArmSdkClient(config, real=True, current_pose_only=True, exclusive_control_confirmed=True, transport=transport).initialize()
    assert not hasattr(transport, 'config')


def test_unknown_waist_blocks_real_acquire(config):
    reviewed(config)
    config['waist'].update(configuration='UNKNOWN', configuration_verified=False, active_joints=[], locked_joints=[])
    with pytest.raises(RuntimeError, match='waist'):
        ArmSdkClient(config, real=True, current_pose_only=True, exclusive_control_confirmed=True, transport=SimulationTransport()).initialize()


def test_bad_partition(config):
    config['waist'].update(configuration='YAW_ONLY', active_joints=[12,13], locked_joints=[14])
    with pytest.raises(ValueError):
        validate_variant(config)


def test_hold_forbids_reference_and_waypoint(config):
    with ArmSdkClient(config, current_pose_only=True) as client:
        with pytest.raises(ValueError, match='forbids'):
            client.acquire(np.zeros(14))
        client.acquire()
        with pytest.raises(RuntimeError, match='waypoint'):
            ArmController(client).move(JointWaypoint('same', client.acquire_q))


@pytest.mark.parametrize('change', ['q','dq'])
def test_boundary_forbids_target_or_velocity(config, change):
    with ArmSdkClient(config, current_pose_only=True) as client:
        client.acquire()
        command = client._command(client.acquire_q)
        getattr(command, change)[0] += .001
        with pytest.raises(ValueError, match='CURRENT_POSE_ONLY'):
            client.send_joint_command(command)


def test_fixed_ramp_and_release_telemetry(config, tmp_path):
    class Drift(SimulationTransport):
        def write(self, command, waist, weight):
            self.q = command.q.copy() + .0001
    transport = Drift(np.full(14,.15), [.1,.2,.3])
    path = tmp_path / 'cycles.jsonl'
    with ArmSdkClient(config, transport=transport, current_pose_only=True) as client:
        recorder = HardwareTelemetry(path, client)
        client.telemetry = recorder
        client.acquire()
        client.hold()
        client.release()
    recorder.close()
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    cycles = [r for r in rows if 'q_command' in r]
    for row in cycles:
        assert row['q_command'] == [.15]*14
        assert 0 <= row['arm_sdk_weight'] <= .1
        for key in ('dq_measured','error','waist_q_command','state_age','loop_dt'):
            assert key in row
    assert {r['phase'] for r in cycles} == {'acquire','hold','release'}
    report = json.loads(path.with_suffix('.report.json').read_text())
    assert report['MAX_COMMAND_STEP'] == 0
    assert report['WEIGHT_BEFORE_RELEASE'] == .1
    assert report['WEIGHT_AFTER_RELEASE'] == 0
    assert report['DQ_PEAK_AFTER_RELEASE'] == 0
    assert report['RELEASE_CAUSES_LARGE_TRANSIENT'] == 'UNKNOWN'
    assert report['READY_FOR_PHASE1_REAL_JOINT_MOTION'] == 'NO'
    assert report['LOWSTATE_REAL_RECEIVED'] == 'NO'


def test_stale_samples_cannot_pass_stability(config):
    class Frozen(SimulationTransport):
        def state(self):
            if not hasattr(self, 'frozen'):
                self.frozen = super().state()
            return self.frozen
    config['control']['initial_state_timeout_s'] = .05
    with ArmSdkClient(config, transport=Frozen(), current_pose_only=True) as client:
        with pytest.raises(TimeoutError, match='stable'):
            client.acquire()
        assert not client._acquired


def test_audit_no_publisher_and_records(config, tmp_path, monkeypatch):
    from g1_dual_arm_teaching import cli
    class ReadOnlyTransport(SimulationTransport):
        def initialize(self, config, interface, publisher):
            assert publisher is False
            super().initialize(config, interface, publisher)
    original = cli.ArmSdkClient
    monkeypatch.setattr(cli, 'ArmSdkClient', lambda *a, **kw: original(*a, transport=ReadOnlyTransport(), **kw))
    path = tmp_path / 'audit.jsonl'
    assert main('audit', ['--duration','0','--log',str(path)]) == 0
    row = json.loads(path.read_text())
    assert row['WAIST_CONFIGURATION_VERIFIED'] == 'NO'
    assert row['LOWSTATE_FREQUENCY'] is None
    assert row['MOTOR_15_28_DQ'] == [0]*14


def test_cli_rejects_unqualified_real_hold_and_targets():
    with pytest.raises(SystemExit):
        main('acquire_hold', ['--real'])
    with pytest.raises(SystemExit):
        main('acquire_hold', ['--acquire-hold-only','--waypoint-a','anything'])


def test_formal_limits_require_every_controlled_joint_and_phase1_review(config):
    reviewed(config)
    config['metadata'].update(source='test fixture only', hardware_verified=True)
    for entry in config['joints'].values():
        entry.update(min=-1., max=1., verified=True)
    with pytest.raises(RuntimeError, match='Phase 1'):
        require_real_acquire(config, False)
    config['robot']['ready_for_phase1_real_joint_motion'] = True
    require_real_acquire(config, False)
    config['joints']['waist_pitch']['verified'] = False
    with pytest.raises(RuntimeError, match='motor 14'):
        require_real_acquire(config, False)
    config['waist'].update(configuration='YAW_ONLY', active_joints=[12], locked_joints=[13,14])
    require_real_acquire(config, False)
    config['joints']['left_elbow']['verified'] = False
    with pytest.raises(RuntimeError, match='motor 18'):
        require_real_acquire(config, False)


def test_audit_frequency_counts_callbacks_not_state_reads():
    t = UnitreeTransport()
    message = SimpleNamespace(mode_machine=4, motor_state=[SimpleNamespace(q=0.,dq=0.) for _ in range(35)])
    t._receive(message)
    assert t.diagnostics()['LOWSTATE_FREQUENCY'] is None
    t._receive(message)
    count = t.diagnostics()['LOWSTATE_SAMPLE_COUNT']
    for _ in range(10):
        t.state()
    assert t.diagnostics()['LOWSTATE_SAMPLE_COUNT'] == count == 2
    assert t.diagnostics()['LOWSTATE_FREQUENCY'] > 0
    assert t.diagnostics()['MODE_MACHINE'] == 4
