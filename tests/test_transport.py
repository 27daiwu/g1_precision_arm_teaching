"""Exercise the actual adapter with a fake DDS API, without importing SDK2."""
import sys
from types import ModuleType, SimpleNamespace
import numpy as np
from g1_dual_arm_teaching.sdk.transport import UnitreeTransport
from g1_dual_arm_teaching.control.command_builder import build_arm_command


def test_official_wire_mapping_and_readonly(monkeypatch, config):
    created, frames, factories = [], [], []
    class Publisher:
        def __init__(self, topic, kind):
            created.append(topic)
        def Init(self):
            pass
        def Write(self, msg):
            frames.append(msg)
            return True
        def Close(self):
            pass
    class Subscriber:
        def __init__(self, topic, kind):
            assert topic == 'rt/lowstate'
        def Init(self, callback, length):
            callback(SimpleNamespace(mode_machine=7, motor_state=[SimpleNamespace(q=i / 100, dq=0.) for i in range(35)]))
        def Close(self):
            pass
    def message():
        return SimpleNamespace(motor_cmd=[SimpleNamespace(q=0., dq=0., kp=0., kd=0., tau=0.) for _ in range(35)])
    def module(name, **attrs):
        result = ModuleType(name)
        result.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, result)
    module('unitree_sdk2py.core.channel', ChannelPublisher=Publisher, ChannelSubscriber=Subscriber,
           ChannelFactoryInitialize=lambda domain, interface: factories.append((domain, interface)))
    module('unitree_sdk2py.idl.default', unitree_hg_msg_dds__LowCmd_=message)
    module('unitree_sdk2py.idl.unitree_hg.msg.dds_', LowCmd_=object, LowState_=object)
    module('unitree_sdk2py.utils.crc', CRC=lambda: SimpleNamespace(Crc=lambda msg: 123))
    transport = UnitreeTransport()
    transport.initialize(config, 'test0', True)
    state = transport.state()
    np.testing.assert_allclose(state.q, np.arange(15, 29) / 100)
    np.testing.assert_allclose(state.waist_q, [.12, .13, .14])
    command = build_arm_command(np.arange(14) / 10, kp=np.ones(14), kd=np.ones(14))
    transport.write(command, np.array([.12, .13, .14]), .5)
    assert created == ['rt/arm_sdk'] and factories == [(0, 'test0')]
    frame = frames[-1]
    assert frame.crc == 123 and frame.mode_machine == 7 and frame.mode_pr == 0
    np.testing.assert_allclose([m.q for m in frame.motor_cmd[15:29]], command.q)
    np.testing.assert_allclose([m.q for m in frame.motor_cmd[12:15]], [.12, .13, .14])
    assert frame.motor_cmd[29].q == .5
    assert all(m.tau == 0 for m in frame.motor_cmd)
    assert all(m.kp == 0 for m in frame.motor_cmd[:12])
    transport.write(command, None, 0.)
    assert all(m.kp == m.kd == m.q == 0 for m in frames[-1].motor_cmd[12:15])
    transport.close()
    readonly = UnitreeTransport()
    readonly.initialize(config, 'test0', False)
    assert len(created) == 1
    readonly.close()
