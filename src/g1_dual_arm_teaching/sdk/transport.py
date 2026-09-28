"""DDS details stay in sdk/. Import Unitree only for explicit hardware use."""
import time
import threading
import numpy as np
from .arm_sdk_types import JointState


class SimulationTransport:
    """Ideal position follower, not a dynamics or accuracy validation model."""
    def __init__(self, q=None, waist_q=None):
        self.q = np.zeros(14) if q is None else np.array(q, dtype=float)
        self.waist = np.zeros(3) if waist_q is None else np.array(waist_q, dtype=float)
        self.closed = False

    def initialize(self, config, interface, publisher):
        self.config = config

    def state(self):
        return JointState(self.q, time.monotonic(), np.zeros(14), self.waist)

    def write(self, command, waist, weight):
        if self.closed:
            raise RuntimeError('transport closed')
        if weight > 0:
            self.q = command.q.copy()
            if waist is not None:
                from ..utils.hardware import active_waist
                for motor in active_waist(self.config):
                    self.waist[motor - 12] = waist[motor - 12]

    def close(self):
        self.closed = True


class UnitreeTransport:
    """G1 arm7 SDK wire format, never publishes rt/lowcmd."""
    def __init__(self):
        self.publisher = self.subscriber = None
        self._state = None
        self._error = None
        self._mode_machine = 0
        self._received_count = 0
        self._first_received = self._last_received = None
        self._mutex = threading.Lock()

    def initialize(self, config, interface, publisher):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC
        self.config, self._message, self._crc = config, unitree_hg_msg_dds__LowCmd_, CRC()
        ChannelFactoryInitialize(config['robot']['domain_id'], interface)
        self.subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        self.subscriber.Init(self._receive, 1)
        if publisher:
            self.publisher = ChannelPublisher('rt/arm_sdk', LowCmd_)
            self.publisher.Init()

    def _receive(self, message):
        try:
            state = JointState([message.motor_state[i].q for i in range(15, 29)], time.monotonic(),
                               [message.motor_state[i].dq for i in range(15, 29)],
                               [message.motor_state[i].q for i in range(12, 15)])
            with self._mutex:
                self._received_count += 1
                if self._first_received is None:
                    self._first_received = state.timestamp
                self._last_received = state.timestamp
                self._state = state
                self._mode_machine = message.mode_machine
        except Exception as exc:
            with self._mutex:
                self._error = exc

    def state(self):
        with self._mutex:
            if self._error:
                raise RuntimeError('invalid DDS state') from self._error
            if self._state is None:
                return None
            s = self._state
            return JointState(s.q, s.timestamp, s.dq, s.waist_q)

    def diagnostics(self):
        with self._mutex:
            elapsed = (self._last_received - self._first_received) if self._received_count > 1 else 0
            return dict(MODE_MACHINE=self._mode_machine,
                        LOWSTATE_RECEIVED=self._received_count > 0,
                        LOWSTATE_FREQUENCY=(self._received_count - 1) / elapsed if elapsed > 0 else None,
                        LOWSTATE_SAMPLE_COUNT=self._received_count)

    def write(self, command, waist, weight):
        if self.publisher is None:
            raise RuntimeError('read-only transport')
        message = self._message()
        message.mode_pr = 0
        message.mode_machine = self._mode_machine
        for offset, index in enumerate(range(15, 29)):
            motor = message.motor_cmd[index]
            motor.q, motor.dq = float(command.q[offset]), float(command.dq[offset])
            motor.kp, motor.kd, motor.tau = float(command.kp[offset]), float(command.kd[offset]), 0.0
        if waist is not None:
            from ..utils.hardware import active_waist
            for index in active_waist(self.config):
                offset = index - 12
                motor = message.motor_cmd[index]
                motor.q, motor.dq, motor.tau = float(waist[offset]), 0.0, 0.0
                motor.kp = float(self.config['waist']['kp'][offset])
                motor.kd = float(self.config['waist']['kd'][offset])
        message.motor_cmd[29].q = float(weight)
        message.crc = self._crc.Crc(message)
        if not self.publisher.Write(message):
            raise RuntimeError('DDS write failed')

    def close(self):
        try:
            if self.publisher is not None:
                self.publisher.Close()
        finally:
            if self.subscriber is not None:
                self.subscriber.Close()
            self.publisher = self.subscriber = None
