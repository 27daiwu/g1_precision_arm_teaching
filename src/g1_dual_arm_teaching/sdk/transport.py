"""DDS details stay in sdk/. Import Unitree only for explicit hardware use."""
import time
from contextlib import nullcontext
import threading
import json
import logging
import numpy as np
from .arm_sdk_types import JointState, ArmJointCommand
from .wire_audit import ARM_MOTOR_IDS, guard, compare, lowcmd_snapshot, snapshot_json


class SimulationTransport:
    """Ideal position follower, not a dynamics or accuracy validation model."""
    def __init__(self, q=None, waist_q=None):
        self.q = np.zeros(14) if q is None else np.array(q, dtype=float)
        self.waist = np.zeros(3) if waist_q is None else np.array(waist_q, dtype=float)
        self.closed = False

    def initialize(self, config, interface, publisher):
        self.config = config

    def state(self):
        return JointState(self.q, time.monotonic(), np.zeros(14), self.waist, np.zeros(3))

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
        self.acquire_reference = None
        self.first_write_check = None
        self._wire_sequence = 0
        self._last_wire_weight = None
        self.snapshots = []
        self._closed = threading.Event()

    def initialize(self, config, interface, publisher):
        self._closed.clear()
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
            state = JointState([message.motor_state[i].q for i in ARM_MOTOR_IDS], time.monotonic(),
                               [message.motor_state[i].dq for i in ARM_MOTOR_IDS],
                               [message.motor_state[i].q for i in range(12, 15)],
                               [message.motor_state[i].dq for i in range(12, 15)],
                               [message.motor_state[i].q for i in range(29)])
            with self._mutex:
                self._received_count += 1
                if self._first_received is None:
                    self._first_received = state.timestamp
                self._last_received = state.timestamp
                self._state = state
                self._mode_machine = message.mode_machine
        except Exception as exc:
            if self._closed.is_set():
                logging.getLogger(__name__).debug('READER_STOPPED: sample arrived during shutdown')
                return
            with self._mutex:
                self._error = exc

    def state(self):
        profiler = getattr(self, 'profiler', None)
        with profiler.measure('state_lock') if profiler is not None else nullcontext():
            self._mutex.acquire()
        try:
            if self._error:
                raise RuntimeError('invalid DDS state') from self._error
            if self._state is None:
                return None
            s = self._state
            return JointState(s.q, s.timestamp, s.dq, s.waist_q, s.waist_dq, s.full_q)
        finally:
            self._mutex.release()

    def diagnostics(self):
        with self._mutex:
            elapsed = (self._last_received - self._first_received) if self._received_count > 1 else 0
            return dict(MODE_MACHINE=self._mode_machine,
                        LOWSTATE_RECEIVED=self._received_count > 0,
                        LOWSTATE_FREQUENCY=(self._received_count - 1) / elapsed if elapsed > 0 else None,
                        LOWSTATE_SAMPLE_COUNT=self._received_count)

    def build_message(self, command, waist, weight):
        command = ArmJointCommand(command.q, command.dq, command.kp, command.kd, command.tau_ff)
        if not np.isfinite(weight) or not 0 <= weight <= 1:
            raise ValueError('invalid wire weight')
        message = self._message()
        if len({id(m) for m in message.motor_cmd}) != len(message.motor_cmd):
            raise ValueError('ABORT_BEFORE_DDS_WRITE: aliased motor slots')
        message.mode_pr = 0
        # Experimental arm-only path leaves mode_machine at the SDK default.
        for offset, index in enumerate(ARM_MOTOR_IDS):
            motor = message.motor_cmd[index]
            motor.q, motor.dq = float(command.q[offset]), float(command.dq[offset])
            motor.kp, motor.kd, motor.tau = float(command.kp[offset]), float(command.kd[offset]), 0.0
        # EXPERIMENTAL_ARM_ONLY_VARIANT. Motors 12..14 retain the exact SDK constructor defaults.
        # The waist argument remains only for API compatibility and is intentionally ignored.
        message.motor_cmd[29].q = float(weight)
        message.crc = self._crc.Crc(message)
        return message

    def dump_command(self, command, waist, weight, acquire_q, official_style=False):
        message = self.build_message(command, waist, weight)
        data = compare(message, acquire_q)
        decoded = type(message).deserialize(message.serialize())
        data['CDR_ROUNDTRIP'] = compare(decoded, acquire_q)
        if official_style:
            from .phase0_acquire import first_write_diagnostics
            data.update(first_write_diagnostics(guard(message, command, acquire_q), acquire_q, self.state(), self.config))
        data['DDS_WRITE'] = 'NO'
        data['COMMAND_PUBLISHER_CREATED'] = 'NO' if self.publisher is None else 'YES'
        try:
            guard(message, command, acquire_q)
            guard(decoded, command, acquire_q)
            data['WIRE_GUARD'] = 'PASS'
        except ValueError as exc:
            data['WIRE_GUARD'] = 'FAIL'
            data['WIRE_GUARD_ERROR'] = str(exc)
            data['ALL_ARM_WIRE_Q_MATCH_ACQUIRE'] = 'NO'
        return data

    def write(self, command, waist, weight):
        if self.publisher is None:
            raise RuntimeError('read-only transport')
        message = self.build_message(command, waist, weight)
        wire = guard(message, command, self.acquire_reference)
        if hasattr(message, 'serialize'):
            decoded = type(message).deserialize(message.serialize())
            guard(decoded, command, self.acquire_reference)
        if message.motor_cmd[29].q != float(weight):
            raise ValueError('ABORT_BEFORE_DDS_WRITE: weight mismatch')
        logging.getLogger(__name__).info('FINAL_WIRE_PAYLOAD %s', json.dumps(wire, allow_nan=False))
        label = None
        if self._wire_sequence == 0:
            label = 'FIRST_FRAME'
        elif self._last_wire_weight == 1.0 and float(weight) < 1.0:
            label = 'RELEASE_FRAME'
        elif self._wire_sequence == 1:
            label = 'STEADY_STATE_FRAME'
        if label is not None:
            snapshot = lowcmd_snapshot(message, label, self._wire_sequence, time.monotonic())
            self.snapshots.append(snapshot)
            logging.getLogger(__name__).info('LOWCMD_SNAPSHOT %s', snapshot_json(snapshot))
        if self.first_write_check is not None:
            self.first_write_check(wire)
        if not self.publisher.Write(message):
            raise RuntimeError('DDS write failed')
        self.first_write_check = None
        self._last_wire_weight = float(weight)
        self._wire_sequence += 1

    def close(self):
        self._closed.set()
        try:
            if self.publisher is not None:
                self.publisher.Close()
        finally:
            if self.subscriber is not None:
                self.subscriber.Close()
            self.publisher = self.subscriber = None

    def shutdown(self, timeout=1.0):
        """Explicit bounded reader/publisher shutdown hook."""
        self.close()
        return True
