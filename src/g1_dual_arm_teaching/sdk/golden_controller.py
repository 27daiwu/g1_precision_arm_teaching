"""Golden current-pose reproduction; watchdogs observe or abort only."""
import copy
import logging
import time
import numpy as np
from .golden_reference import fill_golden_frame, GOLDEN_RAMP_S
from .wire_audit import lowcmd_snapshot
from .golden_diagnostics import GoldenDiagnostics, SafetyAbort
from .cycle_profile import CycleProfiler, Motor14Diagnostics, profiled
from .runtime_guard import RuntimeWireGuard


class ArmSdkGoldenController:
    period_s = .02

    def __init__(self, transport, config, *, clock=time.monotonic, sleep=time.sleep, emit=None, real_dds=False, motor14_kp=40., teach_waist=False, static_hold_waist_kp=None, static_hold_arm_kp=None, waist_kp_by_axis=None):
        if motor14_kp not in (40., 50., 60.):
            raise ValueError('invalid diagnostic motor14 Kp')
        self.transport, self.config = transport, config
        self.clock, self.sleep = clock, sleep
        self.profiler = CycleProfiler(clock)
        self.transport.profiler = self.profiler
        self.motor14 = Motor14Diagnostics()
        self._emit_callback = emit or (lambda data: None)
        self.emit = self._profiled_emit
        self.cmd = transport._message()  # Exactly one persistent SDK LowCmd.
        self.defaults = copy.deepcopy(self.cmd)
        self.q_hold = None
        self.weight = 0.
        self.sequence = 0
        self.attempted_write = False
        self.stopped = False
        self.released = False
        self.max_delta = np.zeros(17)
        self.max_velocity = np.zeros(17)
        self.phase = 'ACQUIRE'
        self.diagnostics = GoldenDiagnostics(config, clock, lambda data: self.emit(data), real_dds)
        self._diagnostics_reported = False
        self._release_logging_failed = False
        self.runtime_guard = None
        self.representatives = {}
        self.guard_timings = []
        self.motor14_kp = float(motor14_kp)
        self.teach_waist = bool(teach_waist)
        self.static_hold_waist_kp = static_hold_waist_kp
        self.static_hold_arm_kp = static_hold_arm_kp
        if waist_kp_by_axis is not None and (len(waist_kp_by_axis) != 3 or
                not all(np.isfinite(kp) and kp > 0 for kp in waist_kp_by_axis)):
            raise ValueError('invalid per-axis waist Kp')
        self.waist_kp_by_axis = None if waist_kp_by_axis is None else tuple(float(kp) for kp in waist_kp_by_axis)
        self.selected_q = None
        self.representative_q = {}
        self.teach_q = None
        self.teach_kp = None
        self.representative_teach = {}

    def set_teach_command(self, q_arm, kp_arm):
        if self.weight != 1.:
            raise ValueError('teach command requires full ownership')
        q = np.asarray(q_arm, dtype=float)
        kp = np.asarray(kp_arm, dtype=float)
        if q.shape != (14,) or kp.shape != (14,) or not np.isfinite(q).all() or not np.isfinite(kp).all():
            raise ValueError('invalid teach command')
        expected = np.array([8. if i in (15,16,17,18,22,23,24,25) else 6. for i in range(15,29)])
        if np.any(kp < expected) or np.any(kp > 40.):
            raise ValueError('invalid teach gain ramp')
        self.teach_q = q.copy()
        self.teach_kp = kp.copy()
        self.runtime_guard.teach_q = self.teach_q
        self.runtime_guard.teach_kp = self.teach_kp

    @profiled('logging')
    def _profiled_emit(self, data):
        self._emit_callback(data)

    @profiled('state_check')
    def _state(self):
        return self.transport.state()

    @profiled('sleep')
    def _profiled_sleep(self, duration):
        self.sleep(duration)

    def stop(self):
        self.stopped = True

    def wait_for_lowstate(self):
        start = self.clock()
        while True:
            state = self._state()
            if state is not None:
                self._observe(state)
                return state
            if self.stopped:
                raise InterruptedError('stop requested')
            if self.clock() - start >= 5.:
                raise RuntimeError('LOWSTATE_TIMEOUT')
            self._profiled_sleep(.01)

    @profiled('diagnostics')
    def _observe(self, state, first=False):
        violations, q, dq, delta, age = self.diagnostics.inspect(
            state, self.q_hold, self.phase, self.weight, first)
        warnings = [v for v in violations if v.get('severity') == 'WARNING']
        violations = [v for v in violations if v.get('severity') != 'WARNING']
        with self.profiler.measure('logging'):
            self.diagnostics.latch(violations)
            self.diagnostics.warn(warnings)
        if q is not None:
            teach_tracking = None if self.phase != 'TEACH' or self.teach_q is None else (q[3:]-self.teach_q).tolist()
            self.motor14.observe(self.phase, q[2], dq[2], state.timestamp)
            self.max_delta = np.maximum(self.max_delta, np.abs(delta))
            self.max_velocity = np.maximum(self.max_velocity, np.abs(dq))
            self.emit(dict(event='STATE', timestamp=self.clock(), state_timestamp=state.timestamp,
                           state_age_ms=age*1000, phase=self.phase, weight=self.weight,
                           safety_latch='LATCHED' if self.diagnostics.first_trigger else 'CLEAR',
                           violations=violations, warnings=warnings, q=q.tolist(), dq=dq.tolist(), delta_q=delta.tolist(),
                           acquire_q=None if self.q_hold is None else self.q_hold.tolist(),
                           max_abs_delta=self.max_delta.tolist(), max_abs_dq=self.max_velocity.tolist(),
                           arm_tracking_error=teach_tracking,
                           motor_ids=list(range(12, 29))))
        if violations:
            raise SafetyAbort(violations[0]['abort_code'])
        return q.copy()

    def capture_upper_body_pose(self):
        state = self.wait_for_lowstate()
        self.diagnostics.capture_time = self.clock()
        self.diagnostics.capture_state_time = state.timestamp
        self.q_hold = self._observe(state, first=True)
        self.diagnostics.q_min = self.q_hold.copy()
        self.diagnostics.q_max = self.q_hold.copy()
        self.q_hold.setflags(write=False)
        self.motor14.capture(self.q_hold[2])
        self.runtime_guard = RuntimeWireGuard(self.defaults, self.q_hold, self.motor14_kp, teach_waist=self.teach_waist,
                                              static_hold_waist_kp=self.static_hold_waist_kp,
                                              static_hold_arm_kp=self.static_hold_arm_kp,
                                              waist_kp_by_axis=self.waist_kp_by_axis)
        return self.q_hold.copy()

    @profiled('wire_guard')
    def _full_guard(self, message, weight):
        for i in range(len(message.motor_cmd)):
            motor = message.motor_cmd[i]
            default = self.defaults.motor_cmd[i]
            for field in ('q', 'dq', 'kp', 'kd', 'tau', 'mode', 'reserve'):
                if not hasattr(default, field):
                    continue
                expected = getattr(default, field)
                if 12 <= i <= 28 and field in ('q', 'dq', 'kp', 'kd', 'tau'):
                    static_kp = (self.waist_kp_by_axis[i-12] if self.waist_kp_by_axis is not None and 12 <= i <= 14
                                 else self.static_hold_waist_kp if self.static_hold_waist_kp is not None and i in (12, 13, 14)
                                 else self.static_hold_arm_kp if self.static_hold_arm_kp is not None and 15 <= i <= 28
                                 else None)
                    expected = dict(q=self.q_hold[i-12], dq=0., kp=static_kp if static_kp is not None else 60. if self.teach_waist and i in (12,13) else self.motor14_kp if i == 14 else 40., kd=1.5, tau=0.)[field]
                    if i == 19 and field == 'q' and self.selected_q is not None:
                        expected = self.selected_q
                    if 15 <= i <= 28 and self.teach_q is not None and field == 'q':
                        expected = self.teach_q[i-15]
                    if 15 <= i <= 28 and self.teach_kp is not None and field == 'kp':
                        expected = self.teach_kp[i-15]
                elif i == 29 and field == 'q':
                    expected = weight
                if not np.allclose(getattr(motor, field), expected, rtol=0, atol=1e-6):
                    raise ValueError(f'GOLDEN_WIRE_GUARD motor{i}.{field}')
        for field in ('mode_pr', 'mode_machine', 'reserve'):
            if hasattr(self.defaults, field) and not np.array_equal(getattr(message, field), getattr(self.defaults, field)):
                raise ValueError(f'GOLDEN_WIRE_GUARD {field}')

    @profiled('wire_guard')
    def _guard(self, message, weight):
        start = self.clock()
        self.runtime_guard.check(message, weight)
        self.guard_timings.append((self.clock()-start)*1e6)

    def _roundtrip(self, message, weight):
        if not hasattr(message, 'serialize'):
            if self.diagnostics.real_dds:
                raise ValueError('GOLDEN_WIRE_GUARD real DDS requires CDR support')
            return 'UNAVAILABLE_SIMULATION'
        with self.profiler.measure('serialization'):
            encoded = message.serialize()
        with self.profiler.measure('deserialization'):
            decoded = type(message).deserialize(encoded)
        self._full_guard(decoded, weight)
        self.runtime_guard.check(decoded, weight)
        if decoded.crc != message.crc or self.transport._crc.Crc(decoded) != message.crc:
            raise ValueError('GOLDEN_WIRE_GUARD CRC/CDR mismatch')
        return 'PASS'

    def _representative_label(self, weight, phase, final=False):
        if not self.representatives:
            return 'FIRST_ACQUIRE'
        if phase == 'ACQUIRE':
            for threshold, label in ((.1, 'WEIGHT_0P1'), (.5, 'WEIGHT_0P5')):
                if weight >= threshold and label not in self.representatives:
                    return label
        if phase == 'ACQUIRE_COMPLETE':
            return 'WEIGHT_1P0'
        if phase == 'HOLD' and 'HOLD_SAMPLE' not in self.representatives:
            return 'HOLD_SAMPLE'
        if phase == 'RELEASE':
            if final:
                return 'FINAL_ZERO'
            if 'RELEASE_SAMPLE' not in self.representatives:
                return 'RELEASE_SAMPLE'
        return None

    def post_run_audit(self):
        records = []
        selected_q = self.selected_q
        teach_q, teach_kp = self.teach_q, self.teach_kp
        for label, message in self.representatives.items():
            weight = float(message.motor_cmd[29].q)
            self.selected_q = self.representative_q[label]
            self.runtime_guard.selected_q = self.selected_q
            self.teach_q, self.teach_kp = self.representative_teach[label]
            self.runtime_guard.teach_q, self.runtime_guard.teach_kp = self.teach_q, self.teach_kp
            self.runtime_guard.check(message, weight)
            self._full_guard(message, weight)
            cdr = self._roundtrip(message, weight)
            records.append(dict(label=label, fields='PASS', crc_cdr=cdr))
        self.selected_q = selected_q
        self.runtime_guard.selected_q = selected_q
        self.teach_q, self.teach_kp = teach_q, teach_kp
        self.runtime_guard.teach_q, self.runtime_guard.teach_kp = teach_q, teach_kp
        return dict(event='POST_RUN_FULL_WIRE_AUDIT', records=records,
                    result=('PASS' if all(r['crc_cdr']=='PASS' for r in records) else 'FIELDS_PASS_CDR_UNAVAILABLE') if records else 'NO_COMMAND_SNAPSHOTS')

    @profiled('prepare')
    def _frame(self, weight, phase, publish=True, final=False, selected_q=None):
        self.phase = 'ACQUIRE' if phase in ('ACQUIRE', 'ACQUIRE_COMPLETE', 'FIRST_FRAME') else phase
        fill_golden_frame(self.cmd, self.q_hold, weight)
        self.cmd.motor_cmd[14].kp = self.motor14_kp
        if self.teach_waist:
            self.cmd.motor_cmd[12].kp = self.cmd.motor_cmd[13].kp = 60.
        if self.static_hold_waist_kp is not None:
            for motor_id in (12, 13, 14):
                self.cmd.motor_cmd[motor_id].kp = float(self.static_hold_waist_kp)
        if self.waist_kp_by_axis is not None:
            for offset, motor_id in enumerate((12, 13, 14)):
                self.cmd.motor_cmd[motor_id].kp = self.waist_kp_by_axis[offset]
        if self.static_hold_arm_kp is not None:
            for motor_id in range(15, 29):
                self.cmd.motor_cmd[motor_id].kp = float(self.static_hold_arm_kp)
        if selected_q is not None:
            if phase not in ('MOVE_TO_TARGET', 'TARGET_HOLD', 'RETURN_TO_START') or weight != 1. or self.weight != 1.:
                raise ValueError('Phase 1 motor19 override requires full ownership and a motion phase')
            if not np.isfinite(selected_q):
                raise ValueError('invalid selected joint command')
            self.selected_q = float(selected_q)
        if self.selected_q is not None:
            self.cmd.motor_cmd[19].q = self.selected_q
        if self.teach_q is not None:
            for offset, motor_id in enumerate(range(15, 29)):
                self.cmd.motor_cmd[motor_id].q = float(self.teach_q[offset])
                self.cmd.motor_cmd[motor_id].kp = float(self.teach_kp[offset])
        self.runtime_guard.selected_q = self.selected_q
        self._guard(self.cmd, weight)
        if self.sequence == 0:
            self._full_guard(self.cmd, weight)
        with self.profiler.measure('crc'):
            self.cmd.crc = self.transport._crc.Crc(self.cmd)
        if self.sequence == 0:
            self._roundtrip(self.cmd, weight)
        if phase != 'RELEASE':
            if self.stopped:
                raise InterruptedError('stop requested')
            self._observe(self._state(), first=self.sequence == 0)
        snapshot = None
        label = self._representative_label(weight, phase, final)
        if label is not None:
            # At most seven independent snapshots; the live cmd remains persistent.
            with self.profiler.measure('allocation'):
                self.representatives[label] = copy.deepcopy(self.cmd)
                self.representative_q[label] = self.selected_q
                self.representative_teach[label] = (None if self.teach_q is None else self.teach_q.copy(),
                                                      None if self.teach_kp is None else self.teach_kp.copy())
            snapshot = lowcmd_snapshot(self.cmd, phase, self.sequence, self.clock())
            snapshot.update(representative=label,
                            FIRST_COMMAND_GUARD='PASS' if self.sequence == 0 else None,
                            DDS_WRITE='PENDING' if publish else 'NO',
                            COMMAND_PUBLISHER_CREATED='YES' if self.transport.publisher is not None else 'NO')
            try:
                self.emit(snapshot)
            except Exception:
                if phase != 'RELEASE':
                    raise
                if not self._release_logging_failed:
                    self._release_logging_failed = True
                    logging.exception('Release snapshot logging failed; continuing release')
        if publish:
            if self.transport.publisher is None:
                raise RuntimeError('read-only Golden transport')
            # Set before Write: a transport exception may occur after transmission.
            self.attempted_write = True
            self.weight = weight
            record = dict(event='DDS_WRITE_RESULT', sequence=self.sequence, phase=phase,
                          real_dds=self.diagnostics.real_dds, weight=float(weight))
            write_start = self.clock()
            try:
                with self.profiler.measure('dds_write'):
                    result = self.transport.publisher.Write(self.cmd)
            except BaseException as exc:
                write_end = self.clock()
                record.update(outcome='EXCEPTION', error=str(exc))
                raise
            else:
                write_end = self.clock()
                record.update(outcome='FAILED' if result is False else 'SUCCESS')
            finally:
                record.update(write_start=write_start, write_end=write_end,
                              write_duration_s=write_end-write_start)
                # Buffer timing in memory; do not add per-frame disk I/O at Write.
                self.diagnostics.writes.append(record)
            if result is False:
                raise RuntimeError('DDS write failed')
            self.sequence += 1
        return snapshot

    def dump_first_command(self):
        if self.q_hold is None:
            self.capture_upper_body_pose()
        return self._frame(0., 'FIRST_FRAME', publish=False)

    @staticmethod
    def smoothstep(x):
        x = max(0., min(1., x))
        return x*x*(3.-2.*x)

    def acquire_current_pose(self):
        if self.q_hold is None:
            self.capture_upper_body_pose()
        self._frame(0., 'ACQUIRE')
        start = self.clock()
        self.diagnostics.acquire_start = start
        while True:
            self._profiled_sleep(self.period_s)
            elapsed = self.clock() - start
            if elapsed >= GOLDEN_RAMP_S:
                break
            self._frame(self.smoothstep(elapsed/GOLDEN_RAMP_S), 'ACQUIRE')
        self._frame(1., 'ACQUIRE_COMPLETE')

    def hold_current_pose(self, duration):
        if not np.isfinite(duration) or duration < 0:
            raise ValueError('invalid hold duration')
        start = self.clock()
        while self.clock() - start < duration:
            self._frame(1., 'HOLD')
            self._profiled_sleep(self.period_s)

    def release(self):
        if not self.attempted_write or self.released:
            return
        start, start_weight = self.clock(), self.weight
        self.diagnostics.release_start = start
        try:
            while self.clock() - start < GOLDEN_RAMP_S:
                ratio = self.smoothstep((self.clock()-start)/GOLDEN_RAMP_S)
                self._frame(start_weight*(1.-ratio), 'RELEASE')
                try:
                    self._observe(self._state())
                except SafetyAbort:
                    pass  # First evidence is latched; continue every release command.
                except Exception as exc:
                    try:
                        self.diagnostics.record_other(exc, 'RELEASE', self.weight)
                    except Exception:
                        if not self._release_logging_failed:
                            self._release_logging_failed = True
                            logging.exception('Release diagnostic logging failed; continuing release')
                self._profiled_sleep(self.period_s)
        finally:
            self._frame(0., 'RELEASE', final=True)
            self._profiled_sleep(.05)
            self.released = True

    def run(self, duration):
        try:
            self.acquire_current_pose()
            self.hold_current_pose(duration)
        except SafetyAbort:
            raise
        except Exception as exc:
            if not isinstance(exc, InterruptedError):
                try:
                    self.diagnostics.record_other(exc, self.phase, self.weight)
                except Exception:
                    logging.exception('Could not persist first failure diagnostics')
            raise
        finally:
            self.release()

    def report_diagnostics(self):
        """Emit buffered actual Write results and one final summary after release."""
        if self._diagnostics_reported:
            return
        self._diagnostics_reported = True
        profiles = self.profiler.reports(self.diagnostics.writes)
        for record in profiles:
            self.emit(record)
            if record['dt_gt_30ms']:
                self.emit(dict(record, event='LONG_COMMAND_INTERVAL'))
        audit_error = None
        try:
            self.emit(self.post_run_audit())
        except Exception as exc:
            audit_error = exc
            self.emit(dict(event='POST_RUN_FULL_WIRE_AUDIT', result='FAIL', error=str(exc)))
        self.emit(dict(event='WIRE_GUARD_PROFILE', runtime_count=len(self.guard_timings),
                       runtime_mean_us=float(np.mean(self.guard_timings)) if self.guard_timings else None,
                       runtime_max_us=max(self.guard_timings) if self.guard_timings else None,
                       expected_command_build='ONCE_AT_CAPTURE',
                       serialization='FIRST_FRAME_AND_POST_RUN_ONLY',
                       deserialization='FIRST_FRAME_AND_POST_RUN_ONLY'))
        self.emit(self.profiler.summary(profiles))
        self.emit(dict(self.motor14.summary(), source='PROJECT_GOLDEN'))
        for record in self.diagnostics.writes:
            self.emit(record)
        summary = self.diagnostics.summary(self.q_hold)
        self.emit(summary)
        if audit_error is not None:
            raise audit_error
        return summary
