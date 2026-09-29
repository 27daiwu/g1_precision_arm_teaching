"""Arm SDK bilateral joint teaching with explicit lock and waypoint capture."""
import argparse
import json
from pathlib import Path
import queue
import signal
import sys
import threading
import time

import numpy as np

from .golden_cli import BufferedDiagnostics
from .sdk.golden_controller import ArmSdkGoldenController
from .sdk.golden_diagnostics import SafetyAbort
from .sdk.transport import UnitreeTransport
from .utils.config import load_config


MOVING_THRESHOLD = .05
BREAKAWAY_DEBOUNCE_S = .05
HOLD_GAIN_RAMP_S = .4
ARM_IDS = tuple(range(15, 29))
FINAL_KP = np.array([8. if i in (15, 16, 17, 18, 22, 23, 24, 25) else 6. for i in ARM_IDS])


class TeachFollower:
    """Per-joint velocity breakaway with explicit return to HOLDING."""
    def __init__(self, q):
        self.reference = np.asarray(q, dtype=float).copy()
        if self.reference.shape != (14,) or not np.isfinite(self.reference).all():
            raise ValueError('invalid initial arm reference')
        self.states = ['HOLDING'] * 14
        self.state_since = [0.] * 14
        self.transition_reason = ['TEACH_ENTRY'] * 14
        self.candidate_since = [None] * 14
        self.ramp_started = [None] * 14
        self.ramp_from = np.full(14, 40.)
        self.ramp_to = np.full(14, 40.)

    def update(self, q, dq, now):
        q, dq = np.asarray(q), np.asarray(dq)
        if q.shape != (14,) or dq.shape != (14,) or not np.isfinite(q).all() or not np.isfinite(dq).all():
            raise ValueError('invalid arm state')
        for i in range(14):
            if self.state_since[i] == 0.:
                self.state_since[i] = now
            velocity = abs(float(dq[i]))
            state = self.states[i]
            breakaway = velocity > MOVING_THRESHOLD
            if state == 'HOLDING' and breakaway:
                if self.candidate_since[i] is None:
                    self.candidate_since[i] = now
                if now - self.candidate_since[i] >= BREAKAWAY_DEBOUNCE_S:
                    current_kp = self.gains(now)[i]
                    self.reference[i] = float(q[i])
                    self.states[i] = 'MOVING'
                    self.state_since[i] = now
                    self.transition_reason[i] = 'VELOCITY'
                    self.candidate_since[i] = None
                    self.ramp_started[i] = now
                    self.ramp_from[i] = current_kp
                    self.ramp_to[i] = FINAL_KP[i]
            elif state == 'MOVING':
                self.candidate_since[i] = None
            elif state == 'HOLDING':
                self.candidate_since[i] = None
            if self.states[i] == 'MOVING':
                self.reference[i] = float(q[i])
        return self.reference.copy()

    def lock(self, q, now):
        captured = np.asarray(q, dtype=float).copy()
        if captured.shape != (14,) or not np.isfinite(captured).all():
            raise ValueError('invalid arm capture')
        current_kp = self.gains(now)
        self.reference = captured
        for i in range(14):
            self.states[i] = 'HOLDING'
            self.state_since[i] = now
            self.transition_reason[i] = 'EXPLICIT_LOCK'
            self.candidate_since[i] = None
            self.ramp_started[i] = now
            self.ramp_from[i] = current_kp[i]
            self.ramp_to[i] = 40.
        return captured.copy()

    def gains(self, now):
        values = np.empty(14)
        for i in range(14):
            if self.ramp_started[i] is None:
                values[i] = 40.
            else:
                s = min(1., max(0., (now - self.ramp_started[i]) / HOLD_GAIN_RAMP_S))
                values[i] = self.ramp_from[i] + (self.ramp_to[i] - self.ramp_from[i]) * (s*s*(3-2*s))
        return values


def run_teach(controller, inbox):
    waypoints = []
    started = controller.clock()
    shutdown_times = {}
    try:
        controller.capture_upper_body_pose()
        controller.acquire_current_pose()
        controller.hold_current_pose(.5)
        measured = controller._state()
        # Full ownership is held at Kp40; TEACH starts frozen at the measured pose.
        teach_entry_q = measured.q.copy()
        follower = TeachFollower(teach_entry_q)
        controller.set_teach_command(teach_entry_q, np.full(14, 40.))
        controller.emit(dict(event='TEACH_READY', waist_q_start=controller.q_hold[:3].tolist(),
                             waist_kp=list(controller.waist_kp_by_axis) if controller.waist_kp_by_axis else controller.static_hold_waist_kp or 60.,
                             shoulder_elbow_kp=8., wrist_kp=6., arm_kd=1.5,
                             arm_reference_at_entry=teach_entry_q.tolist()))
        waist_label = controller.waist_kp_by_axis or (controller.static_hold_waist_kp or 60.,)*3
        print(f'DUAL ARM TEACH MODE\nWAIST: 12..14 HOLD / Kp{waist_label}\nARMS: shoulder/elbow Kp8, wrist Kp6, Kd1.5\n'
              'M = lock and save waypoint; L = list count; Q = finish and release', flush=True)
        last_follower_log = controller.clock() - .2
        while True:
            now = controller.clock()
            state = controller._state()
            reference = follower.update(state.q, state.dq, now)
            finish = False
            while True:
                try:
                    action = inbox.get_nowait().strip().lower()
                except queue.Empty:
                    break
                if action in ('m', 'mark'):
                    captured = follower.lock(state.q, now)
                    reference = captured.copy()
                    point = dict(event='JOINT_WAYPOINT_MARK', index=len(waypoints)+1,
                                 timestamp=now, motor_ids=list(ARM_IDS),
                                 q_command_reference=captured.tolist(), q_measured_at_mark=captured.tolist(),
                                 reference_measurement_error=[0.] * 14, units='rad')
                    waypoints.append(point)
                    controller.emit(point)
                    print(f'[WAYPOINT {len(waypoints)}] LOCK + SAVE\ncaptured arm q: {captured.tolist()}\n'
                          'all arm joints -> HOLDING; gain ramp -> Kp40', flush=True)
                elif action in ('l', 'list'):
                    print(f'WAYPOINT_COUNT = {len(waypoints)}', flush=True)
                elif action in ('q', 'quit', 'exit'):
                    finish = True
                    break
            kp = follower.gains(now)
            controller.set_teach_command(reference, kp)
            controller._frame(1., 'TEACH')
            if now - last_follower_log >= .2:
                last_follower_log = now
                for i, motor_id in enumerate(ARM_IDS):
                    controller.emit(dict(event='FOLLOWER_STATE', timestamp=now, motor_id=motor_id,
                                         follower_state=follower.states[i], q_measured=float(state.q[i]),
                                         q_reference=float(reference[i]), tracking_error=float(state.q[i]-reference[i]),
                                         dq=float(state.dq[i]), kp_command=float(kp[i]),
                                         state_age=float(now-follower.state_since[i]),
                                         transition_reason=follower.transition_reason[i],
                                         units=dict(q='rad', error='rad', dq='rad/s')))
            if finish:
                return waypoints
            controller._profiled_sleep(controller.period_s)
    finally:
        shutdown_times['SHUTDOWN_REQUEST_TIME'] = controller.clock()
        shutdown_times['FREEZE_REFERENCE_TIME'] = shutdown_times['SHUTDOWN_REQUEST_TIME']
        release_start = controller.clock()
        try:
            controller.release()
        finally:
            shutdown_times['RELEASE_START_TIME'] = release_start
            shutdown_times['FINAL_WEIGHT_ZERO_TIME'] = controller.clock()
            writes = controller.diagnostics.writes
            controller.emit(dict(event='DUAL_ARM_TEACH_SUMMARY', teach_duration_s=controller.clock()-started,
                                 waypoint_count=len(waypoints), waist_q_start=None if controller.q_hold is None else controller.q_hold[:3].tolist(),
                                 waist_max_delta=None if controller.q_hold is None else controller.max_delta[:3].tolist(),
                                 hard_safety_abort=controller.diagnostics.first_trigger is not None,
                                 dds_write_failure=any(w['outcome']!='SUCCESS' for w in writes),
                                 final_weight_zero_write=bool(writes and writes[-1]['phase']=='RELEASE' and writes[-1]['weight']==0 and writes[-1]['outcome']=='SUCCESS'),
                                 shutdown_timing=shutdown_times,
                                 release_duration_ms=(shutdown_times['FINAL_WEIGHT_ZERO_TIME']-release_start)*1000.))


def _input_worker(inbox, stop_event):
    for line in sys.stdin:
        if stop_event.is_set():
            break
        inbox.put(line)
    if not stop_event.is_set():
        inbox.put('q')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Dual arm kinesthetic teaching via rt/arm_sdk')
    parser.add_argument('interface')
    args = parser.parse_args(argv)
    config = load_config('configs')
    robot, waist = config['robot'], config['waist']
    if robot['domain_id'] != 0 or not all(robot[k] for k in ('hardware_reviewed', 'model_confirmed', 'arm_joint_mapping_verified')) or not waist['configuration_verified'] or waist['active_joints'] != [12,13,14]:
        raise ValueError('Golden hardware/mapping prerequisites are not verified')
    path = Path(f'logs/dual_arm_teach_{time.time_ns()}.jsonl')
    path.parent.mkdir(parents=True, exist_ok=True)
    transport, controller, previous = UnitreeTransport(), None, {}
    input_stop = threading.Event()
    input_thread = None
    with path.open('x', encoding='utf-8') as stream:
        buffered = BufferedDiagnostics()
        try:
            transport.initialize(config, args.interface, True)
            controller = ArmSdkGoldenController(transport, config, emit=buffered.emit, real_dds=True,
                                                 motor14_kp=60., teach_waist=True,
                                                 waist_kp_by_axis=(80., 80., 100.))
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous[sig] = signal.signal(sig, lambda signum, frame: controller.stop())
            inbox = queue.Queue()
            input_thread = threading.Thread(target=_input_worker, args=(inbox, input_stop), daemon=True)
            input_thread.start()
            exit_reason = None
            try:
                run_teach(controller, inbox)
            except SafetyAbort as exc:
                exit_reason = 'SAFETY_ABORT'
                print(f'SAFETY_ABORT = {exc}', flush=True)
            except (InterruptedError, KeyboardInterrupt):
                exit_reason = 'REQUEST_SAFE_SHUTDOWN'
            print(f'RESPONSE_LOG = {path}')
            return 130 if controller.stopped or exit_reason else 0
        finally:
            try:
                if controller is not None:
                    try:
                        controller.release()
                    finally:
                        try:
                            try:
                                controller.report_diagnostics()
                            except Exception as exc:
                                print(f'POST_RUN_DIAGNOSTICS_FAILED = {exc}', flush=True)
                        finally:
                            buffered.finish(stream)
            finally:
                input_stop.set()
                if input_thread is not None:
                    input_thread.join(timeout=0.5)
                transport.shutdown()
                for sig, handler in previous.items():
                    signal.signal(sig, handler)
