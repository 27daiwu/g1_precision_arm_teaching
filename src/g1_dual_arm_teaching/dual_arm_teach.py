"""Arm SDK bilateral joint teaching; follower semantics from g1_piano ArmController."""
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
from .sdk.transport import UnitreeTransport
from .utils.config import load_config


FOLLOW_RATE = .25
FOLLOW_THRESHOLD = .015
MOVING_THRESHOLD = .05
RELEASE_DQ_THRESHOLD = .03
RELEASE_STABLE_S = .5
REENTER_MOVING_S = .15
GAIN_RAMP_S = 1.
ARM_IDS = tuple(range(15, 29))
FINAL_KP = np.array([8. if i in (15, 16, 17, 18, 22, 23, 24, 25) else 6. for i in ARM_IDS])


class TeachFollower:
    """Per-joint gate and rate-limited reference update from the old TEACH controller."""
    def __init__(self, q):
        self.reference = np.asarray(q, dtype=float).copy()
        if self.reference.shape != (14,) or not np.isfinite(self.reference).all():
            raise ValueError('invalid initial arm reference')
        self.states = ['TEACH_IDLE'] * 14
        self.active = [False] * 14
        self.candidate_since = [None] * 14
        self.reentry_since = [None] * 14

    def update(self, q, dq, now, dt):
        q, dq = np.asarray(q), np.asarray(dq)
        if q.shape != (14,) or dq.shape != (14,) or not np.isfinite(q).all() or not np.isfinite(dq).all():
            raise ValueError('invalid arm state')
        for i in range(14):
            velocity = abs(float(dq[i]))
            state = self.states[i]
            if state == 'TEACH_IDLE' and velocity > MOVING_THRESHOLD:
                self.states[i], self.active[i] = 'TEACH_MOVING', True
            elif state == 'TEACH_MOVING' and velocity < RELEASE_DQ_THRESHOLD:
                self.states[i], self.candidate_since[i] = 'TEACH_RELEASE_CANDIDATE', now
            elif state == 'TEACH_RELEASE_CANDIDATE':
                if velocity > MOVING_THRESHOLD:
                    self.states[i], self.candidate_since[i] = 'TEACH_MOVING', None
                elif velocity < RELEASE_DQ_THRESHOLD and now - self.candidate_since[i] >= RELEASE_STABLE_S:
                    self.states[i], self.active[i], self.reentry_since[i] = 'TEACH_RELEASE_HOLD', False, None
            elif state == 'TEACH_RELEASE_HOLD' and velocity > MOVING_THRESHOLD:
                self.reentry_since[i] = self.reentry_since[i] or now
                if now - self.reentry_since[i] >= REENTER_MOVING_S:
                    self.states[i], self.active[i], self.reentry_since[i] = 'TEACH_MOVING', True, None
            if self.active[i]:
                error = float(q[i] - self.reference[i])
                if abs(error) > FOLLOW_THRESHOLD:
                    self.reference[i] += float(np.clip(error, -FOLLOW_RATE*dt, FOLLOW_RATE*dt))
        return self.reference.copy()


def run_teach(controller, inbox):
    waypoints = []
    started = controller.clock()
    try:
        controller.capture_upper_body_pose()
        controller.acquire_current_pose()
        controller.hold_current_pose(.5)
        measured = controller._state()
        arm_q = measured.q.copy()
        ramp_start = controller.clock()
        while True:
            elapsed = controller.clock() - ramp_start
            s = controller.smoothstep(elapsed/GAIN_RAMP_S)
            controller.set_teach_command(arm_q, 40. + (FINAL_KP-40.)*s)
            controller._frame(1., 'TEACH_GAIN_RAMP')
            if elapsed >= GAIN_RAMP_S:
                break
            controller._profiled_sleep(controller.period_s)
        follower = TeachFollower(arm_q)
        controller.emit(dict(event='TEACH_READY', waist_q_start=controller.q_hold[:3].tolist(),
                             waist_kp=60., shoulder_elbow_kp=8., wrist_kp=6., arm_kd=1.5,
                             follow_rate=FOLLOW_RATE, follow_threshold=FOLLOW_THRESHOLD))
        print('DUAL ARM TEACH MODE\nWAIST: 12..14 HOLD / Kp60\nARMS: shoulder/elbow Kp8, wrist Kp6, Kd1.5\n'
              'FOLLOW_RATE=0.25, FOLLOW_THRESHOLD=0.015\nM = mark waypoint; L = list count; Q = finish and release', flush=True)
        previous = controller.clock()
        while True:
            now = controller.clock()
            state = controller._state()
            reference = follower.update(state.q, state.dq, now, now-previous)
            previous = now
            controller.set_teach_command(reference, FINAL_KP)
            controller._frame(1., 'TEACH')
            while True:
                try:
                    action = inbox.get_nowait().strip().lower()
                except queue.Empty:
                    break
                if action in ('m', 'mark'):
                    measured = controller._state().q.copy()
                    point = dict(event='JOINT_WAYPOINT_MARK', index=len(waypoints)+1,
                                 timestamp=controller.clock(), motor_ids=list(ARM_IDS),
                                 q_command_reference=reference.tolist(), q_measured_at_mark=measured.tolist(),
                                 reference_measurement_error=(reference-measured).tolist(), units='rad')
                    waypoints.append(point)
                    controller.emit(point)
                    print(f"WAYPOINT_COUNT = {len(waypoints)}", flush=True)
                elif action in ('l', 'list'):
                    print(f'WAYPOINT_COUNT = {len(waypoints)}', flush=True)
                elif action in ('q', 'quit', 'exit'):
                    return waypoints
            controller._profiled_sleep(controller.period_s)
    finally:
        try:
            controller.release()
        finally:
            writes = controller.diagnostics.writes
            controller.emit(dict(event='DUAL_ARM_TEACH_SUMMARY', teach_duration_s=controller.clock()-started,
                                 waypoint_count=len(waypoints), waist_q_start=None if controller.q_hold is None else controller.q_hold[:3].tolist(),
                                 waist_max_delta=None if controller.q_hold is None else controller.max_delta[:3].tolist(),
                                 hard_safety_abort=controller.diagnostics.first_trigger is not None,
                                 dds_write_failure=any(w['outcome']!='SUCCESS' for w in writes),
                                 final_weight_zero_write=bool(writes and writes[-1]['phase']=='RELEASE' and writes[-1]['weight']==0 and writes[-1]['outcome']=='SUCCESS')))


def _input_worker(inbox):
    for line in sys.stdin:
        inbox.put(line)
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
    with path.open('x', encoding='utf-8') as stream:
        buffered = BufferedDiagnostics()
        try:
            transport.initialize(config, args.interface, True)
            controller = ArmSdkGoldenController(transport, config, emit=buffered.emit, real_dds=True, motor14_kp=60., teach_waist=True)
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous[sig] = signal.signal(sig, lambda signum, frame: controller.stop())
            inbox = queue.Queue()
            threading.Thread(target=_input_worker, args=(inbox,), daemon=True).start()
            run_teach(controller, inbox)
            print(f'RESPONSE_LOG = {path}')
            return 130 if controller.stopped else 0
        finally:
            try:
                if controller is not None:
                    try:
                        controller.release()
                    finally:
                        try:
                            controller.report_diagnostics()
                        finally:
                            buffered.finish(stream)
            finally:
                transport.close()
                for sig, handler in previous.items():
                    signal.signal(sig, handler)
