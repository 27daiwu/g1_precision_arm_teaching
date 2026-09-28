"""Shared Phase 0/1 entry points. Default execution is fully offline."""
import argparse
import json
import logging
from pathlib import Path
import queue
import sys
import threading
import time
import numpy as np
from .utils.config import load_config
from .utils.logging import ResponseRecorder
from .sdk.arm_sdk_client import ArmSdkClient
from .control.arm_controller import ArmController
from .waypoint import JointWaypoint, WaypointStore


def _input_worker(inbox):
    try:
        for line in sys.stdin:
            inbox.put(line.strip())
    finally:
        inbox.put('quit')


def main(mode, argv=None):
    parser = argparse.ArgumentParser(description=f'{mode}: default is simulation')
    parser.add_argument('--real', action='store_true')
    parser.add_argument('--execute', action='store_true', help='explicit consent for hardware commands')
    parser.add_argument('--interface')
    parser.add_argument('--config-dir', default='configs')
    parser.add_argument('--exclusive-control-confirmed', action='store_true',
                        help='operator confirms Arm Action and ALL other streaming controllers are stopped')
    parser.add_argument('--duration', type=float, default=None)
    parser.add_argument('--log', help='new JSONL output path (must not exist)')
    parser.add_argument('--dump-first-command', action='store_true', help='inspect final LowCmd/CDR without a publisher')
    if mode == 'acquire_hold':
        parser.add_argument('--official-style', action='store_true', help='legacy experimental arm-only path')
        parser.add_argument('--experimental-arm-only', action='store_true')
        parser.add_argument('--golden', action='store_true', help='Golden upper-body hold (default)')
        parser.add_argument('--hold-time', type=float)
        parser.add_argument('--acquire-hold-only', action='store_true')
        parser.add_argument('--weight', type=float)
    if mode == 'demo':
        parser.add_argument('--waypoint-a', required=True)
        parser.add_argument('--waypoint-b', required=True)
    if mode == 'joint_hold':
        parser.add_argument('--interactive', action='store_true', help='hold while accepting waypoint paths, capture PATH, or quit')
    args = parser.parse_args(argv)
    if mode == 'acquire_hold':
        if args.hold_time is not None:
            args.duration = args.hold_time
        experimental = args.experimental_arm_only or args.official_style
        if args.golden and experimental:
            parser.error('--golden conflicts with experimental arm-only')
        if args.weight is not None and not experimental:
            parser.error('Golden weight curve is frozen; --weight is experimental only')
        if args.real and not args.interface:
            parser.error('--real requires an explicit --interface')
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration < 0):
        parser.error('--duration must be finite and non-negative')
    if args.dump_first_command and not args.real:
        parser.error('--dump-first-command requires --real for actual SDK and lowstate')
    if args.real and mode == 'acquire_hold' and experimental and args.execute and not args.dump_first_command and not (args.interface and args.exclusive_control_confirmed and args.acquire_hold_only):
        parser.error('real HOLD requires --interface, --exclusive-control-confirmed and --acquire-hold-only')
    logging.basicConfig(level=logging.INFO)
    recorder = None
    try:
        config = load_config(args.config_dir)
        if args.duration is None:
            args.duration = config['phase0_hold']['duration_s'] if mode == 'acquire_hold' else 5.0
        if mode == 'acquire_hold' and not experimental:
            from .golden_cli import run_golden
            return run_golden(args, config)
        if mode == 'acquire_hold':
            print('MODE = EXPERIMENTAL_ARM_ONLY_VARIANT')
            print('ACQUIRE_STRATEGY = FULL_WEIGHT_CURRENT_POSE')
            print('PHASE0_KP_15_28 =', config['phase0_hold']['kp'])
            print('PHASE0_KD_15_28 =', config['phase0_hold']['kd'])
        # Validate input files before any DDS initialization.
        points = [WaypointStore.load(args.waypoint_a), WaypointStore.load(args.waypoint_b)] if mode == 'demo' else []
        with ArmSdkClient(config, real=args.real, network_interface=args.interface,
                          read_only=args.dump_first_command or mode in ('probe', 'audit') or (args.real and not args.execute),
                          current_pose_only=mode == 'acquire_hold',
                          weight=args.weight if mode == 'acquire_hold' else None,
                          exclusive_control_confirmed=args.exclusive_control_confirmed) as client:
            if args.dump_first_command or (args.real and not args.execute and mode not in ('probe', 'audit')):
                from .utils.hardware import active_waist
                state = client.wait_stable_state() if mode == 'acquire_hold' else client.get_joint_state()
                command = client._command(state.q)
                waist = state.waist_q if active_waist(config) else None
                data = client.transport.dump_command(command, waist, 1.0 if mode == 'acquire_hold' else 0.0, state.q, official_style=mode == 'acquire_hold')
                data['OWNERSHIP_WEIGHT'] = client.weight_target
                path = Path(args.log or f'logs/first_command_{time.time_ns()}.json')
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open('x', encoding='utf-8') as stream:
                    json.dump(data, stream, indent=2, allow_nan=False)
                print(json.dumps(data, indent=2, allow_nan=False))
                return 0 if data['WIRE_GUARD'] == 'PASS' and data.get('FIRST_WRITE_GUARD', 'PASS') == 'PASS' else 1
            if mode == 'audit':
                from .hardware_audit import run_audit
                run_audit(client, args.duration, args.log or f'logs/hardware_audit_{time.time_ns()}.jsonl')
                return 0
            if mode == 'probe':
                state = client.get_joint_state()
                print(json.dumps(dict(BACKEND='hardware' if args.real else 'simulation',
                      ARM_SDK_INITIALIZED=True if args.real else 'NO (simulation)',
                      ARM_SDK_TOPIC=client.topic, ARM_SDK_CONTROL_AVAILABLE='UNKNOWN',
                      ARM_ACTION_CONFLICT_DETECTED='UNKNOWN',
                      CURRENT_ARM_Q=state.q.tolist(), CURRENT_WAIST_Q=state.waist_q.tolist(),
                      WAIST_COMMAND_REQUIRED='UNKNOWN', WAIST_COMMAND_SENT=False,
                      WAIST_STATE_RESPONSE=state.waist_q.tolist(), WAIST_HOLD_BEHAVIOR='UNKNOWN')))
                return 0
            log_path = args.log or str(Path('logs') / f'{mode}_{time.time_ns()}.jsonl')
            if mode == 'acquire_hold':
                from .evaluation.hardware_telemetry import HardwareTelemetry
                recorder = HardwareTelemetry(log_path, client)
                client.telemetry = recorder
                controller = ArmController(client)
            else:
                recorder = ResponseRecorder(log_path, client)
                controller = ArmController(client, recorder)
            controller.acquire()
            if mode == 'demo':
                for point in points:
                    controller.move(point)
            elif mode == 'joint_hold' and args.interactive:
                inbox = queue.Queue()
                threading.Thread(target=_input_worker, args=(inbox,), daemon=True).start()
                print('Input waypoint JSON/YAML path; capture PATH saves measured joints; quit releases.', flush=True)
                while True:
                    # Input never blocks the command heartbeat.
                    controller.hold_current(client.period_s)
                    try:
                        line = inbox.get_nowait()
                    except queue.Empty:
                        continue
                    if line in ('quit', 'exit'):
                        break
                    if line.startswith('capture '):
                        path = line[len('capture '):].strip()
                        state = client.get_joint_state()
                        WaypointStore.save(path, JointWaypoint(Path(path).stem, state.q,
                                                              waist_q_reference=client.acquire_waist_q))
                    elif line:
                        controller.move(WaypointStore.load(line))
            else:
                controller.hold_current(args.duration)
            controller.release()
            print(f'RESPONSE_LOG = {log_path}')
        return 0
    except KeyboardInterrupt:
        logging.error('ABORT: Ctrl+C; publisher shut down')
        return 130
    except Exception as exc:
        logging.error('ABORT: %s', exc)
        return 1
    finally:
        if recorder is not None:
            recorder.close()
