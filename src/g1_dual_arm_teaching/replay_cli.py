"""Replay 1.1 entry. Defaults to validation only; no DDS until permission passes."""
import argparse
import json
import queue
import signal
import threading
from pathlib import Path
from .trajectory.replay_permission import ReplayPermission
from .trajectory.replay_c2 import C2ReplayPlan
from .sdk.replay_controller import ReplayController
from .sdk.transport import UnitreeTransport
from .golden_cli import BufferedDiagnostics
from .utils.config import load_config
from .utils.naming import timestamped_path


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory',type=Path)
    parser.add_argument('--permission',type=Path,required=True)
    parser.add_argument('--interface')
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args(argv)
    permission=ReplayPermission(args.permission,args.trajectory,execute=args.execute)
    plan=C2ReplayPlan.load(args.trajectory,permission.limits,permission.speed,jerk_limit=permission.jerk,
                           jerk_jump_limit=permission.jerk_jump)
    permission.check_plan(plan)
    config=load_config(Path(__file__).resolve().parents[2]/'configs')
    robot,waist=config['robot'],config['waist']
    if robot['domain_id']!=0 or not all(robot[k] for k in ('hardware_reviewed','model_confirmed','arm_joint_mapping_verified')) or not waist['configuration_verified'] or waist['active_joints']!=[12,13,14]:
        raise ValueError('Golden hardware/mapping prerequisites not met')
    if not args.execute:
        print('REPLAY_VALIDATED_ONLY: hardware execute OFF; no DDS initialized')
        return 0
    if not args.interface:raise ValueError('explicit network interface required')
    path=timestamped_path('logs','replay_1_1','.jsonl');path.parent.mkdir(parents=True,exist_ok=True)
    transport=UnitreeTransport();controller=None;buffer=BufferedDiagnostics();previous={}
    requests=queue.Queue()
    def handler(signum,frame):
        if controller is not None and not controller.replay_active:controller.stop()
        else:requests.put('q')
    def input_worker():
        import sys
        for line in sys.stdin:
            if line.strip().lower() in ('q','quit'):requests.put('q')
    try:
        transport.initialize(config,args.interface,True)
        controller=ReplayController(transport,config,permission,emit=buffer.emit)
        for sig in (signal.SIGINT,signal.SIGTERM):previous[sig]=signal.signal(sig,handler)
        threading.Thread(target=input_worker,daemon=True).start()
        buffer.emit(dict(event='REPLAY_PERMISSION',manifest=permission.manifest,trajectory=str(args.trajectory)))
        print('REPLAY: Q + Enter or Ctrl+C requests controlled stop; repeated request hard-aborts per approved policy',flush=True)
        controller.acquire(plan)
        while controller.replay_active:
            while not requests.empty():
                requests.get_nowait()
                if controller.stop_started is None:controller.request_stop(permission.stop)
                else:controller.stop()
            controller.tick()
            if controller.replay_active:controller._profiled_sleep(permission.period)
        return 0 if controller.released and not controller.abort_reason else 2
    except (Exception,KeyboardInterrupt) as exc:
        buffer.emit(dict(event='REPLAY_ENTRY_FAILED',error=str(exc)))
        if controller is not None:
            controller.abort_reason=controller.abort_reason or str(exc)
        print(f'REPLAY_ABORTED = {exc}',flush=True)
        return 2
    finally:
        try:
            if controller is not None:
                try:controller.close()
                finally:controller.report_diagnostics()
        finally:
            try:transport.shutdown()
            finally:
                for sig,old in previous.items():signal.signal(sig,old)
                with path.open('x',encoding='utf-8') as stream:buffer.finish(stream)
                print(f'REPLAY_LOG = {path}',flush=True)


if __name__=='__main__':raise SystemExit(main())
