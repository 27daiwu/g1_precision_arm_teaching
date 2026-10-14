"""Single-run Absolute/Relative Replay entry."""
import argparse
import queue
import signal
import threading
from pathlib import Path
import numpy as np
from .trajectory.replay_permission import ReplayPermission
from .trajectory.replay_c2 import C2ReplayPlan, RelativeReplaySpec
from .sdk.replay_controller import ReplayController
from .sdk.transport import UnitreeTransport
from .golden_cli import BufferedDiagnostics
from .utils.config import load_config
from .utils.naming import timestamped_path


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('--mode', choices=('absolute', 'relative'), default=None)
    parser.add_argument('--speed', type=float, default=None, help='playback speed multiplier')
    parser.add_argument('--start-index', type=int, default=None, help='inclusive trajectory sample index')
    parser.add_argument('--end-index', type=int, default=None, help='exclusive trajectory sample index')
    parser.add_argument('--motion-scale', type=float, default=None, help='relative displacement multiplier')
    parser.add_argument('--interface', default=None, help='DDS network interface; uses configured interface when omitted')
    parser.add_argument('--play', action='store_true', help='perform one playback (hardware execution remains fail-closed)')
    args=parser.parse_args(argv)
    config_dir = Path(__file__).resolve().parents[2] / 'configs'
    if args.play:
        import yaml
        runtime = yaml.safe_load((config_dir / 'replay.yaml').read_text(encoding='utf-8'))
        if runtime.get('hardware_execute_enabled') is not True:
            raise ValueError('hardware playback disabled in configs/replay.yaml')
    permission=ReplayPermission(config_dir / 'replay.yaml', execute=args.play)
    replay_mode = args.mode or permission.replay_mode
    permission.replay_mode = replay_mode
    speed = permission.speed if args.speed is None else args.speed
    if not np.isfinite(speed) or speed <= 0:
        raise ValueError('speed multiplier must be finite and positive')
    if replay_mode=='relative':
        candidate = permission.manifest.get('candidate', {})
        motion_scale = (candidate['motion_scale']
                        if args.motion_scale is None else args.motion_scale)
        start_index = (candidate['start_index']
                       if args.start_index is None else args.start_index)
        end_index = (candidate['end_index']
                     if args.end_index is None else args.end_index)
        candidate.update(mode='relative', motion_scale=motion_scale,
                         start_index=start_index, end_index=end_index)
        plan=RelativeReplaySpec.load(args.trajectory,permission.limits,speed,
            jerk_limit=permission.jerk,jerk_jump_limit=permission.jerk_jump,
            motion_scale=motion_scale,
            start_index=start_index,
            end_index=end_index)
    else:
        plan=C2ReplayPlan.load(args.trajectory,permission.limits,speed,jerk_limit=permission.jerk,
                               jerk_jump_limit=permission.jerk_jump)
    permission.check_plan(plan)
    config=load_config(config_dir)
    from .utils.hardware import require_teaching_topology
    require_teaching_topology(config)
    if not args.play:
        if replay_mode=='relative':
            print('RELATIVE_REPLAY_SPEC_VALIDATED_ONLY: acquire-bound trajectory checks require a fresh LowState; hardware execute OFF; no DDS initialized')
        else:
            print('REPLAY_VALIDATED_ONLY: hardware execute OFF; no DDS initialized')
        return 0
    from .utils.hardware import resolve_network_interface
    interface=resolve_network_interface(config, args.interface)
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
        transport.initialize(config,interface,True)
        controller=ReplayController(transport,config,permission,emit=buffer.emit)
        for sig in (signal.SIGINT,signal.SIGTERM):previous[sig]=signal.signal(sig,handler)
        threading.Thread(target=input_worker,daemon=True).start()
        buffer.emit(dict(event='REPLAY_CONFIG',mode=replay_mode,speed_scale=speed,
                         motion_scale=getattr(plan, 'motion_scale', 1.0),
                         start_index=getattr(plan, 'start_index', 0),
                         end_index=getattr(plan, 'end_index', len(plan.q)),
                         trajectory=str(args.trajectory)))
        print('REPLAY: Q + Enter or Ctrl+C requests controlled stop; repeated request hard-aborts',flush=True)
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
