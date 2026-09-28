"""Phase 0 Golden entry; hardware publication requires --real --execute."""
import json
from pathlib import Path
import signal
import time
from types import SimpleNamespace
from .sdk.transport import UnitreeTransport, SimulationTransport
from .sdk.golden_controller import ArmSdkGoldenController


class BufferedDiagnostics:
    """Ordinary samples are serialized only after release; hard events and first
    joint/reason warnings retain their immediate logging in GoldenDiagnostics.
    """
    def __init__(self):
        self.rows = []

    def emit(self, data):
        self.rows.append(data)

    def finish(self, stream):
        printed_events = {'GOLDEN_DIAGNOSTICS_SUMMARY', 'MOTOR14_HOLD_COMPARISON',
                          'CONTROL_LOOP_PROFILE_SUMMARY', 'WIRE_GUARD_PROFILE',
                          'POST_RUN_FULL_WIRE_AUDIT'}
        for data in self.rows:
            line = json.dumps(data, allow_nan=False)
            stream.write(line + '\n')
            if data.get('sequence') == 0 or data.get('event') in printed_events:
                print(line)
        stream.flush()


def run_golden(args, config):
    if config['robot']['domain_id'] != 0:
        raise ValueError('Golden DDS domain must be 0, as in the reference')
    real_write = args.real and args.execute and not args.dump_first_command
    if real_write:
        robot, waist = config['robot'], config['waist']
        if not all(robot[key] for key in ('hardware_reviewed', 'model_confirmed', 'arm_joint_mapping_verified')):
            raise ValueError('Golden requires reviewed hardware and motor mapping')
        if not waist['configuration_verified'] or waist['active_joints'] != [12, 13, 14]:
            raise ValueError('Golden requires verified active waist motors 12..14')
    print(f'MODE = GOLDEN_UPPER_BODY_CURRENT_POSE_HOLD\nDDS_INTERFACE = {args.interface}\n'
          'CONTROLLED_MOTORS = 12..28\nKP = 40\nKD = 1.5\nCOMMAND_FREQUENCY = 50 Hz\n'
          f'ACQUIRE_RAMP = 2.0 s\nFULL_WEIGHT_HOLD = {args.duration} s\nRELEASE_RAMP = 2.0 s\n'
          f'TAU_FF = 0\nMOTOR29_WEIGHT_ONLY = YES\nREAL_ACTUATION = {"YES" if real_write else "NO"}')
    path = Path(args.log or f'logs/golden_hold_{time.time_ns()}.jsonl')
    path.parent.mkdir(parents=True, exist_ok=True)
    transport = UnitreeTransport() if args.real else SimulationTransport()
    previous = {}
    controller = None
    with path.open('x', encoding='utf-8') as stream:
        buffered = BufferedDiagnostics()
        emit = buffered.emit
        try:
            transport.initialize(config, args.interface, real_write)
            if not args.real:
                # Offline wire-shaped objects; this CRC is explicitly a simulation placeholder.
                transport._message = lambda: SimpleNamespace(mode_pr=0, mode_machine=0, reserve=[0]*4,
                    motor_cmd=[SimpleNamespace(q=0., dq=0., kp=0., kd=0., tau=0., mode=0, reserve=0) for _ in range(35)])
                transport._crc = SimpleNamespace(Crc=lambda message: 0)
                transport.publisher = SimpleNamespace(Write=lambda message: True)
                print('BACKEND = SIMULATION; CRC = PLACEHOLDER')
            controller = ArmSdkGoldenController(transport, config, emit=emit, real_dds=real_write)
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous[sig] = signal.signal(sig, lambda signum, frame: controller.stop())
            if (args.real and not real_write) or args.dump_first_command:
                controller.dump_first_command()
            else:
                controller.run(args.duration)
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
