"""Optional in-memory observer for the original tester; no control decisions."""
import json
import time
from .cycle_profile import Motor14Diagnostics
from .golden_diagnostics import GoldenDiagnostics


class ReferenceDiagnostics:
    def __init__(self, tester, clock=time.monotonic):
        self.clock = clock
        self.phase = 'WAIT'
        self.motor14 = Motor14Diagnostics()
        self.writes = []
        self.capture_time = None
        self.capture_pose = None
        self.parameters = dict(kp=tester.kp, kd=tester.kd, dt=tester.dt)
        capture = tester._capture_pose
        def capture_observed():
            pose = capture()
            self.capture_time = clock()
            self.capture_pose = pose.copy()
            self.motor14.capture(pose[14])
            return pose
        tester._capture_pose = capture_observed
        for name, phase in (('acquire','ACQUIRE'),('release','RELEASE')):
            original = getattr(tester, name)
            def observed(*args, _original=original, _phase=phase, **kwargs):
                self.phase = _phase
                return _original(*args, **kwargs)
            setattr(tester, name, observed)
        write = tester.publisher.Write
        def write_observed(message):
            state = tester.low_state
            if state is not None:
                self.motor14.observe(self.phase, state.motor_state[14].q,
                                     state.motor_state[14].dq, clock())
            row = dict(event='DDS_WRITE_RESULT', sequence=len(self.writes), phase=self.phase,
                       real_dds=True, weight=float(message.motor_cmd[29].q))
            start = clock()
            try:
                result = write(message)
            except BaseException as exc:
                end = clock()
                row.update(outcome='EXCEPTION', error=str(exc))
                raise
            else:
                end = clock()
                row['outcome'] = 'FAILED' if result is False else 'SUCCESS'
                return result  # Preserve original publisher result/exception semantics.
            finally:
                row.update(write_start=start, write_end=end, write_duration_s=end-start)
                self.writes.append(row)
        tester.publisher.Write = write_observed
        tester.diagnostics = self

    def finish(self, stream):
        rows = [w for w in self.writes if w['outcome']=='SUCCESS']
        summary = self.motor14.summary()
        summary.update(source='ORIGINAL_TEST_ARMSDK', capture_time=self.capture_time,
                       parameters=self.parameters, captured_upper_body_pose=self.capture_pose,
                       DDS_COMMAND_TIMING=GoldenDiagnostics._frequency(rows),
                       DDS_COMMAND_TIMING_BY_PHASE={phase:GoldenDiagnostics._frequency([w for w in rows if w['phase']==phase])
                                                    for phase in ('ACQUIRE','HOLD','RELEASE')})
        for row in [*self.writes, *[dict(s,event='MOTOR14_SAMPLE') for s in self.motor14.samples], summary]:
            stream.write(json.dumps(row, allow_nan=False)+'\n')
        stream.flush()
        print(json.dumps(summary, allow_nan=False))
