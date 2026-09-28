"""SDK-independent waypoint progression and measured-response logging."""
import time
import numpy as np
from ..trajectory.trajectory_base import SmoothstepTrajectory
from ..utils.timing import ControlLoop


class ArmController:
    def __init__(self, client, recorder=None):
        self.client, self.recorder = client, recorder
        self.acquired_q = None

    def acquire(self):
        state = self.client.acquire()
        self.acquired_q = state.q.copy()
        return state

    def release(self):
        self.client.release()

    def _record(self, q, phase):
        if self.recorder is not None:
            self.recorder.record(self.client.get_joint_state(), q, phase)

    def hold_current(self, duration_s):
        if not np.isfinite(duration_s) or duration_s < 0:
            raise ValueError('invalid hold duration')
        loop = ControlLoop(self.client.config['control']['frequency_hz'])
        start = time.monotonic()
        while True:
            self.client.hold()
            self._record(self.client._last.q, 'hold')
            if time.monotonic() - start >= duration_s:
                break
            loop.wait()

    def move(self, waypoint):
        try:
            if self.client.current_pose_only:
                raise RuntimeError('CURRENT_POSE_ONLY forbids waypoint')
            if self.client.real:
                from ..utils.hardware import require_real_acquire
                require_real_acquire(self.client.config, False)
            if waypoint.waist_q_reference is not None and not np.array_equal(waypoint.waist_q_reference, self.client.acquire_waist_q):
                raise ValueError('waypoint cannot modify acquired waist reference')
            state = self.client.get_joint_state()
            self.client.guard.validate(waypoint.q_arm)
            # Each new segment starts at its freshly measured pose. A tracking gap
            # exceeding a safe step is rejected by the publishing guard.
            delta = float(np.max(np.abs(waypoint.q_arm - state.q)))
            speed = min(self.client.guard.max_velocity_rad_s,
                        self.client.guard.max_joint_step_rad / self.client.period_s)
            minimum_duration = 1.5 * delta / speed
            duration = waypoint.duration
            if duration is None:
                duration = max(self.client.config['trajectory']['default_duration_s'], minimum_duration * 1.01)
            if duration < minimum_duration:
                raise ValueError('duration exceeds smoothstep velocity/step limits')
            trajectory = SmoothstepTrajectory(state.q, waypoint.q_arm, duration)
            loop = ControlLoop(self.client.config['control']['frequency_hz'])
            # Explicit zero sample: first target is bit-for-bit measured q.
            t = 0.0
            started = time.monotonic()
            while True:
                sample = trajectory.sample(t)
                self.client.send_joint_command(self.client._command(sample.q, sample.dq))
                self._record(sample.q, waypoint.name)
                if t >= duration:
                    break
                loop.wait()
                t = min(time.monotonic() - started, duration)
            self.hold_current(waypoint.hold_time)
        except BaseException:
            self.client.abort('waypoint execution failed')
            raise
