from dataclasses import dataclass
import numpy as np
from ..sdk.arm_sdk_types import vector

@dataclass
class JointTrajectorySample:
    q: np.ndarray
    dq: np.ndarray

class JointTrajectory:
    def __init__(self, q0, q1, duration):
        self.q0 = vector(q0, 14)
        self.q1 = vector(q1, 14)
        self.duration = float(duration)
        if not np.isfinite(self.duration) or self.duration <= 0:
            raise ValueError('trajectory duration must be finite and positive')

    def sample(self, t):
        if not np.isfinite(t):
            raise ValueError('sample time must be finite')
        if t <= 0:
            return JointTrajectorySample(self.q0.copy(), np.zeros_like(self.q0))
        if t >= self.duration:
            return JointTrajectorySample(self.q1.copy(), np.zeros_like(self.q1))
        u = t / self.duration
        return JointTrajectorySample(self.q0 + (self.q1-self.q0)*u*u*(3-2*u), (self.q1-self.q0)*6*u*(1-u)/self.duration)

SmoothstepTrajectory = JointTrajectory
