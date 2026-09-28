from dataclasses import dataclass
import numpy as np

def vector(value, size=None):
    result = np.asarray(value, dtype=float)
    if result.ndim != 1 or (size is not None and result.size != size):
        raise ValueError('invalid joint vector dimensionality')
    if not np.all(np.isfinite(result)):
        raise ValueError('joint vector contains NaN/Inf')
    return result.copy()

@dataclass
class ArmJointCommand:
    q: np.ndarray
    dq: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    tau_ff: np.ndarray

    def __post_init__(self):
        self.q = vector(self.q, 14)
        for field in ('dq', 'kp', 'kd', 'tau_ff'):
            setattr(self, field, vector(getattr(self, field), 14))
        if np.any(self.tau_ff != 0):
            raise ValueError('tau_ff must be exactly zero')
        if np.any(self.kp < 0) or np.any(self.kd < 0):
            raise ValueError('gains must be non-negative')

@dataclass
class JointState:
    q: np.ndarray
    timestamp: float
    dq: np.ndarray | None = None
    waist_q: np.ndarray | None = None
    waist_dq: np.ndarray | None = None

    def __post_init__(self):
        self.q = vector(self.q, 14)
        if self.dq is not None:
            self.dq = vector(self.dq, 14)
        if self.waist_q is not None:
            self.waist_q = vector(self.waist_q, 3)
        if self.waist_dq is not None:
            self.waist_dq = vector(self.waist_dq, 3)
        if not np.isfinite(self.timestamp):
            raise ValueError('invalid state timestamp')
