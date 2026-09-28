"""Validated state compatibility view; timestamps use the monotonic clock."""
from dataclasses import dataclass, field
import time
import numpy as np
from ..sdk.arm_sdk_types import JointState


@dataclass
class RobotState:
    q_arm: np.ndarray
    waist_q: np.ndarray | None = None
    timestamp: float = field(default_factory=time.monotonic)

    def __post_init__(self):
        state = JointState(self.q_arm, self.timestamp, waist_q=self.waist_q)
        self.q_arm, self.waist_q = state.q, state.waist_q
