"""Joint-space waypoint data and validation."""
from dataclasses import dataclass
from typing import Any, Mapping
import numpy as np
from ..sdk.arm_sdk_types import vector


@dataclass(frozen=True)
class JointWaypoint:
    name: str
    q_arm: np.ndarray
    duration: float | None = None
    hold_time: float = 0.0
    waist_q_reference: np.ndarray | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("waypoint name must be a non-empty string")
        q = np.asarray(self.q_arm, dtype=float)
        if q.shape != (14,):
            raise ValueError("q_arm must contain exactly 14 arm joints")
        if not np.all(np.isfinite(q)):
            raise ValueError("q_arm must contain only finite values")
        object.__setattr__(self, "q_arm", q.copy())
        if self.duration is not None and (not np.isfinite(self.duration) or self.duration <= 0):
            raise ValueError("duration must be finite and positive")
        if not np.isfinite(self.hold_time) or self.hold_time < 0:
            raise ValueError("hold_time must be finite and non-negative")
        if self.waist_q_reference is not None:
            waist = np.asarray(self.waist_q_reference, dtype=float)
            if waist.shape != (3,) or not np.all(np.isfinite(waist)):
                raise ValueError("waist_q_reference must contain three finite joints")
            object.__setattr__(self, "waist_q_reference", waist.copy())

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "name": self.name,
            "q_arm": self.q_arm.tolist(),
            "motion": {"duration": self.duration, "hold_time": self.hold_time},
            "waist": {"mode": "HOLD_AT_ACQUIRE_POSE"},
        }
        if self.waist_q_reference is not None:
            result["waist"]["q_reference"] = self.waist_q_reference.tolist()
        return result

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "JointWaypoint":
        if not isinstance(data, Mapping):
            raise ValueError("waypoint must be a mapping")
        motion = data.get("motion") or {}
        waist = data.get("waist") or {}
        if not isinstance(motion, Mapping) or not isinstance(waist, Mapping):
            raise ValueError("motion and waist must be mappings")
        if waist.get("mode", "HOLD_AT_ACQUIRE_POSE") != "HOLD_AT_ACQUIRE_POSE":
            raise ValueError("only HOLD_AT_ACQUIRE_POSE is supported")
        if any(key in data for key in ("cartesian", "pose", "position", "orientation")):
            raise ValueError("Cartesian waypoints are not supported")
        q = data.get("q_arm")
        if q is None:
            left, right = data.get("left_arm"), data.get("right_arm")
            if left is not None and right is not None:
                left_q = vector(left.get("q") if isinstance(left, Mapping) else left, 7)
                right_q = vector(right.get("q") if isinstance(right, Mapping) else right, 7)
                q = np.concatenate((left_q, right_q))
        if q is None:
            raise ValueError("waypoint is missing q_arm")
        return cls(data.get("name", ""), q, motion.get("duration", data.get("duration")), motion.get("hold_time", data.get("hold_time", 0.0)), waist.get("q_reference"))
