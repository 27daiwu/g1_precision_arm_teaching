"""Offline waypoint sequencing and trajectory construction."""
import numpy as np
from ..trajectory.trajectory_base import JointTrajectory, SmoothstepTrajectory
from .waypoint import JointWaypoint


class WaypointPlayer:
    def __init__(self, waypoints: list[JointWaypoint], measured_q: np.ndarray, default_duration: float = 2.0):
        if not waypoints:
            raise ValueError("at least one waypoint is required")
        current = np.asarray(measured_q, dtype=float)
        if current.ndim != 1 or not np.all(np.isfinite(current)):
            raise ValueError("measured_q must be a finite vector")
        if not np.isfinite(default_duration) or default_duration <= 0:
            raise ValueError("default_duration must be finite and positive")
        self._segments: list[tuple[JointTrajectory, float, float]] = []
        elapsed = 0.0
        for waypoint in waypoints:
            if waypoint.q_arm.shape != current.shape:
                raise ValueError("waypoint and measured_q dimensions must match")
            duration = float(waypoint.duration if waypoint.duration is not None else default_duration)
            trajectory = SmoothstepTrajectory(current, waypoint.q_arm, duration)
            self._segments.append((trajectory, elapsed, elapsed + duration))
            elapsed += duration + waypoint.hold_time
            current = waypoint.q_arm.copy()
        self.duration = elapsed

    def sample(self, t: float):
        if not np.isfinite(t):
            raise ValueError("sample time must be finite")
        t = float(np.clip(t, 0.0, self.duration))
        for index, (trajectory, start, end) in enumerate(self._segments):
            next_start = self._segments[index + 1][1] if index + 1 < len(self._segments) else self.duration
            if t <= next_start:
                return trajectory.sample(t - start)
        return self._segments[-1][0].sample(self._segments[-1][0].duration)
