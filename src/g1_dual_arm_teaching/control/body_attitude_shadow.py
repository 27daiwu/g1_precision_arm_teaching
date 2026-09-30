"""Observation-only P candidate for torso roll and pitch."""

from dataclasses import dataclass

import numpy as np

from .torso_attitude_estimator import TorsoAttitudeEstimator


@dataclass(frozen=True)
class ShadowSample:
    torso_roll: float
    torso_pitch: float
    roll_ref: float
    pitch_ref: float
    roll_error: float
    pitch_error: float
    motor13_candidate: float
    motor14_candidate: float
    q12: float
    q13: float
    q14: float


class BodyAttitudeShadow:
    def __init__(self, kp_roll=0.5, kp_pitch=0.5, clamp_rad=0.05,
                 rate_limit_rad_s=0.1):
        values = np.asarray((kp_roll, kp_pitch, clamp_rad, rate_limit_rad_s), dtype=float)
        if not np.isfinite(values).all() or np.any(values[:2] < 0) or np.any(values[2:] <= 0):
            raise ValueError('invalid shadow gains or limits')
        self.kp = values[:2]
        self.clamp = float(clamp_rad)
        self.rate_limit = float(rate_limit_rad_s)
        self.estimator = TorsoAttitudeEstimator()
        self.reference = None
        self.previous = np.zeros(2)
        self.previous_time = None

    def update(self, state):
        if state.imu_quaternion_wxyz is None or state.waist_q is None:
            raise ValueError('shadow requires synchronous LowState IMU and waist state')
        if not np.isfinite(state.timestamp):
            raise ValueError('invalid shadow state timestamp')
        attitude = self.estimator.estimate(state.imu_quaternion_wxyz, *state.waist_q)
        measured = np.array((attitude.torso_roll, attitude.torso_pitch))
        if self.reference is None:
            self.reference = measured.copy()
            self.previous_time = state.timestamp
        dt = state.timestamp - self.previous_time
        if dt < 0:
            raise ValueError('shadow state timestamp moved backwards')
        error = (self.reference - measured + np.pi) % (2 * np.pi) - np.pi
        target = np.clip(self.kp * error, -self.clamp, self.clamp)
        candidate = self.previous + np.clip(target - self.previous,
                                            -self.rate_limit * dt, self.rate_limit * dt)
        self.previous = candidate
        self.previous_time = state.timestamp
        return ShadowSample(float(measured[0]), float(measured[1]),
                            float(self.reference[0]), float(self.reference[1]),
                            float(error[0]), float(error[1]),
                            float(candidate[0]), float(candidate[1]),
                            *(float(value) for value in state.waist_q))
