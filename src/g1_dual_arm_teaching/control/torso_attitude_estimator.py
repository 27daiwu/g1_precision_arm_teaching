"""Offline-capable G1 torso attitude from LowState root IMU and waist joints."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TorsoAttitude:
    pelvis_quaternion_wxyz: np.ndarray
    torso_quaternion_wxyz: np.ndarray
    torso_roll: float
    torso_pitch: float
    torso_yaw: float
    rotation_matrix: np.ndarray


def _product(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array((w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v), dtype=float)


def _axis_quaternion(angle, axis):
    q = np.zeros(4, dtype=float)
    q[0] = np.cos(angle / 2)
    q[axis + 1] = np.sin(angle / 2)
    return q


def _rotation_matrix(q):
    w, x, y, z = q
    return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                     (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                     (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))


class TorsoAttitudeEstimator:
    def __init__(self):
        self._previous_pelvis = None
        self._previous_torso = None

    def estimate(self, pelvis_quaternion_wxyz, q12, q13, q14):
        pelvis = np.asarray(pelvis_quaternion_wxyz, dtype=float)
        joints = np.asarray((q12, q13, q14), dtype=float)
        if pelvis.shape != (4,) or not np.isfinite(pelvis).all():
            raise ValueError('pelvis quaternion must be four finite WXYZ values')
        if not np.isfinite(joints).all():
            raise ValueError('waist joint positions must be finite')
        norm = np.linalg.norm(pelvis)
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError('pelvis quaternion norm is invalid')
        pelvis = pelvis / norm
        if self._previous_pelvis is not None and np.dot(pelvis, self._previous_pelvis) < 0:
            pelvis = -pelvis
        torso = _product(_product(_product(pelvis, _axis_quaternion(q12, 2)),
                                  _axis_quaternion(q13, 0)), _axis_quaternion(q14, 1))
        torso /= np.linalg.norm(torso)
        if self._previous_torso is not None and np.dot(torso, self._previous_torso) < 0:
            torso = -torso
        matrix = _rotation_matrix(torso)
        roll = np.arctan2(matrix[2, 1], matrix[2, 2])
        pitch = np.arcsin(np.clip(-matrix[2, 0], -1.0, 1.0))
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
        self._previous_pelvis = pelvis.copy()
        self._previous_torso = torso.copy()
        return TorsoAttitude(pelvis, torso, float(roll), float(pitch),
                             float(yaw), matrix)
