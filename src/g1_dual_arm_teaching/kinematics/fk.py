"""Offline G1 29DoF arm forward kinematics.

The reported endpoint is the URDF/MJCF wrist-yaw link frame origin. It is an
uncalibrated tool-frame proxy, not a measured gripper/tool-center point.
"""

from dataclasses import dataclass
import hashlib
from pathlib import Path
import importlib
import tempfile
import xml.etree.ElementTree as ET

import numpy as np


DEFAULT_MJCF = Path(
    "/home/hebe/unitree_workspace/unitree_rl_mjlab/src/assets/robots/"
    "unitree_g1/xmls/g1.xml"
)
DEFAULT_URDF = Path(
    "/home/hebe/unitree_workspace/unitree_ros/robots/g1_description/"
    "g1_29dof_with_hand.urdf"
)
ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
WAIST_JOINTS = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")
TCP_FRAMES = {"left": "left_wrist_yaw_link", "right": "right_wrist_yaw_link"}


def _vector(value, shape, label):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{label} must have shape {shape} and finite values")
    return result


@dataclass(frozen=True)
class Pose:
    frame: str
    position_m: np.ndarray
    rotation: np.ndarray
    quaternion_wxyz: np.ndarray


@dataclass(frozen=True)
class ShadowComparison:
    measured: dict
    reference: dict
    position_error_m: dict
    orientation_error_rad: dict
    provenance: str = "geometric FK comparison only; not measured tracking performance"


class G1ArmFK:
    """FK of wrist-yaw frame origins in pelvis- or torso-relative coordinates.

    MuJoCo uses the project G1 29DoF MJCF. Pinocchio is supported when installed
    and is selected with backend="pinocchio". q_arm is motor15..28 order and
    q_waist is motor12..14 order. Pelvis orientation is fixed to identity;
    recorded data does not contain enough root pose information for world FK.
    """

    def __init__(self, backend="ikpy", model_path=None, tcp_offset=None):
        if backend not in ("ikpy", "mujoco", "pinocchio"):
            raise ValueError("backend must be 'ikpy', 'mujoco' or 'pinocchio'")
        self.backend = backend
        self.model_path = Path(model_path or (DEFAULT_MJCF if backend == "mujoco" else DEFAULT_URDF))
        if not self.model_path.is_file():
            raise FileNotFoundError(self.model_path)
        self.tcp_offset = {}
        for side in ("left", "right"):
            offset = np.zeros(3) if tcp_offset is None else tcp_offset.get(side, np.zeros(3))
            self.tcp_offset[side] = _vector(offset, (3,), f"{side} TCP offset").copy()
        if backend == "mujoco":
            self._load_mujoco()
        elif backend == "ikpy":
            self._load_ikpy()
        else:
            self._load_pinocchio()

    @staticmethod
    def _arm_path(side, include_waist):
        arm_names = ARM_JOINTS[:7] if side == "left" else ARM_JOINTS[7:]
        path = ["pelvis"]
        if include_waist:
            path += ["waist_yaw_joint", "waist_yaw_link", "waist_roll_joint",
                     "waist_roll_link", "waist_pitch_joint", "torso_link"]
        else:
            path = ["torso_link"]
        links = [name.removesuffix("_joint") + "_link" for name in arm_names]
        for joint, link in zip(arm_names, links):
            path.extend((joint, link))
        return path, arm_names

    def _load_ikpy(self):
        try:
            chain_module = importlib.import_module("ikpy.chain")
        except ImportError as exc:
            raise RuntimeError("IKPy is unavailable; install the project's 'fk' extra") from exc
        self._ikpy_chains = {"pelvis": {}, "torso": {}}
        self._ikpy_joint_indices = {"pelvis": {}, "torso": {}}
        tree = ET.parse(self.model_path)
        root = tree.getroot()
        for frame in ("pelvis", "torso"):
            for side in ("left", "right"):
                path, arm_names = self._arm_path(side, frame == "pelvis")
                joints = {path[i] for i in range(1, len(path), 2)}
                chain_root = ET.fromstring(ET.tostring(root, encoding="unicode"))
                for joint in list(chain_root.findall("joint")):
                    if joint.get("name") not in joints:
                        chain_root.remove(joint)
                with tempfile.NamedTemporaryFile(mode="wb", suffix=".urdf") as stream:
                    ET.ElementTree(chain_root).write(stream, encoding="utf-8", xml_declaration=True)
                    stream.flush()
                    chain = chain_module.Chain.from_urdf_file(
                        stream.name, base_elements=path, symbolic=False,
                        active_links_mask=[False] + [True] * len(joints))
                actual = {link.name: i for i, link in enumerate(chain.links)}
                missing = joints - actual.keys()
                if missing:
                    raise ValueError(f"URDF chain missing joints {sorted(missing)}")
                expected = set(WAIST_JOINTS if frame == "pelvis" else ()) | set(arm_names)
                if expected - actual.keys():
                    raise ValueError(f"URDF arm chain mapping incomplete for {side}/{frame}")
                self._ikpy_chains[frame][side] = chain
                self._ikpy_joint_indices[frame][side] = actual
        self.model_hash_sha256 = hashlib.sha256(self.model_path.read_bytes()).hexdigest()

    def _load_mujoco(self):
        try:
            self.mujoco = importlib.import_module("mujoco")
        except ImportError as exc:
            raise RuntimeError("MuJoCo is unavailable; install the project's 'fk' extra") from exc
        self.model = self.mujoco.MjModel.from_xml_path(str(self.model_path))
        self.model_hash_sha256 = hashlib.sha256(self.model_path.read_bytes()).hexdigest()
        self.data = self.mujoco.MjData(self.model)
        self._qpos = {}
        for name in WAIST_JOINTS + ARM_JOINTS:
            joint_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0 or self.model.jnt_type[joint_id] != self.mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"model is missing expected hinge joint {name}")
            self._qpos[name] = int(self.model.jnt_qposadr[joint_id])
        self._body = {}
        for side, name in TCP_FRAMES.items():
            body_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id < 0:
                raise ValueError(f"model is missing frame {name}")
            self._body[side] = body_id
        self._torso_id = self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_BODY, "torso_link")
        if self._torso_id < 0:
            raise ValueError("model is missing torso_link")

    def _load_pinocchio(self):
        try:
            self.pin = importlib.import_module("pinocchio")
        except ImportError as exc:
            raise RuntimeError("Pinocchio is unavailable in this Python; install the 'fk-pinocchio' extra") from exc
        if not hasattr(self.pin, "buildModelFromUrdf"):
            raise RuntimeError("the imported 'pinocchio' module is not Robotics Pinocchio; install package 'pin'")
        try:
            self.model = self.pin.buildModelFromUrdf(str(self.model_path))
        except Exception as exc:
            raise ValueError(f"could not build Pinocchio model from {self.model_path}: {exc}") from exc
        self.model_hash_sha256 = hashlib.sha256(self.model_path.read_bytes()).hexdigest()
        self.data = self.model.createData()
        self._joint_ids = {}
        for name in WAIST_JOINTS + ARM_JOINTS:
            jid = self.model.getJointId(name)
            if jid == 0:
                raise ValueError(f"URDF is missing expected joint {name}")
            joint = self.model.joints[jid]
            if joint.nq != 1:
                raise ValueError(f"expected scalar revolute joint {name}")
            self._joint_ids[name] = jid
        self._frames = {}
        for side, name in TCP_FRAMES.items():
            fid = self.model.getFrameId(name)
            if fid >= len(self.model.frames):
                raise ValueError(f"URDF is missing frame {name}")
            self._frames[side] = fid
        self._torso_frame = self.model.getFrameId("torso_link")

    def _configuration(self, q_arm, q_waist):
        arm = _vector(q_arm, (14,), "q_arm motor15..28")
        waist = _vector(q_waist, (3,), "q_waist motor12..14")
        if self.backend == "mujoco":
            self.data.qpos[:] = 0.0
            self.data.qpos[3] = 1.0  # free-joint quaternion WXYZ
            for name, value in zip(WAIST_JOINTS, waist):
                self.data.qpos[self._qpos[name]] = value
            for name, value in zip(ARM_JOINTS, arm):
                self.data.qpos[self._qpos[name]] = value
        elif self.backend == "pinocchio":
            q = self.pin.neutral(self.model)
            for name, value in zip(WAIST_JOINTS, waist):
                joint = self.model.joints[self._joint_ids[name]]
                q[joint.idx_q] = value
            for name, value in zip(ARM_JOINTS, arm):
                joint = self.model.joints[self._joint_ids[name]]
                q[joint.idx_q] = value
            self.pin.forwardKinematics(self.model, self.data, q)
            self.pin.updateFramePlacements(self.model, self.data)
        if self.backend == "mujoco":
            self.mujoco.mj_forward(self.model, self.data)

    def _ikpy_pose(self, side, q_arm, q_waist, frame):
        chain = self._ikpy_chains[frame][side]
        indices = self._ikpy_joint_indices[frame][side]
        values = np.zeros(len(chain.links), dtype=float)
        names = (WAIST_JOINTS + (ARM_JOINTS[:7] if side == "left" else ARM_JOINTS[7:])
                 if frame == "pelvis" else (ARM_JOINTS[:7] if side == "left" else ARM_JOINTS[7:]))
        vals = tuple(q_waist) + tuple(q_arm[:7] if side == "left" else q_arm[7:]) if frame == "pelvis" else tuple(q_arm[:7] if side == "left" else q_arm[7:])
        for name, value in zip(names, vals):
            values[indices[name]] = value
        transform = chain.forward_kinematics(values)
        position = transform[:3, 3] + transform[:3, :3] @ self.tcp_offset[side]
        frame_name = f"{side}_wrist_yaw_link"
        return self._pose(frame_name if frame == "pelvis" else f"torso_link->{frame_name}",
                          position, transform[:3, :3])

    @staticmethod
    def _pose(frame, position, rotation):
        matrix = np.asarray(rotation, dtype=float)
        trace = float(np.trace(matrix))
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            quat = np.array((0.25 * s, (matrix[2, 1] - matrix[1, 2]) / s,
                             (matrix[0, 2] - matrix[2, 0]) / s,
                             (matrix[1, 0] - matrix[0, 1]) / s))
        else:
            i = int(np.argmax(np.diag(matrix)))
            if i == 0:
                s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
                quat = np.array(((matrix[2, 1] - matrix[1, 2]) / s, 0.25 * s,
                                 (matrix[0, 1] + matrix[1, 0]) / s,
                                 (matrix[0, 2] + matrix[2, 0]) / s))
            elif i == 1:
                s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
                quat = np.array(((matrix[0, 2] - matrix[2, 0]) / s,
                                 (matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s,
                                 (matrix[1, 2] + matrix[2, 1]) / s))
            else:
                s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
                quat = np.array(((matrix[1, 0] - matrix[0, 1]) / s,
                                 (matrix[0, 2] + matrix[2, 0]) / s,
                                 (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s))
        quat /= np.linalg.norm(quat)
        return Pose(frame, np.asarray(position).copy(), matrix.copy(), quat)

    def _world_pose(self, side):
        name = TCP_FRAMES[side]
        if self.backend == "mujoco":
            body = self._body[side]
            position = self.data.xpos[body].copy()
            rotation = self.data.xmat[body].reshape(3, 3).copy()
            torso_position = self.data.xpos[self._torso_id].copy()
            torso_rotation = self.data.xmat[self._torso_id].reshape(3, 3).copy()
        else:
            placement = self.data.oMf[self._frames[side]]
            position, rotation = placement.translation.copy(), placement.rotation.copy()
            torso = self.data.oMf[self._torso_frame]
            torso_position, torso_rotation = torso.translation.copy(), torso.rotation.copy()
        position = position + rotation @ self.tcp_offset[side]
        pelvis = self._pose(name, position, rotation)
        torso_position_relative = torso_rotation.T @ (position - torso_position)
        torso_rotation_relative = torso_rotation.T @ rotation
        torso = self._pose(f"torso_link->{name}", torso_position_relative, torso_rotation_relative)
        return pelvis, torso

    def compute(self, q_arm, q_waist=(0.0, 0.0, 0.0), frame="pelvis"):
        if frame not in ("pelvis", "torso"):
            raise ValueError("frame must be 'pelvis' or 'torso'; world pose needs a root pose")
        arm = _vector(q_arm, (14,), "q_arm motor15..28")
        waist = _vector(q_waist, (3,), "q_waist motor12..14")
        if self.backend == "ikpy":
            return {side: self._ikpy_pose(side, arm, waist, frame)
                    for side in ("left", "right")}
        self._configuration(arm, waist)
        poses = {}
        for side in ("left", "right"):
            pelvis, torso = self._world_pose(side)
            poses[side] = pelvis if frame == "pelvis" else torso
        return poses


def compare_shadow(fk, q_measured, q_ref, q_waist):
    """Compare geometric FK of measured and commanded arm configurations.

    This is not a physical tracking measurement. Both inputs must come from the
    same caller snapshot; this function has no transport or controller access.
    """
    measured = fk.compute(q_measured, q_waist, frame="torso")
    reference = fk.compute(q_ref, q_waist, frame="torso")
    position_error, orientation_error = {}, {}
    for side in ("left", "right"):
        a, b = measured[side], reference[side]
        position_error[side] = float(np.linalg.norm(a.position_m - b.position_m))
        relative = a.rotation.T @ b.rotation
        orientation_error[side] = float(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
    return ShadowComparison(measured, reference, position_error, orientation_error)


def compare_relative_plan_to_recording(fk, plan, acquire_waist_q):
    """Compare geometric wrist proxies for a relative plan and its source slice.

    Recorded waist motion is compared with the replay's captured fixed waist
    pose. This is a geometric shape comparison, not a tracking or collision test.
    """
    from ..trajectory.replay_c2 import RelativeC2ReplayPlan
    if not isinstance(plan, RelativeC2ReplayPlan):
        raise ValueError("relative C2 replay plan required")
    waist = _vector(acquire_waist_q, (3,), "acquire waist q")
    position_errors = {side: [] for side in ("left", "right")}
    orientation_errors = {side: [] for side in ("left", "right")}
    for i, source_time in enumerate(plan.source_time):
        replay_time = (source_time-plan.source_time[0])/plan.speed_scale
        replay_q = plan.sample(float(replay_time))[0]
        original = fk.compute(plan.source_q[i], plan.source_waist_q[i], frame="pelvis")
        relative = fk.compute(replay_q, waist, frame="pelvis")
        for side in ("left", "right"):
            a, b = original[side], relative[side]
            position_errors[side].append(float(np.linalg.norm(a.position_m-b.position_m)))
            rotation = a.rotation.T @ b.rotation
            orientation_errors[side].append(float(np.arccos(np.clip((np.trace(rotation)-1)/2,-1,1))))
    return {
        "frame": "pelvis",
        "tcp_definition": "uncalibrated wrist-yaw link origin; not calibrated tool center",
        "provenance": "geometric FK only; relative joint replay does not preserve the recorded TCP path",
        "sample_count": int(len(plan.source_time)),
        "motion_scale": plan.motion_scale,
        "speed_scale": plan.speed_scale,
        "position_error_m": {side: {"max": float(np.max(values)),
                                     "mean": float(np.mean(values))}
                             for side, values in position_errors.items()},
        "orientation_error_rad": {side: {"max": float(np.max(values)),
                                         "mean": float(np.mean(values))}
                                 for side, values in orientation_errors.items()},
    }


def analyze_trajectory(arrays, fk, *, frame="pelvis", use_q_ref=False):
    """Compute poses and finite-difference TCP motion from trajectory arrays."""
    q_key = "q_ref" if use_q_ref else "q_measured"
    if q_key not in arrays or "timestamp" not in arrays or "waist_q" not in arrays:
        raise ValueError("trajectory needs timestamp, waist_q and selected arm q array")
    q = np.asarray(arrays[q_key], dtype=float)
    t = np.asarray(arrays["timestamp"], dtype=float)
    waist = np.asarray(arrays["waist_q"], dtype=float)
    if q.ndim != 2 or q.shape[1] != 14 or t.shape != (len(q),) or waist.shape != (len(q), 3):
        raise ValueError("trajectory arrays have invalid dimensions")
    if len(q) < 2 or not all(np.isfinite(x).all() for x in (q, t, waist)):
        raise ValueError("trajectory needs at least two finite samples")
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError("monotonic timestamp must be strictly increasing")
    median_dt = float(np.median(dt))
    positions = {side: np.empty((len(q), 3)) for side in ("left", "right")}
    quaternions = {side: np.empty((len(q), 4)) for side in ("left", "right")}
    for i in range(len(q)):
        for side, pose in fk.compute(q[i], waist[i], frame).items():
            positions[side][i] = pose.position_m
            quaternions[side][i] = pose.quaternion_wxyz
    linear_velocity = {s: np.diff(p, axis=0) / dt[:, None] for s, p in positions.items()}
    linear_speed = {s: np.linalg.norm(v, axis=1) for s, v in linear_velocity.items()}
    arm_dt = np.diff(q, axis=0)
    waist_dt = np.diff(waist, axis=0)
    angular_step = {}
    for side in ("left", "right"):
        dot = np.sum(quaternions[side][:-1] * quaternions[side][1:], axis=1)
        angular_step[side] = 2 * np.arccos(np.clip(np.abs(dot), 0.0, 1.0))
    return {
        "source_joint_array": q_key,
        "frame": frame,
        "tcp_definition": "wrist-yaw link frame origin plus optional configured fixed offset; uncalibrated proxy",
        "time_s": t - t[0],
        "sample_interval_s": dt,
        "sampling": {
            "sample_count": int(len(t)),
            "duration_s": float(t[-1] - t[0]),
            "mean_frequency_hz": float((len(t) - 1) / (t[-1] - t[0])),
            "min_interval_s": float(np.min(dt)),
            "median_interval_s": median_dt,
            "max_interval_s": float(np.max(dt)),
            "intervals_over_1_5x_median": int(np.count_nonzero(dt > 1.5 * median_dt)),
            "intervals_under_0_5x_median": int(np.count_nonzero(dt < 0.5 * median_dt)),
            "strictly_increasing": True,
        },
        "joint_change": {
            "arm_max_abs_step_rad_by_motor15_28": np.max(np.abs(arm_dt), axis=0).tolist(),
            "waist_max_abs_step_rad_by_motor12_14": np.max(np.abs(waist_dt), axis=0).tolist(),
            "arm_max_abs_velocity_rad_s_by_motor15_28": np.max(np.abs(arm_dt / dt[:, None]), axis=0).tolist(),
            "waist_max_abs_velocity_rad_s_by_motor12_14": np.max(np.abs(waist_dt / dt[:, None]), axis=0).tolist(),
        },
        "position_m": positions,
        "quaternion_wxyz": quaternions,
        "linear_velocity_m_s": linear_velocity,
        "linear_speed_m_s": linear_speed,
        "angular_step_rad": angular_step,
        "angular_speed_rad_s": {s: angular_step[s] / dt for s in angular_step},
        "max_position_step_m": {s: float(np.max(np.linalg.norm(np.diff(p, axis=0), axis=1))) for s, p in positions.items()},
        "position_range_m": {s: (np.max(p, axis=0) - np.min(p, axis=0)).tolist() for s, p in positions.items()},
        "max_orientation_step_rad": {s: float(np.max(v)) for s, v in angular_step.items()},
        "orientation_path_length_rad": {s: float(np.sum(v)) for s, v in angular_step.items()},
        "max_linear_speed_m_s": {s: float(np.max(v)) for s, v in linear_speed.items()},
        "max_angular_speed_rad_s": {s: float(np.max(v)) for s, v in {s: angular_step[s] / dt for s in angular_step}.items()},
        "root_pose_available": False,
        "live_tracking_validation": False,
    }
