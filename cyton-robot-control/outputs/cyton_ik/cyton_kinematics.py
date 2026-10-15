"""Kinematics for the Robai Cyton Gamma 300 (7 revolute joints + gripper).

Geometry comes from the community URDF (Tyler Slabinski's model, as shipped in
SteveMacenski/cyton_gamma_300-1500_operation_and_simulation and
selyunin/cyton_gamma_300). Every joint frame is a pure translation along the
parent z axis, so the model is simply "translate, then rotate about a fixed
axis" per joint.

Coordinate frame: base_link at the bottom of the arm, z up. Joint zero is the
arm pointing straight up. Units: metres and radians. The end effector is the
URDF "virtual_endeffector" point between the gripper fingers.

This module is pure numpy: forward kinematics, the geometric Jacobian, a
damped-least-squares inverse kinematics solver with joint limits and a
null-space "comfort" bias, and the servo raw-unit conversion/calibration.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, asdict

import numpy as np

# --------------------------------------------------------------------------
# Model
# --------------------------------------------------------------------------

JOINT_NAMES = (
    "shoulder_roll",
    "shoulder_pitch",
    "elbow_roll",
    "elbow_pitch",
    "elbow_yaw",
    "wrist_pitch",
    "wrist_roll",
)
# Offset (m) from the previous joint frame along its z axis, and rotation axis
# expressed in the joint's own frame (URDF <origin xyz> and <axis xyz>).
JOINT_OFFSETS = np.array([
    [0.0, 0.0, 0.05315],
    [0.0, 0.0, 0.063],
    [0.0, 0.0, 0.089],
    [0.0, 0.0, 0.052],
    [0.0, 0.0, 0.072],
    [0.0, 0.0, 0.0718],
    [0.0, 0.0, 0.051425],
])
JOINT_AXES = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])
# Software joint limits (rad). These are the ranges the ROS dynamixel
# controller config for the Cyton Gamma 300 drove the real arm through
# (cyton_gamma_300_controller.yaml: rolls ±1720 ticks, pitch/yaw ±1290 ticks
# about the zero tick), which is wider than the URDF's ±105° pitch limits.
# They are intersected at run time with each servo's own stored CW/CCW window,
# which the gateway also enforces on every MOVE.
_ROLL = 1720 * 2 * math.pi / 4096      # ≈ ±151°
_PITCH = 1290 * 2 * math.pi / 4096     # ≈ ±113°
JOINT_LOWER = -np.array([_ROLL, _PITCH, _ROLL, _PITCH, _PITCH, _PITCH, _ROLL])
JOINT_UPPER = -JOINT_LOWER
TOOL_OFFSET = np.array([-0.002316, 0.0079, 0.079425])  # wrist_roll -> virtual_endeffector
DOF = 7

# A mildly bent posture used as the null-space attractor so the redundant
# joint does not drift toward its limits.
REST_POSE = np.array([0.0, 0.35, 0.0, 0.9, 0.0, 0.35, 0.0])


# --------------------------------------------------------------------------
# Rotations
# --------------------------------------------------------------------------

def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rotation matrix for a rotation of `angle` about unit `axis` (Rodrigues)."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF / ROS convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """Inverse of rpy_to_matrix. Handles the pitch = ±90° singularity."""
    sp = -R[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) > 0.999999:
        # Gimbal lock: put all the remaining rotation into roll.
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def rotation_vector(R: np.ndarray) -> np.ndarray:
    """Log map: axis * angle for rotation matrix R."""
    cos_angle = (np.trace(R) - 1.0) / 2.0
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    if abs(angle - math.pi) < 1e-6:
        # Near 180°: extract axis from the symmetric part.
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        # Fix signs using off-diagonals.
        i = int(np.argmax(axis))
        for j in range(3):
            if j != i and A[i, j] < 0:
                axis[j] = -axis[j]
        axis /= max(np.linalg.norm(axis), 1e-12)
        return axis * angle
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * math.sin(angle))
    return axis * angle


# --------------------------------------------------------------------------
# Forward kinematics and Jacobian
# --------------------------------------------------------------------------

def forward_kinematics(q: np.ndarray) -> tuple[list[np.ndarray], np.ndarray]:
    """Return ([T_base_joint_i for i in 0..6], T_base_tool).

    Each transform is a 4x4 matrix. Joint transforms are taken *after* the
    joint rotation, so their origin is the joint centre and their z axis is
    the link direction leaving that joint.
    """
    T = np.eye(4)
    frames: list[np.ndarray] = []
    for i in range(DOF):
        step = np.eye(4)
        step[:3, 3] = JOINT_OFFSETS[i]
        step[:3, :3] = axis_angle(JOINT_AXES[i], float(q[i]))
        T = T @ step
        frames.append(T.copy())
    tool = np.eye(4)
    tool[:3, 3] = TOOL_OFFSET
    return frames, T @ tool


def tool_pose(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(position xyz, rotation matrix) of the tool frame."""
    _, T = forward_kinematics(q)
    return T[:3, 3].copy(), T[:3, :3].copy()


def jacobian(q: np.ndarray) -> np.ndarray:
    """6x7 geometric Jacobian of the tool frame in base coordinates.

    Rows 0-2: linear velocity, rows 3-5: angular velocity.
    """
    frames, T_tool = forward_kinematics(q)
    p_tool = T_tool[:3, 3]
    J = np.zeros((6, DOF))
    for i in range(DOF):
        R = frames[i][:3, :3]
        z = R @ JOINT_AXES[i]          # joint axis in base frame
        p = frames[i][:3, 3]           # joint origin in base frame
        J[:3, i] = np.cross(z, p_tool - p)
        J[3:, i] = z
    return J


def link_points(q: np.ndarray) -> np.ndarray:
    """Polyline of joint centres from base to tool tip, for drawing (N x 3)."""
    frames, T_tool = forward_kinematics(q)
    pts = [np.zeros(3)] + [f[:3, 3] for f in frames] + [T_tool[:3, 3]]
    return np.array(pts)


# --------------------------------------------------------------------------
# Inverse kinematics
# --------------------------------------------------------------------------

@dataclass
class IKResult:
    q: np.ndarray
    converged: bool
    iterations: int
    position_error: float      # metres
    orientation_error: float   # radians
    at_limit: list[str] = field(default_factory=list)


def _pose_error(q: np.ndarray, target_p: np.ndarray, target_R: np.ndarray | None, mode: str) -> np.ndarray:
    p, R = tool_pose(q)
    e_p = target_p - p
    if mode == "position" or target_R is None:
        return np.concatenate([e_p, np.zeros(3)])
    if mode == "pointing":
        # Only align the tool z axis (approach direction); leave roll free.
        z_now = R[:, 2]
        z_want = target_R[:, 2]
        axis = np.cross(z_now, z_want)
        s = np.linalg.norm(axis)
        c = float(np.dot(z_now, z_want))
        angle = math.atan2(s, c)
        if s < 1e-9:
            return np.concatenate([e_p, np.zeros(3)])
        return np.concatenate([e_p, axis / s * angle])
    # full 6-D: rotation that takes the current frame onto the target frame
    e_R = rotation_vector(target_R @ R.T)
    return np.concatenate([e_p, e_R])


def _solve_once(
    target_position: np.ndarray,
    target_rotation: np.ndarray | None,
    q0: np.ndarray,
    mode: str,
    lower: np.ndarray,
    upper: np.ndarray,
    rest: np.ndarray,
    max_iterations: int,
    position_tolerance: float,
    orientation_tolerance: float,
    damping: float,
    max_step: float,
    orientation_weight: float,
    nullspace_gain: float,
) -> IKResult:
    q = np.clip(np.asarray(q0, dtype=float).copy(), lower, upper)
    use_rot = mode != "position" and target_rotation is not None
    W = np.diag([1.0, 1.0, 1.0] + [orientation_weight] * 3) if use_rot else np.eye(3)
    converged = False
    iterations = 0
    for iterations in range(1, max_iterations + 1):
        e = _pose_error(q, target_position, target_rotation, mode)
        pos_err = float(np.linalg.norm(e[:3]))
        rot_err = float(np.linalg.norm(e[3:])) if use_rot else 0.0
        if pos_err < position_tolerance and rot_err < orientation_tolerance:
            converged = True
            break
        J = jacobian(q)
        if not use_rot:
            J = J[:3]
            e = e[:3]
        Jw = W @ J
        e_w = W @ e
        # Damping shrinks with the error so the final approach is Newton-like.
        err_norm = float(np.linalg.norm(e_w))
        lam = damping * min(1.0, err_norm / 0.05) + 1e-4
        JJt = Jw @ Jw.T
        dq = Jw.T @ np.linalg.solve(JJt + lam * lam * np.eye(JJt.shape[0]), e_w)
        # Null-space bias toward the rest pose using an (undamped) projector so
        # it does not inject task-space error.
        N = np.eye(DOF) - np.linalg.pinv(Jw, rcond=1e-4) @ Jw
        dq += N @ (nullspace_gain * (rest - q))
        biggest = float(np.max(np.abs(dq)))
        if biggest > max_step:
            dq *= max_step / biggest
        q = np.clip(q + dq, lower, upper)

    e = _pose_error(q, target_position, target_rotation, mode)
    at_limit = [
        JOINT_NAMES[i] for i in range(DOF)
        if abs(q[i] - lower[i]) < 1e-6 or abs(q[i] - upper[i]) < 1e-6
    ]
    return IKResult(
        q=q,
        converged=converged,
        iterations=iterations,
        position_error=float(np.linalg.norm(e[:3])),
        orientation_error=float(np.linalg.norm(e[3:])) if use_rot else 0.0,
        at_limit=at_limit,
    )


def solve_ik(
    target_position: np.ndarray,
    target_rotation: np.ndarray | None,
    q0: np.ndarray,
    *,
    mode: str = "full",             # "full" | "pointing" | "position"
    lower: np.ndarray | None = None,
    upper: np.ndarray | None = None,
    rest: np.ndarray | None = None,
    max_iterations: int = 300,
    position_tolerance: float = 5e-4,     # 0.5 mm
    orientation_tolerance: float = 0.5 * math.pi / 180.0,
    damping: float = 0.05,
    max_step: float = 0.25,               # rad per iteration, per joint
    orientation_weight: float = 0.3,      # metres per radian equivalence
    nullspace_gain: float = 0.05,
    restarts: int = 4,
) -> IKResult:
    """Damped least squares IK from seed q0, with restarts.

    The redundant DOF is resolved by a weak null-space pull toward `rest`
    (default REST_POSE). Joint limits are enforced by clamping each iterate.
    If the seed does not converge, the solver retries from the rest pose and
    then from random poses (deterministic RNG) and returns the best result.
    """
    lower = JOINT_LOWER if lower is None else np.asarray(lower, dtype=float)
    upper = JOINT_UPPER if upper is None else np.asarray(upper, dtype=float)
    rest = np.clip(REST_POSE if rest is None else np.asarray(rest, dtype=float), lower, upper)
    target_position = np.asarray(target_position, dtype=float)
    args = (mode, lower, upper, rest, max_iterations, position_tolerance, orientation_tolerance,
            damping, max_step, orientation_weight, nullspace_gain)

    seeds = [np.asarray(q0, dtype=float), rest]
    rng = np.random.default_rng(12345)
    for _ in range(max(0, restarts - 1)):
        seeds.append(rng.uniform(lower, upper))
    best: IKResult | None = None
    for seed in seeds[: restarts + 1]:
        res = _solve_once(target_position, target_rotation, seed, *args)
        if best is None or (res.position_error + 0.3 * res.orientation_error) < (
                best.position_error + 0.3 * best.orientation_error):
            best = res
        if res.converged:
            return res
    assert best is not None
    return best


def reach() -> float:
    """Maximum distance from the base to the tool point (arm fully extended)."""
    return float(np.sum(JOINT_OFFSETS[:, 2]) + np.linalg.norm(TOOL_OFFSET))


# --------------------------------------------------------------------------
# Servo calibration (raw units <-> radians)
# --------------------------------------------------------------------------

MX_TICKS_PER_REV = 4096          # MX-28 / MX-64, Protocol 1.0: 0..4095 = 360°
AX_TICKS = 1024                  # AX-12A: 0..1023 = 300°
AX_RANGE_RAD = math.radians(300)

# Base -> wrist in daisy-chain order. The ROS driver config for this arm model
# uses IDs 0-6 (gripper 7) in exactly this order, with no flipped joints and
# zero ticks of 2045-2070; this arm has been re-numbered to 8-15.
DEFAULT_JOINT_IDS = (8, 9, 10, 11, 12, 13, 14)
GRIPPER_ID = 15


@dataclass
class JointCalibration:
    servo_id: int
    zero_raw: int = 2048       # raw reading when the joint is at URDF zero
    sign: int = 1              # +1 if raw increases with positive joint angle
    ticks_per_rev: int = MX_TICKS_PER_REV


@dataclass
class ArmCalibration:
    joints: list[JointCalibration] = field(
        default_factory=lambda: [JointCalibration(i) for i in DEFAULT_JOINT_IDS]
    )
    gripper_id: int = GRIPPER_ID
    gripper_open_raw: int = 0
    gripper_closed_raw: int = 515

    @property
    def servo_ids(self) -> list[int]:
        return [j.servo_id for j in self.joints]

    def rad_to_raw(self, q: np.ndarray) -> list[int]:
        out = []
        for value, cal in zip(q, self.joints):
            ticks = float(value) * cal.ticks_per_rev / (2.0 * math.pi)
            raw = int(round(cal.zero_raw + cal.sign * ticks))
            out.append(max(0, min(cal.ticks_per_rev - 1, raw)))
        return out

    def raw_to_rad(self, raw: list[int] | np.ndarray) -> np.ndarray:
        q = np.zeros(DOF)
        for i, (value, cal) in enumerate(zip(raw, self.joints)):
            q[i] = cal.sign * (float(value) - cal.zero_raw) * (2.0 * math.pi) / cal.ticks_per_rev
        return q

    def limits_from_servo(self, cw: list[int], ccw: list[int]) -> tuple[np.ndarray, np.ndarray]:
        """Joint-angle limits = URDF limits ∩ the servo's own CW/CCW window."""
        lo = JOINT_LOWER.copy()
        hi = JOINT_UPPER.copy()
        for i, cal in enumerate(self.joints):
            a = cal.sign * (cw[i] - cal.zero_raw) * (2.0 * math.pi) / cal.ticks_per_rev
            b = cal.sign * (ccw[i] - cal.zero_raw) * (2.0 * math.pi) / cal.ticks_per_rev
            lo[i] = max(lo[i], min(a, b))
            hi[i] = min(hi[i], max(a, b))
        return lo, hi

    def save(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(asdict(self), fh, indent=2)

    @classmethod
    def load(cls, path: str) -> "ArmCalibration":
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
        cal = cls()
        cal.joints = [JointCalibration(**j) for j in data["joints"]]
        cal.gripper_id = data.get("gripper_id", GRIPPER_ID)
        cal.gripper_open_raw = data.get("gripper_open_raw", 0)
        cal.gripper_closed_raw = data.get("gripper_closed_raw", 515)
        return cal


DEFAULT_CALIBRATION_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cyton_calibration.json")


def load_calibration(path: str = DEFAULT_CALIBRATION_PATH) -> ArmCalibration:
    if os.path.exists(path):
        return ArmCalibration.load(path)
    return ArmCalibration()


if __name__ == "__main__":
    q = np.zeros(DOF)
    p, R = tool_pose(q)
    print(f"reach {reach():.4f} m; tool at zero pose: {p.round(4)} (m)")
    print("rpy(deg):", [round(math.degrees(a), 2) for a in matrix_to_rpy(R)])
