"""Self-contained checks for cyton_kinematics (run: python test_kinematics.py)."""

from __future__ import annotations

import math
import sys

import numpy as np

import cyton_kinematics as ck


def test_jacobian_matches_finite_differences() -> None:
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.uniform(ck.JOINT_LOWER, ck.JOINT_UPPER)
        J = ck.jacobian(q)
        h = 1e-6
        p0, R0 = ck.tool_pose(q)
        for i in range(ck.DOF):
            dq = np.zeros(ck.DOF)
            dq[i] = h
            p1, R1 = ck.tool_pose(q + dq)
            lin = (p1 - p0) / h
            ang = ck.rotation_vector(R1 @ R0.T) / h
            assert np.allclose(J[:3, i], lin, atol=1e-5), (i, J[:3, i], lin)
            assert np.allclose(J[3:, i], ang, atol=1e-4), (i, J[3:, i], ang)


def test_rpy_round_trip() -> None:
    rng = np.random.default_rng(2)
    for _ in range(200):
        rpy = rng.uniform(-math.pi, math.pi, 3)
        rpy[1] = rng.uniform(-1.5, 1.5)  # away from the gimbal-lock pitch
        R = ck.rpy_to_matrix(*rpy)
        back = ck.rpy_to_matrix(*ck.matrix_to_rpy(R))
        assert np.allclose(R, back, atol=1e-9)


def test_rotation_vector_round_trip() -> None:
    rng = np.random.default_rng(3)
    for _ in range(200):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(0, math.pi - 1e-3)
        R = ck.axis_angle(axis, angle)
        v = ck.rotation_vector(R)
        assert np.allclose(v, axis * angle, atol=1e-6)


def test_ik_full_pose_round_trip() -> None:
    rng = np.random.default_rng(4)
    ok = 0
    trials = 60
    for _ in range(trials):
        q_true = rng.uniform(ck.JOINT_LOWER * 0.7, ck.JOINT_UPPER * 0.7)
        p, R = ck.tool_pose(q_true)
        seed = np.clip(q_true + rng.normal(scale=0.6, size=ck.DOF), ck.JOINT_LOWER, ck.JOINT_UPPER)
        res = ck.solve_ik(p, R, seed, mode="full")
        if res.converged:
            ok += 1
            p2, R2 = ck.tool_pose(res.q)
            assert np.linalg.norm(p2 - p) < 1e-3
            assert np.linalg.norm(ck.rotation_vector(R @ R2.T)) < math.radians(1)
    # Random seeds far from the answer will not always converge for a 7-DOF
    # arm with limits; demand a healthy majority.
    assert ok >= trials * 0.95, f"only {ok}/{trials} converged"


def test_ik_position_and_pointing_modes() -> None:
    rng = np.random.default_rng(5)
    for _ in range(30):
        q_true = rng.uniform(ck.JOINT_LOWER * 0.6, ck.JOINT_UPPER * 0.6)
        p, R = ck.tool_pose(q_true)
        res = ck.solve_ik(p, None, ck.REST_POSE, mode="position")
        assert res.converged, res
        res = ck.solve_ik(p, R, ck.REST_POSE, mode="pointing")
        assert res.converged, res
        _, R2 = ck.tool_pose(res.q)
        assert float(np.dot(R2[:, 2], R[:, 2])) > math.cos(math.radians(1))


def test_ik_rejects_unreachable_gracefully() -> None:
    res = ck.solve_ik(np.array([0.0, 0.0, ck.reach() + 0.2]), None, ck.REST_POSE, mode="position")
    assert not res.converged
    assert res.position_error > 0.1
    assert np.all(res.q >= ck.JOINT_LOWER) and np.all(res.q <= ck.JOINT_UPPER)


def test_calibration_round_trip() -> None:
    cal = ck.ArmCalibration()
    cal.joints[2].sign = -1
    cal.joints[3].zero_raw = 1900
    q = np.array([0.3, -0.7, 1.1, -1.2, 0.5, 0.9, -2.0])
    raw = cal.rad_to_raw(q)
    back = cal.raw_to_rad(raw)
    assert np.allclose(back, q, atol=2 * math.pi / ck.MX_TICKS_PER_REV)
    lo, hi = cal.limits_from_servo([0] * 7, [4095] * 7)
    assert np.allclose(lo, ck.JOINT_LOWER) and np.allclose(hi, ck.JOINT_UPPER)
    lo, hi = cal.limits_from_servo([1024] * 7, [3072] * 7)
    for i, j in enumerate(cal.joints):
        a = j.sign * (1024 - j.zero_raw) * 2 * math.pi / 4096
        b = j.sign * (3072 - j.zero_raw) * 2 * math.pi / 4096
        assert math.isclose(lo[i], max(ck.JOINT_LOWER[i], min(a, b)), abs_tol=1e-9)
        assert math.isclose(hi[i], min(ck.JOINT_UPPER[i], max(a, b)), abs_tol=1e-9)


if __name__ == "__main__":
    failures = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as exc:
                failures += 1
                print(f"FAIL {name}: {exc}")
    sys.exit(1 if failures else 0)
