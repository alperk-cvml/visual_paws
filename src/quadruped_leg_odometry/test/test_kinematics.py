"""Unit tests for leg FK, Jacobian, and the stance-velocity formula."""

import math

import numpy as np
import pytest

from quadruped_leg_odometry.kinematics import (
    HIP_LENGTH, HIP_X_OFFSET, HIP_Y_OFFSET, L1, L2, LEG_SIGNS, LEGS,
    body_velocity_from_stance, leg_fk,
)

# Standing pose used by the gait controller (must match gait_node.py).
STAND = np.array([0.0, 0.5, -1.2])


def test_fk_stand_pose_matches_gait_controller_precomputation():
    # gait_node.py precomputes the standing foot position relative to the hip:
    z_stand = -(L1 * math.cos(0.5) + L2 * math.cos(0.5 - 1.2))
    x_stand = -(L1 * math.sin(0.5) + L2 * math.sin(0.5 - 1.2))

    for leg in LEGS:
        x_sign, y_sign = LEG_SIGNS[leg]
        p, _ = leg_fk(STAND, x_sign, y_sign)
        assert p[0] == pytest.approx(x_sign * HIP_X_OFFSET + x_stand, abs=1e-9)
        assert p[1] == pytest.approx(y_sign * (HIP_Y_OFFSET + HIP_LENGTH), abs=1e-9)
        assert p[2] == pytest.approx(z_stand, abs=1e-9)


def test_jacobian_matches_finite_differences():
    rng = np.random.default_rng(42)
    eps = 1e-7
    for _ in range(20):
        q = rng.uniform([-0.6, -1.2, -2.5], [0.6, 1.2, -0.3])
        for leg in LEGS:
            x_sign, y_sign = LEG_SIGNS[leg]
            _, jac = leg_fk(q, x_sign, y_sign)
            for k in range(3):
                dq = np.zeros(3)
                dq[k] = eps
                p_plus, _ = leg_fk(q + dq, x_sign, y_sign)
                p_minus, _ = leg_fk(q - dq, x_sign, y_sign)
                fd = (p_plus - p_minus) / (2 * eps)
                assert np.allclose(jac[:, k], fd, atol=1e-6), (leg, k)


def test_fk_round_trips_gait_controller_ik():
    # The IK may legitimately return an alternate hip_roll/hip_pitch branch
    # for the same foot point, so the invariant is FK(IK(FK(q))) == FK(q),
    # not joint-space equality.
    gait = pytest.importorskip('quadruped_gait_controller.gait_node')
    rng = np.random.default_rng(7)
    for _ in range(50):
        q = rng.uniform([-0.4, -0.8, -2.2], [0.4, 0.8, -0.5])
        for leg in LEGS:
            x_sign, y_sign = LEG_SIGNS[leg]
            p, _ = leg_fk(q, x_sign, y_sign)
            # gait IK takes foot x relative to the hip, y/z relative to body.
            sol = gait.leg_ik(p[0] - x_sign * HIP_X_OFFSET, p[1], p[2], y_sign)
            p_back, _ = leg_fk(np.array(sol), x_sign, y_sign)
            assert np.allclose(p_back, p, atol=1e-9)


def test_stance_velocity_pure_translation():
    # Feet sweeping backward at 0.1 m/s → body moves forward at 0.1 m/s.
    pos = [np.array([0.1, 0.1, -0.15]), np.array([-0.1, -0.1, -0.15])]
    vel = [np.array([-0.1, 0.0, 0.0])] * 2
    v = body_velocity_from_stance(pos, vel, np.zeros(3))
    assert np.allclose(v, [0.1, 0.0, 0.0])


def test_stance_velocity_pure_rotation():
    # Rotating in place about z: stationary world feet, ω × p accounts for
    # everything, so the linear velocity estimate must be zero.
    omega = np.array([0.0, 0.0, 1.0])
    pos = [np.array([0.1, 0.105, -0.15]), np.array([-0.1, -0.105, -0.15])]
    vel = [-np.cross(omega, p) for p in pos]
    v = body_velocity_from_stance(pos, vel, omega)
    assert np.allclose(v, np.zeros(3), atol=1e-12)


def test_stance_velocity_no_feet_returns_zero():
    assert np.allclose(body_velocity_from_stance([], [], np.zeros(3)), np.zeros(3))
