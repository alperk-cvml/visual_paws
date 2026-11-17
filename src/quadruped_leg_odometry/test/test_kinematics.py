"""Unit tests for leg FK in base_link and the stance-velocity formula."""

import numpy as np
import pytest

from quadruped_kinematics import SPOT_MICRO, hip_origin, leg_ik, stand_joint_angles
from quadruped_leg_odometry.kinematics import (
    LEGS, LEG_SIGNS, body_velocity_from_stance, leg_fk,
)

HEIGHT = 0.17


def test_fk_stand_pose_puts_feet_on_the_ground_under_the_hips():
    q_all = stand_joint_angles(HEIGHT)
    for i, leg in enumerate(LEGS):
        x_sign, y_sign = LEG_SIGNS[leg]
        p, _ = leg_fk(q_all[3 * i:3 * i + 3], leg)
        assert p[0] == pytest.approx(x_sign * SPOT_MICRO.hip_x, abs=1e-9)
        assert p[1] == pytest.approx(y_sign * (SPOT_MICRO.hip_y + SPOT_MICRO.hip_link), abs=1e-9)
        assert p[2] == pytest.approx(-(HEIGHT - SPOT_MICRO.foot_radius), abs=1e-9)


def test_jacobian_matches_finite_differences():
    rng = np.random.default_rng(42)
    eps = 1e-7
    for _ in range(20):
        q = rng.uniform([-0.7, -1.0, -2.6], [0.7, 1.5, -0.4])
        for leg in LEGS:
            _, jac = leg_fk(q, leg)
            for k in range(3):
                dq = np.zeros(3)
                dq[k] = eps
                p_plus, _ = leg_fk(q + dq, leg)
                p_minus, _ = leg_fk(q - dq, leg)
                fd = (p_plus - p_minus) / (2 * eps)
                assert np.allclose(jac[:, k], fd, atol=1e-6), (leg, k)


def test_fk_round_trips_shared_ik():
    rng = np.random.default_rng(7)
    for _ in range(50):
        q = rng.uniform([-0.4, 0.2, -2.2], [0.4, 1.3, -0.6])
        for leg in LEGS:
            p, _ = leg_fk(q, leg)
            sol = leg_ik(p - hip_origin(leg), LEG_SIGNS[leg][1])
            p_back, _ = leg_fk(sol, leg)
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
