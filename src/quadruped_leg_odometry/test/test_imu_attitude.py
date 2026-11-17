"""Unit tests for the complementary attitude filter."""

import math

import numpy as np

from quadruped_leg_odometry.imu_attitude import (
    GRAVITY, ComplementaryAttitude, quat_to_rot,
)


def _roll_pitch(q):
    r = quat_to_rot(q)
    pitch = -math.asin(max(-1.0, min(1.0, r[2, 0])))
    roll = math.atan2(r[2, 1], r[2, 2])
    return roll, pitch


def test_gyro_integration_yaw():
    f = ComplementaryAttitude()
    # 1 rad/s about z for 1 s at 200 Hz, freefall-invalid accel (rejected).
    for _ in range(200):
        f.update(np.array([0.0, 0.0, 1.0]), np.zeros(3), 1.0 / 200.0)
    r = quat_to_rot(f.q)
    yaw = math.atan2(r[1, 0], r[0, 0])
    np.testing.assert_allclose(yaw, 1.0, atol=1e-3)


def test_accel_corrects_initial_tilt():
    f = ComplementaryAttitude(accel_gain=2.0)
    # Wrong initial estimate: pitched 0.3 rad. Body actually level and still.
    f.q = np.array([math.cos(0.15), 0.0, math.sin(0.15), 0.0])
    accel = np.array([0.0, 0.0, GRAVITY])
    for _ in range(2000):
        f.update(np.zeros(3), accel, 1.0 / 200.0)
    roll, pitch = _roll_pitch(f.q)
    assert abs(roll) < 1e-3
    assert abs(pitch) < 1e-3


def test_dynamic_accel_is_rejected():
    f = ComplementaryAttitude(accel_gain=2.0)
    q0 = f.q.copy()
    # |accel| far from g → no correction, no gyro → attitude unchanged.
    f.update(np.zeros(3), np.array([0.0, 0.0, 2.0 * GRAVITY]), 1.0 / 200.0)
    np.testing.assert_allclose(f.q, q0, atol=1e-12)
