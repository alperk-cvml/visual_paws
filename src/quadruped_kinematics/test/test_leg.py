"""Leg FK / IK / Jacobian tests."""

import math

import numpy as np
import pytest

from quadruped_kinematics import SPOT_MICRO, ReachError, leg_fk, leg_ik, leg_jacobian
from quadruped_kinematics.geometry import LEGS, LEG_SIGNS

Q_LO = np.array([-0.7, -1.0, -2.6])
Q_HI = np.array([0.7, 1.5, -0.4])


def test_zero_pose_hangs_straight_down():
    for y_sign in (1, -1):
        p = leg_fk(np.zeros(3), y_sign)
        assert p == pytest.approx([0.0, y_sign * SPOT_MICRO.hip_link,
                                   -SPOT_MICRO.max_reach])


def test_positive_hip_pitch_swings_foot_backward():
    p0 = leg_fk([0.0, 0.0, -1.0], 1)
    p1 = leg_fk([0.0, 0.3, -1.0], 1)
    assert p1[0] < p0[0]


def _foot_below_hip_pitch_axis(q) -> bool:
    # In-plane height of the foot relative to the hip pitch axis. The IK
    # always returns the branch with the foot below the axis (the walking
    # workspace); a foot swung up above the axis has a mirrored hip-roll
    # solution and is excluded from the joint-space round trip.
    l2, l3 = SPOT_MICRO.upper_leg, SPOT_MICRO.lower_leg
    return -l2 * math.cos(q[1]) - l3 * math.cos(q[1] + q[2]) < -0.005


def test_ik_fk_round_trip_in_joint_space():
    # Within the knee-backward, foot-below-hip workspace the IK solution is
    # unique, so joint angles must round-trip exactly (not just the foot
    # position).
    rng = np.random.default_rng(1)
    checked = 0
    while checked < 300:
        q = rng.uniform(Q_LO, Q_HI)
        if not _foot_below_hip_pitch_axis(q):
            continue
        checked += 1
        for leg in LEGS:
            y_sign = LEG_SIGNS[leg][1]
            p = leg_fk(q, y_sign)
            q_back = leg_ik(p, y_sign, clamp=False)
            assert np.allclose(q_back, q, atol=1e-9), (leg, q, q_back)


def test_ik_knee_always_folded_forward():
    rng = np.random.default_rng(2)
    for _ in range(200):
        q = rng.uniform(Q_LO, Q_HI)
        p = leg_fk(q, -1)
        assert leg_ik(p, -1)[2] <= 0.0


def test_unreachable_raises_or_clamps():
    far = np.array([0.0, SPOT_MICRO.hip_link, -0.5])
    with pytest.raises(ReachError):
        leg_ik(far, 1, clamp=False)
    q = leg_ik(far, 1, clamp=True)
    p = leg_fk(q, 1)
    # Clamped solution lands on the workspace boundary, straight below.
    assert np.linalg.norm(p - np.array([0.0, SPOT_MICRO.hip_link, 0.0])) == \
        pytest.approx(SPOT_MICRO.max_reach, abs=1e-5)
    assert math.isfinite(q[2]) and q[2] <= 0.0


def test_jacobian_matches_finite_differences():
    rng = np.random.default_rng(3)
    eps = 1e-7
    for _ in range(30):
        q = rng.uniform(Q_LO, Q_HI)
        for leg in LEGS:
            y_sign = LEG_SIGNS[leg][1]
            jac = leg_jacobian(q, y_sign)
            for k in range(3):
                dq = np.zeros(3)
                dq[k] = eps
                fd = (leg_fk(q + dq, y_sign) - leg_fk(q - dq, y_sign)) / (2 * eps)
                assert np.allclose(jac[:, k], fd, atol=1e-6), (leg, k)
