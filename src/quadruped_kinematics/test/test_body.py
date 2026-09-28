"""Body IK tests."""

import math

import numpy as np
import pytest

from quadruped_kinematics import (
    JOINT_ORDER, SPOT_MICRO, body_ik, hip_origin, leg_fk, neutral_feet,
    rotation_rpy, stand_joint_angles,
)
from quadruped_kinematics.geometry import LEGS, LEG_SIGNS

HEIGHT = 0.17


def _feet_from_joints(q, position, rpy):
    """FK the 12 joints back into N to compare against the commanded feet."""
    rot = rotation_rpy(*rpy)
    feet = {}
    for i, leg in enumerate(LEGS):
        p_base = hip_origin(leg) + leg_fk(q[3 * i:3 * i + 3], LEG_SIGNS[leg][1])
        feet[leg] = rot @ p_base + np.asarray(position)
    return feet


def test_joint_order_is_twelve_names_leg_major():
    assert len(JOINT_ORDER) == 12
    assert JOINT_ORDER[:3] == ('FL_hip_roll_joint', 'FL_hip_pitch_joint', 'FL_knee_joint')
    assert JOINT_ORDER[9] == 'RR_hip_roll_joint'


def test_stand_is_level_with_zero_hip_roll():
    q = stand_joint_angles(HEIGHT)
    assert np.allclose(q[0::3], 0.0)
    assert np.allclose(q[1::3], q[1])   # all hip pitches equal
    assert np.allclose(q[2::3], q[2])   # all knees equal
    assert q[2] < 0.0
    # matches the 2-link closed form the URDF uses for initial_value
    h = HEIGHT - SPOT_MICRO.foot_radius
    l2, l3 = SPOT_MICRO.upper_leg, SPOT_MICRO.lower_leg
    knee = -math.acos((h * h - l2 * l2 - l3 * l3) / (2 * l2 * l3))
    pitch = math.acos((l2 * l2 + h * h - l3 * l3) / (2 * l2 * h))
    assert q[1] == pytest.approx(pitch, abs=1e-9)
    assert q[2] == pytest.approx(knee, abs=1e-9)


def test_neutral_feet_are_on_the_ground_under_the_hips():
    feet = neutral_feet(HEIGHT)
    for leg, p in feet.items():
        x_sign, y_sign = LEG_SIGNS[leg]
        assert p[2] == pytest.approx(-(HEIGHT - SPOT_MICRO.foot_radius))
        assert np.sign(p[0]) == x_sign and np.sign(p[1]) == y_sign


def test_body_pose_keeps_feet_fixed_in_n():
    rng = np.random.default_rng(5)
    feet = neutral_feet(HEIGHT)
    for _ in range(50):
        position = rng.uniform([-0.03, -0.03, -0.03], [0.03, 0.03, 0.03])
        rpy = rng.uniform(-0.2, 0.2, size=3)
        q = body_ik(feet, position, rpy, clamp=False)
        back = _feet_from_joints(q, position, rpy)
        for leg in LEGS:
            assert np.allclose(back[leg], feet[leg], atol=1e-9), leg


def test_lowering_body_folds_knees():
    q_high = stand_joint_angles(0.20)
    q_low = stand_joint_angles(0.12)
    assert q_low[2] < q_high[2]      # more negative knee when crouched


def test_wider_stance_rolls_hips_outward():
    q = stand_joint_angles(HEIGHT, y_offset=0.03)
    for i, leg in enumerate(LEGS):
        y_sign = LEG_SIGNS[leg][1]
        assert np.sign(q[3 * i]) == y_sign
