"""Whole-body inverse kinematics (Spot Micro style).

The gait plans foot positions in the *nominal body frame* N: a frame that
sits where base_link would be at the commanded stand height with zero roll,
pitch and yaw, i.e. base_link's footprint lifted to body height. The body
may then be posed relative to N (height changes, leaning, turning the torso)
while the feet stay put, which is what makes body-pose control independent
of the gait.

    foot_base = R(rpy)ᵀ · (foot_N − t)          then hip-frame IK per leg
"""

import math

import numpy as np

from .geometry import JOINT_ORDER, LEGS, LEG_SIGNS, SPOT_MICRO, QuadrupedGeometry, hip_origin
from .leg import leg_ik


def rotation_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation matrix for REP-103 fixed-axis roll, pitch, yaw (R = Rz·Ry·Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def neutral_feet(height: float, geom: QuadrupedGeometry = SPOT_MICRO,
                 x_offset: float = 0.0, y_offset: float = 0.0) -> dict[str, np.ndarray]:
    """Neutral footholds in N: below each hip's outboard end, on the ground.

    Args:
        height: base_link height above ground (m).
        x_offset: shift every foothold fore/aft (positive = forward), e.g. to
            move the support polygon under a nose-heavy body.
        y_offset: extra outboard spread beyond the hip link (positive = wider).
    """
    feet = {}
    for leg in LEGS:
        x_sign, y_sign = LEG_SIGNS[leg]
        feet[leg] = np.array([
            x_sign * geom.hip_x + x_offset,
            y_sign * (geom.hip_y + geom.hip_link + y_offset),
            -(height - geom.foot_radius),
        ])
    return feet


def foot_in_base(foot_n: np.ndarray, position: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """Express a foot given in N in base_link, for base_link posed at
    (position, rot) relative to N."""
    return rot.T @ (foot_n - position)


def feet_in_hip_frames(feet_n: dict[str, np.ndarray], position, rpy,
                       geom: QuadrupedGeometry = SPOT_MICRO) -> dict[str, np.ndarray]:
    """Foot targets in each leg's hip frame for a body pose relative to N."""
    rot = rotation_rpy(*rpy)
    position = np.asarray(position, dtype=float)
    return {
        leg: foot_in_base(feet_n[leg], position, rot) - hip_origin(leg, geom)
        for leg in LEGS
    }


def body_ik(feet_n: dict[str, np.ndarray], position=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0), geom: QuadrupedGeometry = SPOT_MICRO,
            clamp: bool = True) -> np.ndarray:
    """All 12 joint angles (JOINT_ORDER) for feet in N and a body pose in N."""
    targets = feet_in_hip_frames(feet_n, position, rpy, geom)
    q = np.empty(12)
    for i, leg in enumerate(LEGS):
        q[3 * i:3 * i + 3] = leg_ik(targets[leg], LEG_SIGNS[leg][1], geom, clamp)
    return q


def stand_joint_angles(height: float, geom: QuadrupedGeometry = SPOT_MICRO,
                       x_offset: float = 0.0, y_offset: float = 0.0) -> np.ndarray:
    """Joint angles (JOINT_ORDER) of the level stand at the given body height."""
    return body_ik(neutral_feet(height, geom, x_offset, y_offset), geom=geom)


__all__ = ['JOINT_ORDER', 'body_ik', 'feet_in_hip_frames', 'foot_in_base',
           'neutral_feet', 'rotation_rpy', 'stand_joint_angles']
