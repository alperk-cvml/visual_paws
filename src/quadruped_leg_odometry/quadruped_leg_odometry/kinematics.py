"""Leg forward kinematics in base_link, plus the stance-velocity formula.

The chain itself lives in quadruped_kinematics (shared with the gait
controller and checked against the URDF); this module only moves the
hip-frame result into base_link and adds the odometry-specific maths.
"""

import numpy as np

from quadruped_kinematics import (  # noqa: F401  (re-exported for the node)
    JOINT_NAMES, LEGS, LEG_SIGNS, SPOT_MICRO, QuadrupedGeometry, hip_origin,
    leg_jacobian,
)
from quadruped_kinematics import leg_fk as _leg_fk_hip


def leg_fk(q: np.ndarray, leg: str,
           geom: QuadrupedGeometry = SPOT_MICRO) -> tuple[np.ndarray, np.ndarray]:
    """Foot-centre position and geometric Jacobian in base_link.

    Args:
        q: (hip_roll, hip_pitch, knee) joint angles in radians.
        leg: 'FL' | 'FR' | 'RL' | 'RR'.

    Returns:
        (p, J): foot position (3,) and Jacobian (3, 3) with
        foot_velocity = J @ q_dot, both expressed in base_link. The Jacobian
        is the same as in the hip frame because the two frames differ by a
        pure translation.
    """
    y_sign = LEG_SIGNS[leg][1]
    p = hip_origin(leg, geom) + _leg_fk_hip(q, y_sign, geom)
    return p, leg_jacobian(q, y_sign, geom)


def body_velocity_from_stance(foot_pos: list[np.ndarray],
                              foot_vel: list[np.ndarray],
                              omega: np.ndarray) -> np.ndarray:
    """Body linear velocity in base_link from stance-foot kinematics.

    A stance foot is stationary in the world, so its body-relative motion is
    exactly the negative of the body's own motion at that point:
        0 = v_base + omega × p_foot + v_foot
    Averaging over stance feet gives the least-squares body velocity.

    Args:
        foot_pos: stance-foot positions in base_link.
        foot_vel: matching body-relative foot velocities (J @ q_dot).
        omega: body angular velocity in base_link (rad/s).

    Returns:
        Body linear velocity (3,) in base_link. Zero if no stance feet.
    """
    if not foot_pos:
        return np.zeros(3)
    estimates = [-(np.cross(omega, p) + v) for p, v in zip(foot_pos, foot_vel)]
    return np.mean(estimates, axis=0)
