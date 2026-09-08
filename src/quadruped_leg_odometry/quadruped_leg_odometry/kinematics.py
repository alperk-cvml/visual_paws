"""Leg forward kinematics for the quadruped.

Mirror of the analytic IK in quadruped_gait_controller/gait_node.py:
given the 3 joint angles of one leg it returns the foot-centre position in
the base_link frame and the geometric Jacobian mapping joint rates to foot
velocity (also in base_link).

Chain per leg (matches quadruped.urdf.xacro):
  base_link
    → translate (±HIP_X_OFFSET, ±HIP_Y_OFFSET, 0)   hip_roll joint, axis x
    → translate (0, ±HIP_LENGTH, 0)                  hip_pitch joint, axis y
    → translate (0, 0, -L1)                          knee joint, axis y
    → translate (0, 0, -L2)                          foot centre (fixed)
"""

import math

import numpy as np

# ── Robot geometry (metres — must match URDF properties) ────────────────────
HIP_X_OFFSET = 0.10    # fore-aft distance from body centre to hip
HIP_Y_OFFSET = 0.065   # lateral distance from body centre to hip
HIP_LENGTH   = 0.04    # hip abduction link length (along Y)
L1           = 0.085   # thigh length
L2           = 0.095   # calf length

LEGS = ('FL', 'FR', 'RL', 'RR')
LEG_SIGNS = {
    'FL': ( 1,  1),   # (x_sign, y_sign)
    'FR': ( 1, -1),
    'RL': (-1,  1),
    'RR': (-1, -1),
}

# Joint-state names per leg, in kinematic-chain order.
JOINT_NAMES = {
    leg: (f'{leg}_hip_roll_joint', f'{leg}_hip_pitch_joint', f'{leg}_knee_joint')
    for leg in LEGS
}

_X = np.array([1.0, 0.0, 0.0])
_Y = np.array([0.0, 1.0, 0.0])


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0,   c,  -s],
                     [0.0,   s,   c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[  c, 0.0,   s],
                     [0.0, 1.0, 0.0],
                     [ -s, 0.0,   c]])


def leg_fk(q: np.ndarray, x_sign: int, y_sign: int) -> tuple[np.ndarray, np.ndarray]:
    """Foot-centre position and geometric Jacobian in base_link.

    Args:
        q: (hip_roll, hip_pitch, knee) joint angles in radians.
        x_sign: +1 front legs, -1 rear legs.
        y_sign: +1 left legs, -1 right legs.

    Returns:
        (p, J): foot position (3,) and Jacobian (3, 3) with
        foot_velocity = J @ q_dot, both expressed in base_link.
    """
    q1, q2, q3 = q

    o1 = np.array([x_sign * HIP_X_OFFSET, y_sign * HIP_Y_OFFSET, 0.0])
    r1 = _rot_x(q1)
    o2 = o1 + r1 @ np.array([0.0, y_sign * HIP_LENGTH, 0.0])
    r2 = r1 @ _rot_y(q2)
    o3 = o2 + r2 @ np.array([0.0, 0.0, -L1])
    r3 = r2 @ _rot_y(q3)
    p = o3 + r3 @ np.array([0.0, 0.0, -L2])

    a1 = _X            # hip_roll axis in base frame
    a2 = r1 @ _Y       # hip_pitch axis after hip roll
    a3 = a2            # knee axis is parallel to hip_pitch (both +y of the leg plane)

    j = np.column_stack([
        np.cross(a1, p - o1),
        np.cross(a2, p - o2),
        np.cross(a3, p - o3),
    ])
    return p, j


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
