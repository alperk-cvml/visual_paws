"""Single-leg forward and inverse kinematics.

Everything here is expressed in the *hip frame*: origin at the hip roll
pivot, axes parallel to base_link. Add `hip_origin(leg)` to move into
base_link.

Conventions match the URDF exactly: hip_roll about +x, hip_pitch and knee
about +y, upper leg along -z at zero pitch, knee negative (folded forward).
"""

import math

import numpy as np

from .geometry import SPOT_MICRO, QuadrupedGeometry

_X = np.array([1.0, 0.0, 0.0])
_Y = np.array([0.0, 1.0, 0.0])


class ReachError(ValueError):
    """Foot target outside the leg workspace."""


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0, c]])


def _chain(q, y_sign: int, geom: QuadrupedGeometry):
    q1, q2, q3 = q
    o1 = np.zeros(3)
    r1 = _rot_x(q1)
    o2 = o1 + r1 @ np.array([0.0, y_sign * geom.hip_link, 0.0])
    r2 = r1 @ _rot_y(q2)
    o3 = o2 + r2 @ np.array([0.0, 0.0, -geom.upper_leg])
    r3 = r2 @ _rot_y(q3)
    p = o3 + r3 @ np.array([0.0, 0.0, -geom.lower_leg])
    return o1, o2, o3, p, r1


def leg_fk(q, y_sign: int, geom: QuadrupedGeometry = SPOT_MICRO) -> np.ndarray:
    """Foot centre position (3,) in the hip frame for joint angles q."""
    return _chain(q, y_sign, geom)[3]


def leg_jacobian(q, y_sign: int, geom: QuadrupedGeometry = SPOT_MICRO) -> np.ndarray:
    """Geometric Jacobian (3, 3): foot velocity = J @ q_dot, hip frame."""
    o1, o2, o3, p, r1 = _chain(q, y_sign, geom)
    a1 = _X
    a2 = r1 @ _Y
    a3 = a2
    return np.column_stack([
        np.cross(a1, p - o1),
        np.cross(a2, p - o2),
        np.cross(a3, p - o3),
    ])


def leg_ik(p, y_sign: int, geom: QuadrupedGeometry = SPOT_MICRO,
           clamp: bool = True) -> np.ndarray:
    """Closed-form IK: foot centre in the hip frame → (hip_roll, hip_pitch, knee).

    Args:
        p: (x, y, z) foot centre in the hip frame.
        y_sign: +1 left legs, -1 right legs (sets which way the abduction
            link points).
        geom: link lengths.
        clamp: when True an out-of-reach target is projected onto the
            workspace boundary (nearest reachable point along the same ray)
            instead of raising ReachError. Gait code wants clamping so a
            transient overshoot never aborts a control tick.

    Returns:
        np.ndarray of the three joint angles, knee ≤ 0 (folded forward).
    """
    x, y, z = float(p[0]), float(p[1]), float(p[2])

    # ── Coronal plane: hip roll ─────────────────────────────────────────
    # Distance from the roll axis to the foot; the leg plane is offset from
    # the axis by hip_link, so the in-plane vertical reach is the other leg
    # of that right triangle.
    d_sq = y * y + z * z
    d_min = geom.hip_link + 1e-6
    if d_sq < d_min * d_min:
        if not clamp:
            raise ReachError('foot target inside the abduction link radius')
        d_sq = d_min * d_min
    h = math.sqrt(d_sq - geom.hip_link ** 2)
    hip_roll = math.atan2(y, -z) - math.atan2(y_sign * geom.hip_link, h)

    # ── Sagittal plane: hip pitch + knee (2-link) ───────────────────────
    r = math.sqrt(x * x + h * h)
    r_max = geom.max_reach - 1e-6
    r_min = geom.min_reach + 1e-6
    if r > r_max or r < r_min:
        if not clamp:
            raise ReachError(f'foot target at r={r:.4f} m outside '
                             f'[{r_min:.4f}, {r_max:.4f}]')
        scale = (r_max if r > r_max else r_min) / max(r, 1e-9)
        x, h = x * scale, h * scale
        r = math.sqrt(x * x + h * h)

    l2, l3 = geom.upper_leg, geom.lower_leg
    cos_knee = (r * r - l2 * l2 - l3 * l3) / (2.0 * l2 * l3)
    knee = -math.acos(max(-1.0, min(1.0, cos_knee)))

    # Angle of the hip→foot line from straight down (positive = foot behind
    # the hip), plus the interior angle between that line and the upper leg.
    phi = math.atan2(-x, h)
    cos_beta = (l2 * l2 + r * r - l3 * l3) / (2.0 * l2 * r)
    beta = math.acos(max(-1.0, min(1.0, cos_beta)))
    hip_pitch = phi + beta

    return np.array([hip_roll, hip_pitch, knee])
