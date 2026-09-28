"""Spot Micro geometry and naming conventions.

Dimensions follow the open-source Spot Micro (KDY0532 / mike4192 spotMicro):
0.186 m between the front and rear hip pivots, 0.078 m between the left and
right pivots, a 0.055 m abduction ("shoulder") link that carries the leg
plane outboard of the pivot, 0.1075 m upper leg and 0.130 m lower leg.

Frames (REP-103: x forward, y left, z up):

  base_link ── hip pivot (±hip_x, ±hip_y, 0)
                 │ hip_roll  about +x
                 ├─ translate (0, ±hip_link, 0)      → leg plane
                 │ hip_pitch about +y
                 ├─ translate (0, 0, -upper_leg)     → knee
                 │ knee      about +y
                 └─ translate (0, 0, -lower_leg)     → foot centre

All four knees point backward (Spot configuration): positive hip_pitch swings
the upper leg backward, the knee angle is always negative so the lower leg
folds forward under the body.
"""

from dataclasses import dataclass

import numpy as np

LEGS = ('FL', 'FR', 'RL', 'RR')

# (x_sign, y_sign): +x front, +y left.
LEG_SIGNS = {
    'FL': (1, 1),
    'FR': (1, -1),
    'RL': (-1, 1),
    'RR': (-1, -1),
}

# Joint names per leg in kinematic-chain order.
JOINT_NAMES = {
    leg: (f'{leg}_hip_roll_joint', f'{leg}_hip_pitch_joint', f'{leg}_knee_joint')
    for leg in LEGS
}

# Flat 12-entry order used by the ros2_control JointGroupPositionController
# (see quadruped_gazebo/config/ros2_control.yaml).
JOINT_ORDER = tuple(name for leg in LEGS for name in JOINT_NAMES[leg])


@dataclass(frozen=True)
class QuadrupedGeometry:
    """Link lengths in metres. Defaults are the Spot Micro."""

    hip_x: float = 0.093       # half the front-to-rear hip pivot distance
    hip_y: float = 0.039       # half the left-to-right hip pivot distance
    hip_link: float = 0.055    # abduction link: pivot → leg plane (outboard)
    upper_leg: float = 0.1075
    lower_leg: float = 0.130
    foot_radius: float = 0.015

    @property
    def max_reach(self) -> float:
        """Fully stretched leg, hip pitch joint to foot centre."""
        return self.upper_leg + self.lower_leg

    @property
    def min_reach(self) -> float:
        """Fully folded leg, hip pitch joint to foot centre."""
        return abs(self.upper_leg - self.lower_leg)


SPOT_MICRO = QuadrupedGeometry()


def hip_origin(leg: str, geom: QuadrupedGeometry = SPOT_MICRO) -> np.ndarray:
    """Hip roll pivot position in base_link."""
    x_sign, y_sign = LEG_SIGNS[leg]
    return np.array([x_sign * geom.hip_x, y_sign * geom.hip_y, 0.0])
