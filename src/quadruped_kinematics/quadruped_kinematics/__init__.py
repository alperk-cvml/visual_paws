"""Spot Micro quadruped kinematics.

Single source of truth for the leg geometry used by every node that reasons
about foot positions (gait controller, leg odometry). The Gazebo model
(`quadruped_gazebo/urdf/quadruped.urdf.xacro`) carries the same numbers as
xacro properties; `test/test_geometry_matches_urdf.py` keeps the two in sync.
"""

from .geometry import (  # noqa: F401
    SPOT_MICRO, JOINT_ORDER, JOINT_NAMES, LEGS, LEG_SIGNS, QuadrupedGeometry,
    hip_origin,
)
from .leg import leg_fk, leg_ik, leg_jacobian, ReachError  # noqa: F401
from .body import (  # noqa: F401
    body_ik, feet_in_hip_frames, foot_in_base, neutral_feet, rotation_rpy,
    stand_joint_angles,
)
