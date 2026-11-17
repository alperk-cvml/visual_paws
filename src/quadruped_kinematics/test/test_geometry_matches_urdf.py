"""The python geometry must agree with the xacro properties in the URDF."""

import re
from pathlib import Path

import pytest

from quadruped_kinematics import SPOT_MICRO

URDF = Path(__file__).resolve().parents[2] / 'quadruped_gazebo' / 'urdf' / 'quadruped.urdf.xacro'

PROPERTIES = {
    'hip_x': SPOT_MICRO.hip_x,
    'hip_y': SPOT_MICRO.hip_y,
    'hip_link': SPOT_MICRO.hip_link,
    'upper_leg': SPOT_MICRO.upper_leg,
    'lower_leg': SPOT_MICRO.lower_leg,
    'foot_radius': SPOT_MICRO.foot_radius,
}


@pytest.mark.skipif(not URDF.exists(), reason='source tree layout required')
def test_urdf_properties_match_python_geometry():
    text = URDF.read_text()
    for name, expected in PROPERTIES.items():
        m = re.search(rf'<xacro:property\s+name="{name}"\s+value="([^"]+)"', text)
        assert m, f'property {name} missing from {URDF}'
        assert float(m.group(1)) == pytest.approx(expected, abs=1e-9), name
