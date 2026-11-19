"""The simulated ZED 2i rig must match the real driver's frames and optics.

Downstream nodes are configured once for both sim and hardware, so the frame
names here must be exactly the ones the ZED wrapper stamps its data with
(general.camera_name = zed), and the optics must give the nominal HD720
intrinsics of the 2.1 mm lens.
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

xacro = pytest.importorskip("xacro")

URDF = Path(__file__).resolve().parents[1] / "urdf" / "quadruped.urdf.xacro"

# Frames the ZED wrapper publishes or stamps data with (zed_camera_component_main.cpp)
DRIVER_FRAMES = {
    "zed_camera_link",
    "zed_camera_center",
    "zed_left_camera_frame",
    "zed_left_camera_frame_optical",
    "zed_right_camera_frame",
    "zed_right_camera_frame_optical",
}
NOMINAL_FX = 528.0
BASELINE = 0.12


def _expand(**mappings: str) -> ET.Element:
    try:
        document = xacro.process_file(str(URDF), mappings=mappings)
    except Exception as error:  # $(find ...) needs the installed package
        pytest.skip(f"xacro could not expand the URDF here: {error}")
    return ET.fromstring(document.toxml())


def _joint_xyz(robot: ET.Element, child: str) -> list[float]:
    for joint in robot.iter("joint"):
        if joint.find("child").get("link") == child:
            return [float(v) for v in joint.find("origin").get("xyz").split()]
    raise AssertionError(f"no joint with child {child}")


def _sensor(robot: ET.Element, name: str) -> ET.Element:
    for sensor in robot.iter("sensor"):
        if sensor.get("name") == name:
            return sensor
    raise AssertionError(f"sensor {name} missing")


def test_mono_rig_is_unchanged_by_default() -> None:
    robot = _expand()
    sensors = {s.get("name") for s in robot.iter("sensor")}
    assert {"front_camera", "depth_camera"} <= sensors
    assert not any(name.startswith("zed") for name in sensors)
    assert not any(l.get("name").startswith("zed") for l in robot.iter("link"))


def test_stereo_rig_uses_driver_frame_names() -> None:
    robot = _expand(camera_rig="stereo")
    links = {l.get("name") for l in robot.iter("link")}
    assert DRIVER_FRAMES | {"zed_imu_link"} <= links
    assert _sensor(robot, "zed_left_camera").findtext("gz_frame_id") == (
        "zed_left_camera_frame_optical"
    )
    assert _sensor(robot, "zed_right_camera").findtext("gz_frame_id") == (
        "zed_right_camera_frame_optical"
    )
    # ZED depth is registered to the left camera.
    assert _sensor(robot, "zed_depth_ground_truth").findtext("gz_frame_id") == (
        "zed_left_camera_frame_optical"
    )


def test_stereo_geometry_and_intrinsics() -> None:
    robot = _expand(camera_rig="stereo")
    left = _joint_xyz(robot, "zed_left_camera_frame")
    right = _joint_xyz(robot, "zed_right_camera_frame")
    assert left[1] - right[1] == pytest.approx(BASELINE)
    assert left[0] == right[0] and left[2] == right[2]

    for name in ("zed_left_camera", "zed_right_camera", "zed_depth_ground_truth"):
        camera = _sensor(robot, name).find("camera")
        width = float(camera.findtext("image/width"))
        hfov = float(camera.findtext("horizontal_fov"))
        assert (width / 2) / math.tan(hfov / 2) == pytest.approx(NOMINAL_FX)

    # A rectified ROS stereo pair: right P carries Tx = -fx * baseline.
    right_tx = float(
        _sensor(robot, "zed_right_camera").findtext("camera/lens/projection/tx")
    )
    assert right_tx == pytest.approx(-NOMINAL_FX * BASELINE)

    # Eyes and GT depth render on the same step -> identical stamps.
    rates = {
        _sensor(robot, name).findtext("update_rate")
        for name in ("zed_left_camera", "zed_right_camera", "zed_depth_ground_truth")
    }
    assert len(rates) == 1


def test_real_robot_leaves_the_imu_edge_to_the_driver() -> None:
    robot = _expand(camera_rig="stereo", publish_zed_imu_link="false")
    assert "zed_imu_link" not in {l.get("name") for l in robot.iter("link")}
    assert "zed_imu" not in {s.get("name") for s in robot.iter("sensor")}
