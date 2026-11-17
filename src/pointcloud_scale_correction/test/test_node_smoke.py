"""Launch smoke test: the node starts and republishes a corrected cloud.

Feeds a scale estimate of 2.0 followed by synthetic clouds and asserts that
/depth_camera/points_corrected carries the same points divided by 2, with the
original header (frame + stamp) preserved, within 10 s.
Skipped automatically outside a ROS environment (no rclpy on the host).
"""

import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.qos import qos_profile_sensor_data           # noqa: E402
from sensor_msgs.msg import PointCloud2                 # noqa: E402
from sensor_msgs_py import point_cloud2                 # noqa: E402
from std_msgs.msg import Float32, Header                # noqa: E402

from pointcloud_scale_correction.scale_correction_node import (  # noqa: E402
    ScaleCorrectionNode,
)

SCALE = 2.0


def _cloud(stamp) -> PointCloud2:
    rng = np.random.default_rng(0)
    points = rng.uniform([-2.0, -2.0, 0.5], [2.0, 2.0, 6.0], size=(500, 3))
    header = Header(stamp=stamp, frame_id='camera_link_optical')
    return point_cloud2.create_cloud_xyz32(header, points.astype(np.float32))


def test_node_republishes_corrected_cloud():
    rclpy.init()
    try:
        node = ScaleCorrectionNode()
        driver = rclpy.create_node('scale_correction_test_driver')
        cloud_pub = driver.create_publisher(
            PointCloud2, '/depth_camera/points', 10)
        scale_pub = driver.create_publisher(Float32, '/ground_plane/scale', 10)
        received: list[PointCloud2] = []
        # Sensor QoS to match the node's best-effort cloud publisher.
        driver.create_subscription(
            PointCloud2, '/depth_camera/points_corrected',
            received.append, qos_profile_sensor_data)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(driver)

        sent = _cloud(driver.get_clock().now().to_msg())
        deadline = time.monotonic() + 10.0
        next_pub = 0.0
        while time.monotonic() < deadline and not received:
            if time.monotonic() >= next_pub:
                scale_pub.publish(Float32(data=SCALE))
                cloud_pub.publish(sent)
                next_pub = time.monotonic() + 0.3
            executor.spin_once(timeout_sec=0.02)

        assert received, 'no corrected cloud within 10 s'
        out = received[-1]
        assert out.header.frame_id == sent.header.frame_id
        assert out.header.stamp == sent.header.stamp
        got = point_cloud2.read_points_numpy(
            out, field_names=('x', 'y', 'z'))
        expected = point_cloud2.read_points_numpy(
            sent, field_names=('x', 'y', 'z')) / SCALE
        np.testing.assert_allclose(got, expected, rtol=1e-5)
    finally:
        rclpy.shutdown()
