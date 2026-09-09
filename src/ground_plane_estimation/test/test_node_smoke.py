"""Launch smoke test: the node starts and publishes a ground distance.

Feeds a synthetic PointCloud2 of a flat floor 0.18 m below base_link (cloud
stamped directly in the base frame, so no TF is required) and asserts a Range
close to 0.18 m appears on /ground_plane/distance within 10 s.
Skipped automatically outside a ROS environment (no rclpy on the host).
"""

import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from sensor_msgs.msg import PointCloud2, Range          # noqa: E402
from sensor_msgs_py import point_cloud2                 # noqa: E402
from std_msgs.msg import Header                         # noqa: E402

from ground_plane_estimation.ground_plane_node import GroundPlaneNode  # noqa: E402

HEIGHT = 0.18


def _floor_cloud(stamp) -> PointCloud2:
    rng = np.random.default_rng(0)
    xy = rng.uniform([0.5, -2.0], [4.0, 2.0], size=(2000, 2))
    z = np.full(len(xy), -HEIGHT) + rng.normal(0.0, 0.003, size=len(xy))
    header = Header(stamp=stamp, frame_id='base_link')
    return point_cloud2.create_cloud_xyz32(
        header, np.column_stack((xy, z)).astype(np.float32))


def test_node_publishes_ground_distance():
    rclpy.init()
    try:
        node = GroundPlaneNode()
        driver = rclpy.create_node('ground_plane_test_driver')
        cloud_pub = driver.create_publisher(
            PointCloud2, '/depth_camera/points', 10)
        received: list[Range] = []
        driver.create_subscription(
            Range, '/ground_plane/distance', received.append, 10)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(driver)

        deadline = time.monotonic() + 10.0
        next_pub = 0.0
        while time.monotonic() < deadline and len(received) < 2:
            if time.monotonic() >= next_pub:
                cloud_pub.publish(_floor_cloud(driver.get_clock().now().to_msg()))
                next_pub = time.monotonic() + 0.3  # respect target_fps=5
            executor.spin_once(timeout_sec=0.02)

        assert len(received) >= 2, 'no /ground_plane/distance output within 10 s'
        last = received[-1]
        assert last.header.frame_id == 'base_link'
        assert abs(last.range - HEIGHT) < 0.01
    finally:
        rclpy.shutdown()
