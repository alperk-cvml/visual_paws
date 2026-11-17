"""Launch smoke test: the node starts and publishes a scale estimate.

Feeds two synthetic odometry streams — a metric reference walking at
0.5 m/s and an estimate reporting the same walk scaled ×2 — and asserts
that /odom_scale/scale converges to ≈ 2.0 within 10 s.
Skipped automatically outside a ROS environment (no rclpy on the host).
"""

import time

import pytest

rclpy = pytest.importorskip('rclpy')

from nav_msgs.msg import Odometry                        # noqa: E402
from std_msgs.msg import Float32                         # noqa: E402

from odometry_scale_estimation.odom_scale_node import (  # noqa: E402
    OdomScaleNode,
)

SCALE = 2.0
SPEED = 0.5  # m/s along x


def _odom(stamp_msg, x: float) -> Odometry:
    msg = Odometry()
    msg.header.stamp = stamp_msg
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_node_publishes_scale_estimate():
    rclpy.init()
    try:
        node = OdomScaleNode()
        driver = rclpy.create_node('odom_scale_test_driver')
        ref_pub = driver.create_publisher(Odometry, '/odom_lko', 10)
        est_pub = driver.create_publisher(Odometry, '/odom_lidar', 10)
        received: list[Float32] = []
        driver.create_subscription(
            Float32, '/odom_scale/scale', received.append, 10)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(driver)

        start = time.monotonic()
        deadline = start + 10.0
        next_pub = 0.0
        while time.monotonic() < deadline and len(received) < 3:
            if time.monotonic() >= next_pub:
                t = time.monotonic() - start
                stamp = driver.get_clock().now().to_msg()
                ref_pub.publish(_odom(stamp, SPEED * t))
                est_pub.publish(_odom(stamp, SCALE * SPEED * t))
                next_pub = time.monotonic() + 0.05
            executor.spin_once(timeout_sec=0.02)

        assert received, 'no scale estimate within 10 s'
        assert abs(received[-1].data - SCALE) < 0.05
    finally:
        rclpy.shutdown()
