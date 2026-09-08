"""Launch smoke test: the node starts and /odom_lko appears within 10 s.

Feeds synthetic /joint_states (standing pose) + /imu/data (level, still)
and asserts an Odometry message with near-zero twist comes back.
Skipped automatically outside a ROS environment (no rclpy on the host).
"""

import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from nav_msgs.msg import Odometry            # noqa: E402
from sensor_msgs.msg import Imu, JointState  # noqa: E402

from quadruped_leg_odometry.imu_attitude import GRAVITY          # noqa: E402
from quadruped_leg_odometry.kinematics import JOINT_NAMES, LEGS  # noqa: E402
from quadruped_leg_odometry.leg_odometry_node import LegOdometryNode  # noqa: E402

STAND = (0.0, 0.5, -1.2)


def test_node_publishes_odom_lko():
    rclpy.init()
    try:
        node = LegOdometryNode()
        driver = rclpy.create_node('lko_test_driver')
        joint_pub = driver.create_publisher(JointState, '/joint_states', 10)
        imu_pub = driver.create_publisher(Imu, '/imu/data', 10)
        received: list[Odometry] = []
        driver.create_subscription(Odometry, '/odom_lko', received.append, 10)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(driver)

        deadline = time.monotonic() + 10.0
        seq = 0
        while time.monotonic() < deadline and len(received) < 5:
            stamp = driver.get_clock().now().to_msg()

            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'imu_link'
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = GRAVITY
            imu_pub.publish(imu)

            js = JointState()
            js.header.stamp = stamp
            for leg in LEGS:
                js.name.extend(JOINT_NAMES[leg])
                js.position.extend(STAND)
                js.velocity.extend([0.0, 0.0, 0.0])
            joint_pub.publish(js)

            executor.spin_once(timeout_sec=0.02)
            seq += 1

        assert len(received) >= 5, 'no /odom_lko output within 10 s'
        last = received[-1]
        assert last.header.frame_id == 'odom'
        assert last.child_frame_id == 'base_link'
        # Standing still → near-zero twist and negligible integrated motion.
        twist = last.twist.twist
        assert math.hypot(twist.linear.x, twist.linear.y, twist.linear.z) < 1e-6
        pos = last.pose.pose.position
        assert math.hypot(pos.x, pos.y, pos.z) < 1e-3
    finally:
        rclpy.shutdown()
