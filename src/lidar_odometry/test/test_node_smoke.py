"""Launch smoke test: the node starts and /odom_lidar appears within 10 s.

Feeds synthetic /lidar/points (a static floor + two walls seen from a sensor
translating along +x) plus the static base_link<-lidar_link TF, and asserts
the recovered odometry moves forward in the base frame.

All ROS / kiss_icp imports happen inside the test on purpose: the
launch_testing pytest plugin imports every test module during collection
(looking for `generate_test_description`), and a module-level importorskip
raising there aborts collection of the whole test directory, silently hiding
the other test files. Function-level importorskip degrades to a clean
per-test skip instead.
"""

import time

import numpy as np
import pytest

SPEED = 0.5          # m/s along +x
FRAME_DT = 0.1       # synthetic scan spacing (s)


def _world_points() -> np.ndarray:
    """Static structured scene: floor grid + two perpendicular walls."""
    g = np.arange(-6.0, 6.0, 0.25)
    xx, yy = np.meshgrid(g, g)
    floor = np.stack([xx.ravel(), yy.ravel(), np.full(xx.size, -0.4)], axis=1)
    zz = np.arange(-0.4, 2.0, 0.25)
    wx, wz = np.meshgrid(g, zz)
    wall_y = np.stack([wx.ravel(), np.full(wx.size, 6.0), wz.ravel()], axis=1)
    wall_x = np.stack([np.full(wx.size, 6.0), wx.ravel(), wz.ravel()], axis=1)
    world = np.concatenate([floor, wall_y, wall_x])
    return world + np.random.default_rng(0).normal(0.0, 0.005, world.shape)


def test_node_publishes_odom_lidar():
    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('kiss_icp')

    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster

    from lidar_odometry.lidar_odometry_node import LidarOdometryNode

    rclpy.init()
    try:
        node = LidarOdometryNode()
        driver = rclpy.create_node('lidar_odom_test_driver')

        static_tf = StaticTransformBroadcaster(driver)
        tf = TransformStamped()
        tf.header.stamp = driver.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'lidar_link'
        tf.transform.translation.z = 0.15
        tf.transform.rotation.w = 1.0
        static_tf.sendTransform(tf)

        cloud_pub = driver.create_publisher(PointCloud2, '/lidar/points', 1)
        received: list[Odometry] = []
        driver.create_subscription(Odometry, '/odom_lidar', received.append, 10)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(driver)

        world = _world_points()
        start = driver.get_clock().now()
        deadline = time.monotonic() + 10.0
        k = 0
        while time.monotonic() < deadline and len(received) < 6:
            header = Header()
            header.frame_id = 'lidar_link'
            header.stamp = (start + rclpy.duration.Duration(
                seconds=k * FRAME_DT)).to_msg()
            sensor_pos = np.array([SPEED * k * FRAME_DT, 0.0, 0.0])
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                header, (world - sensor_pos).astype(np.float32)))
            k += 1
            for _ in range(5):
                executor.spin_once(timeout_sec=0.02)

        assert len(received) >= 6, 'no /odom_lidar output within 10 s'
        last = received[-1]
        assert last.header.frame_id == 'odom'
        assert last.child_frame_id == 'base_link'
        # Pure +x translation. The depth-1 input QoS may drop frames, which
        # only advances x further, so bound it from both sides.
        step = SPEED * FRAME_DT
        pos = last.pose.pose.position
        assert step * (len(received) - 1) - 0.05 <= pos.x <= step * k + 0.05
        assert abs(pos.y) < 0.05 and abs(pos.z) < 0.05
        # Twist is delta/dt, so it stays SPEED even across dropped frames.
        assert last.twist.twist.linear.x == pytest.approx(SPEED, abs=0.1)
    finally:
        rclpy.shutdown()
