"""Node smoke test: needs rclpy; skipped where ROS is not sourced."""

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from quadruped_gait_controller.gait_node import GaitNode, State  # noqa: E402


def _spin(node, seconds):
    import time
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def test_node_stands_up_then_walks_then_stands():
    rclpy.init()
    try:
        node = GaitNode()
        node.set_parameters([rclpy.parameter.Parameter(
            'stand_up_duration', rclpy.Parameter.Type.DOUBLE, 0.2)])
        node._stand_up_s = 0.2
        received = []
        listener = rclpy.create_node('listener')
        listener.create_subscription(
            Float64MultiArray, '/quadruped_joint_controller/commands',
            lambda m: received.append(list(m.data)), 10)
        pub = listener.create_publisher(Twist, '/cmd_vel', 10)

        _spin(node, 0.5)
        _spin(listener, 0.1)
        assert node._state is State.STANDING
        assert received and len(received[-1]) == 12

        twist = Twist()
        twist.linear.x = 0.2
        for _ in range(30):
            pub.publish(twist)
            _spin(node, 0.02)
        assert node._state is State.WALKING

        for _ in range(120):
            pub.publish(Twist())
            _spin(node, 0.02)
        assert node._state is State.STANDING
        listener.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
