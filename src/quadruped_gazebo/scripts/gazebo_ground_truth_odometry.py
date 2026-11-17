#!/usr/bin/env python3
"""Convert Gazebo's model pose stream into canonical ROS odometry."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class GazeboGroundTruthOdometry(Node):
    """Publish the simulated base pose as odometry and, optionally, TF."""

    def __init__(self) -> None:
        super().__init__('gazebo_ground_truth_odometry')
        self.declare_parameter('input_topic', '/gazebo/model_pose')
        # The pose publisher emits the *model* pose in the world frame under
        # the model name, and every link pose *relative to the model* under
        # '<model>/<link>'. base_link is the canonical link at the model
        # origin, so the model pose is base_link's world pose; the link entry
        # is identity forever and must not be used.
        self.declare_parameter('source_child_frame', 'quadruped')
        self.declare_parameter('odom_topic', '/odom_ground_truth')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        input_topic = self.get_parameter('input_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.source_child_frame = self.get_parameter(
            'source_child_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(TFMessage, input_topic, self.pose_callback, 20)

        owner = 'odometry and odom->base_link TF' if self.publish_tf else 'odometry only'
        self.get_logger().info(
            f'Converting {input_topic} to {odom_topic} ({owner})')

    def pose_callback(self, msg: TFMessage) -> None:
        source = next(
            (
                transform for transform in msg.transforms
                if transform.child_frame_id == self.source_child_frame
            ),
            None,
        )
        if source is None:
            self.get_logger().warning(
                f'No {self.source_child_frame} pose in Gazebo pose message',
                throttle_duration_sec=5.0,
            )
            return

        odom = Odometry()
        odom.header.stamp = source.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = source.transform.translation.x
        odom.pose.pose.position.y = source.transform.translation.y
        odom.pose.pose.position.z = source.transform.translation.z
        odom.pose.pose.orientation = source.transform.rotation

        # Gazebo ground truth is deliberately trusted more than estimated
        # sources, while retaining non-zero covariance for EKF conditioning.
        odom.pose.covariance[0] = 1.0e-4
        odom.pose.covariance[7] = 1.0e-4
        odom.pose.covariance[14] = 1.0e-4
        odom.pose.covariance[21] = 1.0e-4
        odom.pose.covariance[28] = 1.0e-4
        odom.pose.covariance[35] = 1.0e-4
        self.odom_pub.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = odom.pose.pose.position.z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboGroundTruthOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
