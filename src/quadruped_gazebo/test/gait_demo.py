#!/usr/bin/env python3
"""
Quadruped Gait Demo Node
========================
Publishes simple trot and stand poses to the joint controller
to verify the Gazebo simulation is working correctly.

Run after launching Gazebo:
    ros2 run quadruped_robot gait_demo.py
"""

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# Joint order must match ros2_control.yaml
JOINT_NAMES = [
    "FL_hip_roll_joint", "FL_hip_pitch_joint", "FL_knee_joint",
    "FR_hip_roll_joint", "FR_hip_pitch_joint", "FR_knee_joint",
    "RL_hip_roll_joint", "RL_hip_pitch_joint", "RL_knee_joint",
    "RR_hip_roll_joint", "RR_hip_pitch_joint", "RR_knee_joint",
]

# Comfortable standing pose (radians)
# [roll, hip_pitch, knee] per leg
STAND_POSE = [
    0.0,  0.5, -1.2,   # FL
    0.0,  0.5, -1.2,   # FR
    0.0,  0.5, -1.2,   # RL
    0.0,  0.5, -1.2,   # RR
]

# Sit pose
SIT_POSE = [
    0.0,  0.8, -1.8,   # FL
    0.0,  0.8, -1.8,   # FR
    0.0,  0.3, -0.6,   # RL
    0.0,  0.3, -0.6,   # RR
]


class GaitDemo(Node):
    def __init__(self):
        super().__init__("gait_demo")
        self.pub = self.create_publisher(
            Float64MultiArray,
            "/quadruped_joint_controller/commands",
            10,
        )
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz
        self.t = 0.0
        self.phase = "stand"
        self.get_logger().info("Gait demo started. Publishing to joint controller.")

    def publish_pose(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.pub.publish(msg)

    def trot_pose(self, t):
        """
        Simple trot gait: diagonal pairs move together.
        FL+RR swing while FR+RL support, then swap.
        """
        freq = 1.0   # Hz
        amp_pitch = 0.3   # hip pitch swing amplitude
        amp_knee  = 0.4   # knee amplitude
        phase     = 2 * math.pi * freq * t

        # Diagonal pairs: FL+RR vs FR+RL (180° out of phase)
        def leg_angles(phi):
            swing = math.sin(phi)
            lift  = max(0.0, math.sin(phi)) * amp_knee
            return [
                0.0,
                0.5 + swing * amp_pitch,
                -1.2 - lift,
            ]

        fl_rr = leg_angles(phase)
        fr_rl = leg_angles(phase + math.pi)

        return (
            fl_rr +   # FL
            fr_rl +   # FR
            fr_rl +   # RL
            fl_rr     # RR
        )

    def update(self):
        self.t += 0.02

        # Sequence: stand 3s → trot 6s → sit 2s → repeat
        cycle_t = self.t % 11.0

        if cycle_t < 3.0:
            self.publish_pose(STAND_POSE)
            if self.phase != "stand":
                self.get_logger().info("Phase: STAND")
                self.phase = "stand"
        elif cycle_t < 9.0:
            self.publish_pose(self.trot_pose(cycle_t - 3.0))
            if self.phase != "trot":
                self.get_logger().info("Phase: TROT")
                self.phase = "trot"
        else:
            self.publish_pose(SIT_POSE)
            if self.phase != "sit":
                self.get_logger().info("Phase: SIT")
                self.phase = "sit"


def main(args=None):
    rclpy.init(args=args)
    node = GaitDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
