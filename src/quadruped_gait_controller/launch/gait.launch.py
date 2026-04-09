from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quadruped_gait_controller',
            executable='gait_controller',
            name='gait_controller',
            output='screen',
        ),
    ])
