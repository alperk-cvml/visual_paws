import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('quadruped_gait_controller'),
        'config', 'gait_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('gait_type', default_value='trot',
                              description='trot | walk'),
        Node(
            package='quadruped_gait_controller',
            executable='gait_controller',
            name='gait_controller',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'gait_type': LaunchConfiguration('gait_type'),
                },
            ],
        ),
    ])
