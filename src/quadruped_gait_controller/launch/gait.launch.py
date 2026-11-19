import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gait_node(context):
    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'gait_type': LaunchConfiguration('gait_type'),
    }
    # 'default' keeps the params file's value; a number overrides it (ros2
    # launch rejects empty values, hence the sentinel). The body offset
    # depends on what is mounted on the robot (the ZED 2i rig moves the
    # centre of mass ~1.1 cm forward), so the compose profile sets it.
    body_x = LaunchConfiguration('body_x').perform(context).strip()
    if body_x not in ('', 'default'):
        overrides['body_x'] = float(body_x)
    return [
        Node(
            package='quadruped_gait_controller',
            executable='gait_controller',
            name='gait_controller',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), overrides],
        )
    ]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('quadruped_gait_controller'),
        'config', 'gait_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('gait_type', default_value='trot',
                              description='trot | walk'),
        DeclareLaunchArgument(
            'body_x', default_value='default',
            description="Body fore/aft offset over the feet (m); 'default' keeps params_file"),
        OpaqueFunction(function=_gait_node),
    ])
