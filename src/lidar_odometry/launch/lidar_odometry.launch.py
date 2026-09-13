from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('input_topic', default_value='/lidar/points'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_lidar'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('max_range', default_value='30.0'),
        DeclareLaunchArgument('min_range', default_value='0.3'),
        DeclareLaunchArgument(
            'deskew', default_value='false',
            description='Motion-compensate scans; needs a per-point time '
                        'field in the cloud.'),
        DeclareLaunchArgument(
            'publish_tf', default_value='false',
            description='Publish odom->base_link. Only for standalone '
                        'debugging when no EKF or ground-truth node owns '
                        'the edge.'),
        Node(
            package='lidar_odometry',
            executable='lidar_odometry',
            name='lidar_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': LaunchConfiguration('input_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'max_range': LaunchConfiguration('max_range'),
                'min_range': LaunchConfiguration('min_range'),
                'deskew': LaunchConfiguration('deskew'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }],
        ),
    ])
