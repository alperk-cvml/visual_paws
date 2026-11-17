from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_lko'),
        DeclareLaunchArgument(
            'publish_tf', default_value='false',
            description='Publish odom->base_link. Only for standalone debugging '
                        'when no EKF or ground-truth node owns the edge.'),
        Node(
            package='quadruped_leg_odometry',
            executable='leg_odometry',
            name='leg_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }],
        ),
    ])
