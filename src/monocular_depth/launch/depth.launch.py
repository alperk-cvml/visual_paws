"""Launch file for the Depth Anything V2 monocular depth node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_type', default_value='vits',
                              description='Encoder variant: vits | vitb | vitl'),
        DeclareLaunchArgument('max_depth', default_value='80.0',
                              description='Maximum depth in metres (80 outdoor, 20 indoor)'),
        DeclareLaunchArgument('input_image_topic', default_value='/front_camera/image_raw',
                              description='Input RGB image topic'),
        DeclareLaunchArgument('output_depth_topic', default_value='/depth/image_raw',
                              description='Output metric depth image topic'),
        DeclareLaunchArgument('output_camera_info_topic', default_value='/depth/camera_info',
                              description='Output camera info topic (synced with depth)'),
        DeclareLaunchArgument('device', default_value='cuda',
                              description='Inference device: cuda | cpu'),
        DeclareLaunchArgument('target_fps', default_value='10.0',
                              description='Target inference rate (Hz)'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='Path to model weights (empty = auto-resolve)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use a ROS simulation clock'),

        Node(
            package='monocular_depth',
            executable='depth_node',
            name='depth_node',
            output='screen',
            parameters=[{
                'model_type': LaunchConfiguration('model_type'),
                'max_depth': LaunchConfiguration('max_depth'),
                'input_image_topic': LaunchConfiguration('input_image_topic'),
                'output_depth_topic': LaunchConfiguration('output_depth_topic'),
                'output_camera_info_topic': LaunchConfiguration('output_camera_info_topic'),
                'device': LaunchConfiguration('device'),
                'target_fps': LaunchConfiguration('target_fps'),
                'model_path': LaunchConfiguration('model_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
