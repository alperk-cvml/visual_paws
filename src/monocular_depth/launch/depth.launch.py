"""Launch file for the Metric3D V2 monocular depth node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_name', default_value='metric3d_vit_small',
                              description='Metric3D hub entrypoint: metric3d_vit_small | '
                                          'metric3d_vit_large | metric3d_vit_giant2'),
        DeclareLaunchArgument('repo', default_value='yvanyin/metric3d',
                              description='torch.hub repository providing the model'),
        DeclareLaunchArgument('max_depth', default_value='300.0',
                              description='Clamp on the de-canonicalised depth, in metres'),
        DeclareLaunchArgument('input_image_topic', default_value='/front_camera/image_raw',
                              description='Input RGB image topic'),
        DeclareLaunchArgument('input_camera_info_topic', default_value='',
                              description='Intrinsics source; empty resolves to the '
                                          'camera_info alongside input_image_topic'),
        DeclareLaunchArgument('output_depth_topic', default_value='/depth/image_raw',
                              description='Output metric depth image topic'),
        DeclareLaunchArgument('output_camera_info_topic', default_value='/depth/camera_info',
                              description='Output camera info topic (synced with depth)'),
        DeclareLaunchArgument('device', default_value='cuda',
                              description='Inference device: cuda | cpu'),
        DeclareLaunchArgument('target_fps', default_value='10.0',
                              description='Target inference rate (Hz)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use a ROS simulation clock'),

        Node(
            package='monocular_depth',
            executable='depth_node',
            name='depth_node',
            output='screen',
            parameters=[{
                'model_name': LaunchConfiguration('model_name'),
                'repo': LaunchConfiguration('repo'),
                'max_depth': LaunchConfiguration('max_depth'),
                'input_image_topic': LaunchConfiguration('input_image_topic'),
                'input_camera_info_topic': LaunchConfiguration('input_camera_info_topic'),
                'output_depth_topic': LaunchConfiguration('output_depth_topic'),
                'output_camera_info_topic': LaunchConfiguration('output_camera_info_topic'),
                'device': LaunchConfiguration('device'),
                'target_fps': LaunchConfiguration('target_fps'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
