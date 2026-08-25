"""Launch file for the Video Camera Simulator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_path', default_value='/videos/input.mp4',
                              description='Path to the video file to play back'),
        DeclareLaunchArgument('fps', default_value='10.0',
                              description='Publish rate in Hz'),
        DeclareLaunchArgument('loop', default_value='true',
                              description='Loop video when it ends'),
        DeclareLaunchArgument('paused', default_value='false',
                              description='Start video playback in paused state'),
        DeclareLaunchArgument('width', default_value='640',
                              description='Target width of published images'),
        DeclareLaunchArgument('height', default_value='480',
                              description='Target height of published images'),
        DeclareLaunchArgument('start_delay', default_value='0.0',
                              description='Delay before publishing the first frame'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw',
                              description='Output image topic'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info',
                              description='Output CameraInfo topic'),
        DeclareLaunchArgument('frame_id', default_value='camera_link_optical',
                              description='TF frame_id for published messages'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use a ROS simulation clock'),
        DeclareLaunchArgument('fx', default_value='381.36',
                              description='Focal length X (pixels)'),
        DeclareLaunchArgument('fy', default_value='381.36',
                              description='Focal length Y (pixels)'),
        DeclareLaunchArgument('cx', default_value='320.0',
                              description='Principal point X (pixels)'),
        DeclareLaunchArgument('cy', default_value='240.0',
                              description='Principal point Y (pixels)'),

        Node(
            package='video_camera_sim',
            executable='video_node',
            name='video_camera_sim',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'video_path': LaunchConfiguration('video_path'),
                'fps': LaunchConfiguration('fps'),
                'loop': LaunchConfiguration('loop'),
                'paused': LaunchConfiguration('paused'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'start_delay': LaunchConfiguration('start_delay'),
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'fx': LaunchConfiguration('fx'),
                'fy': LaunchConfiguration('fy'),
                'cx': LaunchConfiguration('cx'),
                'cy': LaunchConfiguration('cy'),
            }],
        ),
    ])
