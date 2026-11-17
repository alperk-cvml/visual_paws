"""Launch the depth model comparison node and one point cloud per backend.

The `backends` argument drives both halves: the comparison node publishes a
depth image per key, and a `point_cloud_xyz_node` is spawned per key to turn it
into `<namespace>/<key>/points`. Because every depth image carries the source
image's frame_id, all the clouds land in the same optical frame and overlay
directly in RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'config_file', default_value='',
        description='Backend parameter YAML (empty = the packaged config/backends.yaml)'),
    DeclareLaunchArgument(
        'backends', default_value='gt,dav2_vkitti,dav2_hypersim,metric3d_s',
        description='Comma-separated backend keys; must exist in the config file. '
                    'Drop `gt` when there is no ground-truth depth sensor.'),
    DeclareLaunchArgument(
        'input_image_topic', default_value='/front_camera/image_raw',
        description='Input RGB image topic'),
    DeclareLaunchArgument(
        'input_camera_info_topic', default_value='/front_camera/camera_info',
        description='CameraInfo matching the input RGB stream'),
    DeclareLaunchArgument(
        'output_namespace', default_value='/depth_compare',
        description='Namespace for the per-backend depth and point cloud topics'),
    DeclareLaunchArgument(
        'target_fps', default_value='5.0',
        description='Upper bound on comparison rate (Hz); every backend runs per frame'),
    DeclareLaunchArgument(
        'device', default_value='cuda',
        description='Inference device: cuda | cpu'),
    DeclareLaunchArgument(
        'publish_pointclouds', default_value='true',
        description='Spawn a point_cloud_xyz_node per backend'),
    DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use a ROS simulation clock'),
]


def _setup(context, *args, **kwargs):
    """Resolve the backend list so we can spawn one point cloud node per key."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config_file = arg('config_file') or os.path.join(
        get_package_share_directory('depth_comparison'), 'config', 'backends.yaml'
    )
    keys = [k.strip() for k in arg('backends').split(',') if k.strip()]
    if not keys:
        raise RuntimeError('`backends` resolved to an empty list.')

    namespace = arg('output_namespace').rstrip('/')
    use_sim_time = arg('use_sim_time').lower() in ('true', '1')

    # The YAML supplies the per-backend definitions; the launch arguments
    # override the shared wiring so the same config works for Gazebo and the
    # offline video pipeline.
    comparison_node = Node(
        package='depth_comparison',
        executable='depth_comparison_node',
        name='depth_comparison_node',
        output='screen',
        parameters=[
            config_file,
            {
                'backends': keys,
                'input_image_topic': arg('input_image_topic'),
                'input_camera_info_topic': arg('input_camera_info_topic'),
                'output_namespace': namespace,
                'target_fps': float(arg('target_fps')),
                'device': arg('device'),
                'use_sim_time': use_sim_time,
            },
        ],
    )

    nodes = [comparison_node]

    if arg('publish_pointclouds').lower() in ('true', '1'):
        nodes += [
            Node(
                package='depth_image_proc',
                executable='point_cloud_xyz_node',
                name=f'{key}_point_cloud',
                output='screen',
                remappings=[
                    ('image_rect', f'{namespace}/{key}/image_raw'),
                    ('camera_info', f'{namespace}/{key}/camera_info'),
                    ('points', f'{namespace}/{key}/points'),
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            )
            for key in keys
        ]

    return nodes


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=_setup)])
