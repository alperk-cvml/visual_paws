"""Launch one ground plane estimator per input point cloud topic.

A single topic publishes directly under `output_namespace`. With several
topics (e.g. the depth comparison channels) each estimator gets its own
`<output_namespace>/<key>/*` subtree, where <key> is derived from the topic
(`/depth_compare/dav2_vkitti/points` -> `dav2_vkitti`), so the per-model
distances can be plotted side by side.
"""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'input_topics', default_value='/depth_camera/points',
        description='Comma-separated PointCloud2 topics; one estimator each'),
    DeclareLaunchArgument(
        'output_namespace', default_value='/ground_plane',
        description='Parent namespace for distance/camera_height/scale/marker'),
    DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Frame whose origin the ground distance is measured from'),
    DeclareLaunchArgument(
        'expected_base_height', default_value='0.0',
        description='Nominal standing height of base_frame above ground (m); '
                    '> 0 enables the /scale topic with a static reference'),
    DeclareLaunchArgument(
        'height_reference_frame', default_value='',
        description='TF frame whose z=0 plane is the true ground (e.g. `odom` '
                    'in ground-truth sim); enables /scale when '
                    'expected_base_height is 0'),
    DeclareLaunchArgument(
        'target_fps', default_value='5.0',
        description='Upper bound on per-topic processing rate (Hz)'),
    DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use a ROS simulation clock'),
]


def _topic_key(topic: str) -> str:
    """`/depth_compare/dav2_vkitti/points` -> `dav2_vkitti`."""
    parts = [s for s in topic.split('/') if s]
    if len(parts) > 1 and parts[-1] in ('points', 'pointcloud', 'point_cloud'):
        parts = parts[:-1]
    return re.sub(r'\W', '_', parts[-1])


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    topics = [t.strip() for t in arg('input_topics').split(',') if t.strip()]
    if not topics:
        raise RuntimeError('`input_topics` resolved to an empty list.')

    namespace = arg('output_namespace').rstrip('/')
    common = {
        'base_frame': arg('base_frame'),
        'expected_base_height': float(arg('expected_base_height')),
        'height_reference_frame': arg('height_reference_frame'),
        'target_fps': float(arg('target_fps')),
        'use_sim_time': arg('use_sim_time').lower() in ('true', '1'),
    }

    nodes = []
    for topic in topics:
        key = _topic_key(topic)
        nodes.append(Node(
            package='ground_plane_estimation',
            executable='ground_plane',
            name=f'ground_plane_{key}',
            output='screen',
            parameters=[{
                **common,
                'input_topic': topic,
                'output_namespace':
                    namespace if len(topics) == 1 else f'{namespace}/{key}',
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=_setup)])
