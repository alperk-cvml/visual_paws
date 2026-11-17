"""Launch one scale-correction node per input point cloud topic.

Mirrors ground_plane.launch.py: `input_topics` is a comma-separated list and
each topic gets its own node. `scale_topics` pairs with it 1:1; a single
entry is shared by every input. Output topics default to
`<input_topic>_corrected` and can be overridden 1:1 via `output_topics`.

Example (depth comparison channels, each corrected by its own probe):

  input_topics:=/depth_compare/dav2_vkitti/points,/depth_compare/metric3d_s/points
  scale_topics:=/ground_plane/dav2_vkitti/scale,/ground_plane/metric3d_s/scale
"""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'input_topics', default_value='/depth_camera/points',
        description='Comma-separated PointCloud2 topics; one node each'),
    DeclareLaunchArgument(
        'scale_topics', default_value='/ground_plane/scale',
        description='Comma-separated Float32 scale-estimate topics; one per '
                    'input topic, or a single topic shared by all'),
    DeclareLaunchArgument(
        'shift_topics', default_value='',
        description='Optional comma-separated Float32 shift-estimate topics '
                    '(affine model only); same pairing rules as scale_topics'),
    DeclareLaunchArgument(
        'output_topics', default_value='',
        description='Optional comma-separated output topics; empty derives '
                    '`<input_topic>_corrected`'),
    DeclareLaunchArgument(
        'model', default_value='scalar',
        description='Depth error model: scalar (p/s) or affine ((z-b)/s '
                    'along each ray)'),
    DeclareLaunchArgument(
        'smoothing', default_value='ema',
        description='Estimate smoothing: none, ema, or median'),
    DeclareLaunchArgument(
        'ema_alpha', default_value='0.2',
        description='EMA weight of a new observation (smoothing:=ema)'),
    DeclareLaunchArgument(
        'median_window', default_value='5',
        description='Rolling window length (smoothing:=median)'),
    DeclareLaunchArgument(
        'estimate_timeout', default_value='5.0',
        description='Seconds before an estimate is stale; 0 disables'),
    DeclareLaunchArgument(
        'stale_policy', default_value='hold',
        description='Without a fresh estimate: hold, passthrough, or drop'),
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


def _paired(name: str, values: list[str], inputs: list[str]) -> list[str]:
    """Pair a topic list with the inputs: 1:1, one-for-all, or empty."""
    if not values:
        return [''] * len(inputs)
    if len(values) == 1:
        return values * len(inputs)
    if len(values) != len(inputs):
        raise RuntimeError(
            f'`{name}` has {len(values)} entries for {len(inputs)} '
            f'input topics; give one per input, a single shared one, '
            f'or none')
    return values


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def topic_list(name):
        return [t.strip() for t in arg(name).split(',') if t.strip()]

    inputs = topic_list('input_topics')
    if not inputs:
        raise RuntimeError('`input_topics` resolved to an empty list.')
    scales = _paired('scale_topics', topic_list('scale_topics'), inputs)
    shifts = _paired('shift_topics', topic_list('shift_topics'), inputs)
    outputs = _paired('output_topics', topic_list('output_topics'), inputs)

    common = {
        'model': arg('model'),
        'smoothing': arg('smoothing'),
        'ema_alpha': float(arg('ema_alpha')),
        'median_window': int(arg('median_window')),
        'estimate_timeout': float(arg('estimate_timeout')),
        'stale_policy': arg('stale_policy'),
        'use_sim_time': arg('use_sim_time').lower() in ('true', '1'),
    }

    nodes = []
    for input_topic, scale, shift, output in zip(
            inputs, scales, shifts, outputs):
        nodes.append(Node(
            package='pointcloud_scale_correction',
            executable='scale_correction',
            name=f'scale_correction_{_topic_key(input_topic)}',
            output='screen',
            parameters=[{
                **common,
                'input_topic': input_topic,
                'scale_topic': scale,
                'shift_topic': shift,
                'output_topic': output,
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=_setup)])
