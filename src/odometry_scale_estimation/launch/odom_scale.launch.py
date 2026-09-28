"""Launch one odometry scale estimator per estimate odometry topic.

A single estimate topic publishes directly under `output_namespace`. With
several topics (e.g. KISS-ICP instances riding on different depth backends)
each estimator gets its own `<output_namespace>/<key>/*` subtree, where
<key> is derived from the topic (`/odom_lidar` -> `odom_lidar`), so the
per-source scales can be plotted side by side. The reference odometry is
shared by every node.
"""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'reference_topic', default_value='/odom_lko',
        description='Metric-truth nav_msgs/Odometry (leg kinematics + IMU); '
                    'shared by all estimators'),
    DeclareLaunchArgument(
        'estimate_topics', default_value='/odom_lidar',
        description='Comma-separated scale-bearing nav_msgs/Odometry topics '
                    '(e.g. KISS-ICP on the monodepth cloud); one node each'),
    DeclareLaunchArgument(
        'output_namespace', default_value='/odom_scale',
        description='Parent namespace for scale/shift/residual/ref_motion/'
                    'marker'),
    DeclareLaunchArgument(
        'model', default_value='scalar',
        description='Fit model: scalar (chord_est = s·chord_ref) or affine '
                    '(chord_est = s·chord_ref + b)'),
    DeclareLaunchArgument(
        'window_duration', default_value='5.0',
        description='Sliding comparison window (s)'),
    DeclareLaunchArgument(
        'min_motion', default_value='0.1',
        description='Minimum reference chord (m) in the window before a fit '
                    'is attempted — a standing robot yields pure noise'),
    DeclareLaunchArgument(
        'publish_rate', default_value='2.0',
        description='Output rate (Hz); each cycle re-fits the whole window'),
    DeclareLaunchArgument(
        'time_offset', default_value='0.0',
        description='Constant stamp lag of the estimate stream (s), added '
                    'to its stamps before alignment'),
    DeclareLaunchArgument(
        'marker_frame', default_value='odom',
        description='Frame the reference odometry poses live in (RViz '
                    'trajectory overlay)'),
    DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Robot body frame the motion-vector arrows originate '
                    'from (RViz marker only)'),
    DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use a ROS simulation clock'),
]


def _topic_key(topic: str) -> str:
    """`/depth_compare/metric3d_s/odom` -> `metric3d_s`; `/odom_lidar` stays."""
    parts = [s for s in topic.split('/') if s]
    if len(parts) > 1 and parts[-1] in ('odom', 'odometry'):
        parts = parts[:-1]
    return re.sub(r'\W', '_', parts[-1])


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    topics = [t.strip() for t in arg('estimate_topics').split(',') if t.strip()]
    if not topics:
        raise RuntimeError('`estimate_topics` resolved to an empty list.')

    namespace = arg('output_namespace').rstrip('/')
    common = {
        'reference_topic': arg('reference_topic'),
        'model': arg('model'),
        'window_duration': float(arg('window_duration')),
        'min_motion': float(arg('min_motion')),
        'publish_rate': float(arg('publish_rate')),
        'time_offset': float(arg('time_offset')),
        'marker_frame': arg('marker_frame'),
        'base_frame': arg('base_frame'),
        'use_sim_time': arg('use_sim_time').lower() in ('true', '1'),
    }

    nodes = []
    for topic in topics:
        key = _topic_key(topic)
        nodes.append(Node(
            package='odometry_scale_estimation',
            executable='odom_scale',
            name=f'odom_scale_{key}',
            output='screen',
            parameters=[{
                **common,
                'estimate_topic': topic,
                'output_namespace':
                    namespace if len(topics) == 1 else f'{namespace}/{key}',
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=_setup)])
