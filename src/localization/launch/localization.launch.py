"""
Launch file for the Visual Paws localization stack.

Starts:
  1. orbslam3_rgbd_node  — ORB-SLAM3 visual SLAM (RGBD mode)
  2. ekf_node            — robot_localization EKF sensor fusion
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('localization')

    # ── Launch arguments ─────────────────────────────────────────────────
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/front_camera/image_raw',
        description='Input RGB image topic')

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/depth_camera/depth_image',
        description='Input depth image topic')

    voc_file_arg = DeclareLaunchArgument(
        'voc_file',
        default_value='/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt',
        description='Path to ORB-SLAM3 vocabulary file')

    settings_file_arg = DeclareLaunchArgument(
        'settings_file',
        default_value='',
        description='Path to ORB-SLAM3 camera settings YAML (dynamically resolved if empty)')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='MONOCULAR',
        description='ORB-SLAM3 tracking mode (MONOCULAR or RGBD)')

    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='false',
        description='Enable ORB-SLAM3 Pangolin viewer (requires display)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock')

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID')

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odom frame ID')

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base link frame ID')

    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='ground_truth',
        choices=['ground_truth', 'slam', 'offline_video'],
        description=(
            'ground_truth leaves TF to Gazebo; slam enables the local and '
            'global EKFs; offline_video makes ORB-SLAM own map->base_link'))

    # ── ORB-SLAM3 Node ──────────────────────────────────────────────────
    orbslam3_node = Node(
        package='localization',
        executable='orbslam3_rgbd_node',
        name='orbslam3_rgbd_node',
        output='screen',
        parameters=[{
            'voc_file': LaunchConfiguration('voc_file'),
            'settings_file': LaunchConfiguration('settings_file'),
            'mode': LaunchConfiguration('mode'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'enable_viewer': LaunchConfiguration('enable_viewer'),
            'publish_tf': ParameterValue(
                PythonExpression([
                    "'", LaunchConfiguration('localization_mode'),
                    "' == 'offline_video'",
                ]),
                value_type=bool,
            ),
            'map_frame': LaunchConfiguration('map_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            # ORB optical axes are converted to a ROS-aligned camera pose in
            # the node; this frame supplies the remaining camera lever arm.
            'camera_frame': 'camera_link',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Offline video has no Gazebo/robot_state_publisher. Supply only the
    # camera mount extrinsics; ORB-SLAM supplies the moving map->base_link TF.
    offline_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='offline_base_to_camera',
        arguments=[
            '--x', '0.125', '--y', '0.0', '--z', '0.01375',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', 'camera_link',
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('localization_mode'),
            "' == 'offline_video'",
        ])),
    )
    offline_camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='offline_camera_to_optical',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5707963267948966',
            '--pitch', '0.0',
            '--yaw', '-1.5707963267948966',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_link_optical',
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('localization_mode'),
            "' == 'offline_video'",
        ])),
    )

    # The local filter owns odom->base_link. Gazebo ground truth is a
    # temporary continuous odometry input until /odom_lko is implemented.
    local_ekf_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_filter',
        output='screen',
        parameters=[
            local_ekf_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('localization_mode'), "' == 'slam'",
        ])),
    )

    # The global filter consumes the local estimate plus ORB-SLAM's absolute
    # map pose. With world_frame=map it owns map->odom, never base_link.
    global_ekf_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_filter',
        output='screen',
        parameters=[
            global_ekf_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/global'),
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('localization_mode'), "' == 'slam'",
        ])),
    )

    return LaunchDescription([
        # Arguments
        rgb_topic_arg,
        depth_topic_arg,
        voc_file_arg,
        settings_file_arg,
        mode_arg,
        enable_viewer_arg,
        use_sim_time_arg,
        map_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        localization_mode_arg,

        # Nodes
        offline_base_to_camera,
        offline_camera_to_optical,
        orbslam3_node,
        local_ekf_node,
        global_ekf_node,
    ])
