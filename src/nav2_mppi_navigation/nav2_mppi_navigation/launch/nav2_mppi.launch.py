"""
nav2_mppi.launch.py
-------------------
Launches Nav2 with the MPPI controller, consuming the occupancy map from
/occupancy_grid (published by simple_elevation_mapping_cupy).

Localization: fake (static map->odom TF). No AMCL / laser scan required.
Goal input:   RViz2 Nav2 panel ("2D Nav Goal" button).
Output:       /cmd_vel (nav_msgs/Twist)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_mppi_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ── Launch arguments ─────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from simulation')
    declare_map_frame = DeclareLaunchArgument(
        'map_frame', default_value='map',
        description='Global map frame')
    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame', default_value='odom',
        description='Odometry frame')
    declare_base_frame = DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Robot base frame')
    declare_occ_topic = DeclareLaunchArgument(
        'occupancy_grid_topic', default_value='/occupancy_grid',
        description='OccupancyGrid topic from elevation_mapping_cupy')

    params_file = os.path.join(pkg_share, 'config', 'nav2_mppi_params.yaml')
    configured_params = ReplaceString(
        source_file=params_file,
        replacements={
            '<map_frame>': LaunchConfiguration('map_frame'),
            '<odom_frame>': LaunchConfiguration('odom_frame'),
            '<base_frame>': LaunchConfiguration('base_frame'),
            '<occupancy_grid_topic>': LaunchConfiguration(
                'occupancy_grid_topic'),
        },
    )

    # ── Nav2 navigation stack (no map_server; costmap reads /occupancy_grid) ─
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': configured_params,
            'use_composition': 'False',
            'use_respawn': 'False',
            'use_collision_monitor': 'False',
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_frame,
        declare_odom_frame,
        declare_base_frame,
        declare_occ_topic,
        nav2_launch,
    ])
