import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_elevation_mapping_cupy')

    # Launch Arguments
    input_pointcloud_topic_arg = DeclareLaunchArgument(
        'input_pointcloud_topic', default_value='/depth_camera/points',
        description='Input point cloud topic'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use a ROS simulation clock'
    )

    elevation_mapping_cupy_share = get_package_share_directory('elevation_mapping_cupy')
    core_param_path = os.path.join(elevation_mapping_cupy_share, 'config', 'core', 'core_param.yaml')

    # Frame configuration (map_frame, base_frame, corrected_map_frame) is set
    # exclusively in elevation_mapping.yaml. Do NOT override it here — the
    # launch dict is the LAST parameter source and would always win, defeating
    # any YAML-driven frame configuration.
    elevation_mapping_config = os.path.join(pkg_share, 'config', 'elevation_mapping.yaml')
    elevation_mapping_node = Node(
        package='elevation_mapping_cupy',
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        output='screen',
        parameters=[
            core_param_path,
            elevation_mapping_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/depth_camera/points', LaunchConfiguration('input_pointcloud_topic'))
        ]
    )

    # Grid Map Visualization Node (for Occupancy Grid)
    grid_map_vis_config = os.path.join(pkg_share, 'config', 'grid_map_visualization.yaml')
    grid_map_vis_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[
            grid_map_vis_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        input_pointcloud_topic_arg,
        use_sim_time_arg,
        elevation_mapping_node,
        grid_map_vis_node
    ])

