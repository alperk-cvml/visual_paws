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
        'input_pointcloud_topic', default_value='/depth_camera/depth_image/points',
        description='Input point cloud topic'
    )
    map_frame_id_arg = DeclareLaunchArgument(
        'map_frame_id', default_value='odom',
        description='Map frame id'
    )
    base_frame_id_arg = DeclareLaunchArgument(
        'base_frame_id', default_value='base_link',
        description='Base frame id'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use a ROS simulation clock'
    )

    elevation_mapping_cupy_share = get_package_share_directory('elevation_mapping_cupy')
    core_param_path = os.path.join(elevation_mapping_cupy_share, 'config', 'core', 'core_param.yaml')

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
                'map_frame': LaunchConfiguration('map_frame_id'),
                'base_frame': LaunchConfiguration('base_frame_id'),
                'corrected_map_frame': LaunchConfiguration('map_frame_id'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/camera/depth/points', LaunchConfiguration('input_pointcloud_topic'))
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
        map_frame_id_arg,
        base_frame_id_arg,
        use_sim_time_arg,
        elevation_mapping_node,
        grid_map_vis_node
    ])
