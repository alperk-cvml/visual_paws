import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("mechdog_gazebo")

    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    world_default = os.path.join(pkg_share, "worlds", "empty.sdf")
    xacro_file = os.path.join(pkg_share, "urdf", "mechdog.urdf.xacro")

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments={
            # IMPORTANT: no -s here
            "gz_args": [' -r -v 4 ', world],
            "on_exit_shutdown": "true",
        }.items(),
    )
    # gz_server = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare("ros_gz_sim"),
    #             "launch",
    #             "gz_server.launch.py",
    #         ])
    #     ),
    #     launch_arguments={
    #         "world_sdf_file": world,
    #     }.items(),
    # )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", robot_name,
            "-topic", "robot_description",
            "-x", x,
            "-y", y,
            "-z", z,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world_default),
        DeclareLaunchArgument("robot_name", default_value="mechdog"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.35"),
        rsp,
        #gz_server,
        gz_sim,
        TimerAction(period=8.0, actions=[spawn]),
    ])
