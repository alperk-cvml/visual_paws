import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mechdog_gazebo")

    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    world_default = os.path.join(pkg_share, "worlds", "empty.sdf")
    xacro_file = os.path.join(pkg_share, "urdf", "mechdog.urdf.xacro")

    declare_args = [
        DeclareLaunchArgument("world", default_value=world_default),
        DeclareLaunchArgument("robot_name", default_value="mechdog"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.35"),
    ]

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Start Gazebo Sim (GUI by default)
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", world],
        output="screen",
    )

    # Spawn from /robot_description via ros_gz_sim
    spawn = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-name", robot_name,
            "-topic", "robot_description",
            "-x", x,
            "-y", y,
            "-z", z,
        ],
        output="screen",
    )

    # Give Gazebo a moment to come up before spawning
    spawn_delayed = TimerAction(period=3.0, actions=[spawn])

    ld = LaunchDescription()
    for a in declare_args:
        ld.add_action(a)

    ld.add_action(rsp)
    ld.add_action(gz_sim)
    ld.add_action(spawn_delayed)
    return ld
