"""
Launch file for quadruped robot — ROS 2 Jazzy + Gazebo Harmonic (gz-sim)


Usage:
    ros2 launch quadruped_gazebo gazebo.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = "quadruped_gazebo"
    pkg_share = get_package_share_directory(pkg_name)

    # Resolve world path at launch-description build time (plain Python string)
    # This avoids Gazebo trying to treat it as a Fuel URI.
    world_path = os.path.join(pkg_share, "worlds", "quadruped_world.sdf")

    # ── Launch arguments ──────────────────────────────────────────
    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_x = DeclareLaunchArgument("x_pose", default_value="0.0")
    declare_y = DeclareLaunchArgument("y_pose", default_value="0.0")
    declare_z = DeclareLaunchArgument("z_pose", default_value="0.35")
    declare_localization_mode = DeclareLaunchArgument(
        "localization_mode",
        default_value="ground_truth",
        choices=["ground_truth", "slam"],
        description=(
            "ground_truth publishes map->odom and odom->base_link; "
            "slam publishes only /odom_ground_truth for the localization EKFs"
        ),
    )

    # ── Robot description ─────────────────────────────────────────
    xacro_file = os.path.join(pkg_share, "urdf", "quadruped.urdf.xacro")
    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    # ── Robot State Publisher ─────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ── Gazebo Harmonic ───────────────────────────────────────────
    # gz_args: -r = run immediately (not paused)
    # Pass world_path as a plain Python string so gz receives the real
    # filesystem path, not a Fuel download URI.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_path}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── Spawn robot ───────────────────────────────────────────────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_quadruped",
        arguments=[
            "-name", "quadruped",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x_pose"),
            "-y", LaunchConfiguration("y_pose"),
            "-z", LaunchConfiguration("z_pose"),
        ],
        output="screen",
    )

    # ── ROS <-> Gazebo topic bridge ───────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros_bridge",
        arguments=[
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/front_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/quadruped/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[
            ("/model/quadruped/pose", "/gazebo/model_pose"),
        ],
        output="screen",
    )

    # ── Load ros2_control controllers ────────────────────────────
    # Delay gives gz_ros2_control time to register the hardware interfaces
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )
    load_joint_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "quadruped_joint_controller",
        ],
        output="screen",
    )

    # Chain: spawn exits → wait 3s → broadcaster → joint controller
    load_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=3.0, actions=[load_joint_state_broadcaster])
            ],
        )
    )
    load_joint_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_controller],
        )
    )

    # Gazebo supplies continuous ground-truth odometry. In ground_truth mode
    # this node also owns odom->base_link. In slam mode only the odometry topic
    # is published, leaving that TF to the local EKF.
    ground_truth_odometry = Node(
        package="quadruped_gazebo",
        executable="gazebo_ground_truth_odometry.py",
        name="gazebo_ground_truth_odometry",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "publish_tf": ParameterValue(
                PythonExpression([
                    "'", LaunchConfiguration("localization_mode"),
                    "' == 'ground_truth'",
                ]),
                value_type=bool,
            ),
        }],
    )

    # Phase 1 global frame. The global EKF owns map->odom in slam mode.
    map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("localization_mode"),
            "' == 'ground_truth'",
        ])),
    )

    return LaunchDescription(
        [
        declare_sim_time,
        declare_x,
        declare_y,
        declare_z,
        declare_localization_mode,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        load_after_spawn,
        load_joint_after_broadcaster,
        ground_truth_odometry,
        map_tf,
        ]
    )
