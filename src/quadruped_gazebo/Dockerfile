ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

SHELL ["/bin/bash", "-lc"]

# Gazebo Classic (commonly used with ROS2 Humble via gazebo_ros_pkgs)
RUN apt-get update && apt-get install -y \
    bash-completion \
    python3-numpy \
    python3-dev \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-grid-map \
    ros-${ROS_DISTRO}-grid-map-ros \
    ros-${ROS_DISTRO}-grid-map-rviz-plugin \
    python3-colcon-common-extensions \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work/ws

