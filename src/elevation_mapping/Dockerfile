ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y \
    bash-completion \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-grid-map \
    ros-${ROS_DISTRO}-grid-map-ros \
    ros-${ROS_DISTRO}-grid-map-pcl \
    ros-${ROS_DISTRO}-grid-map-rviz-plugin \
    python3-colcon-common-extensions \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work/ws
