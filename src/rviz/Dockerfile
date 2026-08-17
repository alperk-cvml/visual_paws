# syntax=docker/dockerfile:1.4

ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

SHELL ["/bin/bash", "-lc"]

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt update && apt install -y --no-install-recommends \
      ros-${ROS_DISTRO}-grid-map-rviz-plugin \
      ros-${ROS_DISTRO}-nav2-rviz-plugins \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work/ws

# Copy default RViz configuration so it is built into the image as a fallback
COPY config/visual_paws.rviz /opt/rviz/config/visual_paws.rviz

CMD ["bash"]
