# syntax=docker/dockerfile:1.4
#
# nav2_mppi_navigation — ROS 2 Jazzy + Nav2 MPPI Controller
# CPU-only image; no GPU required.

FROM ros:jazzy-ros-base

ARG ROS_DISTRO=jazzy
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    TZ=UTC \
    ROS_DISTRO=${ROS_DISTRO}

# System utilities
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt update && apt install -y --no-install-recommends \
      locales tzdata \
      sudo git bash-completion \
      build-essential cmake \
      python3-pip \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=${LC_ALL} LANG=${LANG} \
    && rm -rf /var/lib/apt/lists/*

# Nav2 + RViz2 + CycloneDDS
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt update && apt install -y --no-install-recommends \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-nav2-mppi-controller \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-nav2-rviz-plugins \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-ament-cmake \
      ros-${ROS_DISTRO}-ament-cmake-python \
      python3-colcon-common-extensions \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Non-root user matching host UID/GID
RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}; fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

RUN rosdep init || true

WORKDIR /work/ws
USER ${USERNAME}

CMD ["bash"]
