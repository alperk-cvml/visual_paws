ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# GIDs for /dev/dri (Mesa GPU rendering, group-owned video:render on the host)
# so the non-root user can still open the Gazebo GUI's render nodes.
ARG VIDEO_GID=44
ARG RENDER_GID=109

SHELL ["/bin/bash", "-lc"]

# Gazebo Classic (commonly used with ROS2 Humble via gazebo_ros_pkgs)
RUN apt-get update && apt-get install -y \
    bash-completion \
    sudo \
    python3-numpy \
    python3-dev \
    python3-pil \
    python3-yaml \
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

# Non-root user matching host UID/GID, plus video/render supplementary groups
# so it can still open /dev/dri for GPU-accelerated Gazebo GUI rendering.
RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! getent group ${VIDEO_GID} >/dev/null; then groupadd --gid ${VIDEO_GID} video_host; fi \
    && if ! getent group ${RENDER_GID} >/dev/null; then groupadd --gid ${RENDER_GID} render_host; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}; fi \
    # usermod -aG (not the useradd -G above) so pre-existing base-image users
    # — e.g. Ubuntu 24.04 cloud images already ship a UID-1000 "ubuntu" user —
    # still pick up the video/render groups needed for /dev/dri.
    && usermod -aG $(getent group ${VIDEO_GID} | cut -d: -f1),$(getent group ${RENDER_GID} | cut -d: -f1) ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /work/ws
USER ${USERNAME}
