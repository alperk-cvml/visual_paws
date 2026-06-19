# syntax=docker/dockerfile:1.4
#
# video_camera_sim — lightweight CPU-only container that plays back
# pre-recorded video files as ROS 2 camera topics.
#
# Base: ROS 2 Jazzy on Ubuntu 24.04 (CPU-only, no CUDA needed)
#

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

# ── System packages ──────────────────────────────────────────────────────
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt update && apt install -y --no-install-recommends \
      locales tzdata \
      python3-pip \
      python3-opencv \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=${LC_ALL} LANG=${LANG} \
    && rm -rf /var/lib/apt/lists/*

# ── Non-root user ────────────────────────────────────────────────────────
RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}; fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /work/ws
USER ${USERNAME}

CMD ["bash"]
