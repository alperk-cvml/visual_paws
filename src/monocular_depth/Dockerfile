# syntax=docker/dockerfile:1.4
#
# monocular_depth — Depth Anything V2 metric depth inference container
#
# Base: NVIDIA CUDA 12.6 + cuDNN on Ubuntu 24.04
# Includes: ROS 2 Jazzy, PyTorch (CUDA), Depth Anything V2, pre-downloaded weights
#

FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04

ARG ROS_DISTRO=jazzy
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG MODEL_TYPE=vits

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    TZ=UTC \
    ROS_DISTRO=${ROS_DISTRO}

# ── System packages ──────────────────────────────────────────────────────
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt update && apt install -y --no-install-recommends \
      locales tzdata \
      curl gnupg2 lsb-release \
      sudo git bash-completion \
      build-essential cmake \
      python3-pip python3-venv \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=${LC_ALL} LANG=${LANG} \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 Jazzy ──────────────────────────────────────────────────────────
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt update && apt install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      python3-colcon-common-extensions \
      python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# ── PyTorch (CUDA 12.x wheels) ───────────────────────────────────────────
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    -U --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch torchvision

# ── Depth Anything V2 ────────────────────────────────────────────────────
# Clone the official repo for the model definition code
RUN git clone --depth 1 https://github.com/DepthAnything/Depth-Anything-V2.git /opt/depth_anything_v2 \
    && cd /opt/depth_anything_v2 \
    && python3 -m pip install --no-cache-dir --break-system-packages -e .

# ── Download metric depth weights at build time ──────────────────────────
# Outdoor (Virtual KITTI) model — max_depth=80m
RUN mkdir -p /models && \
    curl -sSL -o /models/depth_anything_v2_metric_vkitti_${MODEL_TYPE}.pth \
      "https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-VKITTI-Small/resolve/main/depth_anything_v2_metric_vkitti_${MODEL_TYPE}.pth"

# ── Non-root user ────────────────────────────────────────────────────────
RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}; fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /work/ws
USER ${USERNAME}

CMD ["bash"]
