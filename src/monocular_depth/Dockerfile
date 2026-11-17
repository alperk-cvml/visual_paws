# syntax=docker/dockerfile:1.4
#
# monocular_depth — Metric3D V2 metric depth inference container
#
# Base: NVIDIA CUDA 12.6 + cuDNN on Ubuntu 24.04
# Includes: ROS 2 Jazzy, PyTorch (CUDA), Metric3D V2 with weights pre-fetched
# at build time so the container never needs the network at runtime.
#
# The build context is the **repository root** (see this module's
# docker-compose.yaml): the Metric3D layers need the shared compat/mmcv shim,
# which lives outside this directory because depth_comparison uses it too.
#

FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04

ARG ROS_DISTRO=jazzy
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Metric3D hub entrypoint pre-fetched into the image cache.
ARG METRIC3D_MODEL=metric3d_vit_small

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    TZ=UTC \
    ROS_DISTRO=${ROS_DISTRO} \
    TORCH_HOME=/opt/torch_hub

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
      ros-${ROS_DISTRO}-rqt-image-view \
      python3-colcon-common-extensions \
      python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# ── PyTorch (CUDA 12.x wheels) ───────────────────────────────────────────
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    -U --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch torchvision

# ── Metric3D V2 dependencies ─────────────────────────────────────────────
# The numpy constraint matters. mmengine and timm both want a newer numpy, but
# numpy here comes from the Debian ROS packages and pip cannot uninstall it
# ("RECORD file not found"). Pinning it to whatever is already installed makes
# the requirement resolve against the existing copy instead of trying to
# replace it — and keeps cv_bridge on the numpy ABI it was compiled against.
RUN python3 -c "import numpy; print('numpy==' + numpy.__version__)" > /tmp/pip-constraints.txt \
    && cat /tmp/pip-constraints.txt \
    && PIP_CONSTRAINT=/tmp/pip-constraints.txt python3 -m pip install \
         --no-cache-dir --break-system-packages \
         mmengine timm scipy

# mmcv 1.x compatibility shim. Metric3D imports mmcv on its inference path, but
# mmcv 1.x will not install on Python 3.12 and mmcv 2.x dropped the mmcv.utils
# namespace entirely. See compat/mmcv/__init__.py for the full reasoning.
#
# Shared with the depth_comparison image; the build context is the repository
# root so both can reach it.
COPY compat/ /opt/metric3d_compat/
ENV PYTHONPATH="/opt/metric3d_compat:${PYTHONPATH}"

# ── Metric3D V2 weights (torch.hub) ──────────────────────────────────────
# Pre-fetch the repo and checkpoint into TORCH_HOME so runtime is offline.
# Unlike depth_comparison — where a broken backend is dropped and the other
# channels still run — Metric3D is the only depth source in this container, so
# a failed prefetch is fatal here rather than a warning.
RUN python3 -c "import torch; torch.hub.load('yvanyin/metric3d', '${METRIC3D_MODEL}', pretrain=True, trust_repo=True)" \
    && chmod -R a+rX /opt/torch_hub

# ── Non-root user ────────────────────────────────────────────────────────
RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}; fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /work/ws
USER ${USERNAME}

CMD ["bash"]
