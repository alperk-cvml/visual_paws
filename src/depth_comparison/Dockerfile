# syntax=docker/dockerfile:1.4
#
# depth_comparison — multi-model metric depth comparison container
#
# Base: NVIDIA CUDA 12.6 + cuDNN on Ubuntu 24.04 (x86_64 development host).
# On the Jetson target, swap the base for dustynv/ros:jazzy-pytorch-l4t-r36.4.0
# and drop the PyTorch install below — that image ships PyTorch + CUDA already.
#
# Includes: ROS 2 Jazzy, depth_image_proc, PyTorch, Depth Anything V2 (both
# metric checkpoints) and Metric3D V2, with all weights baked in at build time
# so the container never needs the network at runtime.
#

FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04

ARG ROS_DISTRO=jazzy
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Encoder variant for the Depth Anything V2 checkpoints (vits | vitb | vitl).
ARG DAV2_ENCODER=vits
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
# depth_image_proc is included here so the point cloud fan-out runs in the
# same container as the comparison node.
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
      ros-${ROS_DISTRO}-depth-image-proc \
      ros-${ROS_DISTRO}-rqt-image-view \
      python3-colcon-common-extensions \
      python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# ── PyTorch (CUDA 12.x wheels) ───────────────────────────────────────────
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    -U --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch torchvision

# ── Depth Anything V2 ────────────────────────────────────────────────────
# No setup.py upstream — the metric_depth/ variant goes on PYTHONPATH. That is
# the one whose DPT class accepts max_depth.
RUN git clone --depth 1 https://github.com/DepthAnything/Depth-Anything-V2.git \
      /opt/depth_anything_v2
ENV PYTHONPATH="/opt/depth_anything_v2/metric_depth:${PYTHONPATH}"

# ── Depth Anything V2 metric weights ─────────────────────────────────────
# VKITTI (outdoor/driving, 80 m) and Hypersim (indoor, 20 m).
#
# Upstream splits these across one HuggingFace repo per encoder size, so the
# repo name has to track DAV2_ENCODER — asking the -Small repo for a vitb file
# is a 404, not a fallback.
RUN mkdir -p /models \
    && case "${DAV2_ENCODER}" in \
         vits) DAV2_SIZE=Small ;; \
         vitb) DAV2_SIZE=Base  ;; \
         vitl) DAV2_SIZE=Large ;; \
         *) echo "unsupported DAV2_ENCODER=${DAV2_ENCODER}" >&2; exit 1 ;; \
       esac \
    && for dataset in vkitti hypersim; do \
         case "$dataset" in \
           vkitti)   repo_dataset=VKITTI   ;; \
           hypersim) repo_dataset=Hypersim ;; \
         esac; \
         curl -fsSL -o "/models/depth_anything_v2_metric_${dataset}_${DAV2_ENCODER}.pth" \
           "https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-${repo_dataset}-${DAV2_SIZE}/resolve/main/depth_anything_v2_metric_${dataset}_${DAV2_ENCODER}.pth" \
         || exit 1; \
       done \
    && chmod -R a+rX /models

# ── Metric3D V2 (torch.hub) ──────────────────────────────────────────────
# Deliberately the last model layer: only the metric3d_v2 backend needs any of
# this, so a dependency problem here cannot invalidate the Depth Anything V2
# layers above it.
#
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
COPY compat/ /opt/metric3d_compat/
ENV PYTHONPATH="/opt/metric3d_compat:${PYTHONPATH}"

# Pre-fetch the repo and checkpoint into TORCH_HOME so runtime is offline.
#
# Non-fatal on purpose. Metric3D's mono/utils/comm.py imports mmcv 1.x
# unconditionally — mmengine only satisfies hubconf.py — and mmcv 1.x has to be
# compiled against this exact torch/CUDA build. Until that is resolved the
# metric3d_v2 backend is unavailable, the node drops it at startup with an
# error, and the Depth Anything V2 channels are unaffected. Failing the image
# here instead would block Steps 1 and 2 on a Step 3 dependency.
RUN python3 -c "import torch; torch.hub.load('yvanyin/metric3d', '${METRIC3D_MODEL}', pretrain=True, trust_repo=True)" \
      && touch /opt/torch_hub/.metric3d_prefetch_ok \
      || echo "WARNING: Metric3D prefetch failed — the metric3d_v2 backend will be unavailable at runtime." >&2; \
    chmod -R a+rX /opt/torch_hub

# ── Non-root user ────────────────────────────────────────────────────────
RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}; fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /work/ws
USER ${USERNAME}

CMD ["bash"]
