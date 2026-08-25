# syntax=docker/dockerfile:1.4
#
# localization — ORB-SLAM3 RGBD + robot_localization EKF
#
# Builds ORB-SLAM3 from source with all dependencies (Pangolin, DBoW2, g2o).
# Includes robot_localization for EKF sensor fusion.
#
# Base: ros:jazzy-ros-base (Ubuntu 24.04)
# NOTE: First build takes ~15-20 min due to ORB-SLAM3 compilation. Subsequent
# builds are cached.

FROM ros:jazzy-ros-base AS base

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
      sudo git bash-completion \
      build-essential cmake \
      python3-pip python3-colcon-common-extensions \
      # ORB-SLAM3 dependencies
      libeigen3-dev \
      libopencv-dev \
      libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libepoxy-dev \
      libsuitesparse-dev \
      libboost-serialization-dev \
      libssl-dev \
      wget curl \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=${LC_ALL} LANG=${LANG} \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 packages (cv_bridge, tf2, robot_localization, etc.) ────────────
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt update && apt install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# ── Build Pangolin ───────────────────────────────────────────────────────
RUN git clone --depth 1 --branch v0.9.2 \
      https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin \
    && mkdir -p /tmp/Pangolin/build && cd /tmp/Pangolin/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTS=OFF \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/Pangolin

# ── Build ORB-SLAM3 ─────────────────────────────────────────────────────
# Use AeroTec-ATLAS fork — specifically maintained for Ubuntu 24.04 / GCC 13.
# Additional sed patches for any remaining GCC 13 false positives.
RUN git clone --depth 1 \
      https://github.com/AeroTec-ATLAS/ORB-SLAM3-STEREO-FIXED-for-ubuntu-24.04-LTS.git \
      /opt/ORB_SLAM3 \
    && cd /opt/ORB_SLAM3 \
    # ── Extra safety patches ────────────────────────────────────────────
    # Suppress GCC 13 false-positive -Werror promotions in root CMakeLists
    && sed -i 's|-Wall|-Wall -Wno-error|g' CMakeLists.txt \
    # Suppress in bundled g2o CMakeLists as well
    && sed -i 's|-Wall|-Wall -Wno-error|g' Thirdparty/g2o/CMakeLists.txt \
    # Fix Eigen version requirement in bundled g2o (if not already fixed)
    && sed -i 's/FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)/FIND_PACKAGE(Eigen3 REQUIRED)/' \
        Thirdparty/g2o/CMakeLists.txt \
    # Fix C++17: bool++ is forbidden — replace with int flag (if not already fixed)
    && sed -i 's/bMatch++;/bMatch = 1;/g' src/LoopClosing.cc 2>/dev/null || true \
    # ── Build DBoW2 ─────────────────────────────────────────────────────
    && cd /opt/ORB_SLAM3/Thirdparty/DBoW2 \
    && mkdir -p build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wno-error" \
    && make -j$(nproc) \
    # ── Build g2o ────────────────────────────────────────────────────────
    && cd /opt/ORB_SLAM3/Thirdparty/g2o \
    && mkdir -p build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wno-error" \
    && make -j$(nproc) \
    # ── Build Sophus (header-only library — skip tests) ──────────────────
    # GCC 13 triggers false-positive -Werror=array-bounds on SSE intrinsics
    # in Sophus tests. Sophus is header-only so we only need cmake install.
    && cd /opt/ORB_SLAM3/Thirdparty/Sophus \
    && mkdir -p build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
    && make -j$(nproc) install || true \
    # ── Build ORB-SLAM3 main library ─────────────────────────────────────
    && cd /opt/ORB_SLAM3 \
    && mkdir -p build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="-O3 -Wno-error -Wno-deprecated-declarations -Wno-maybe-uninitialized -Wno-array-bounds" \
    && make -j$(nproc) \
    && ldconfig

# ── Download ORB vocabulary ─────────────────────────────────────────────
# The vocabulary file is ~40 MB compressed, ~140 MB uncompressed
RUN cd /opt/ORB_SLAM3/Vocabulary \
    && if [ ! -f ORBvoc.txt ]; then \
         if [ -f ORBvoc.txt.tar.gz ]; then \
           tar xzf ORBvoc.txt.tar.gz; \
         else \
           echo "WARNING: ORBvoc.txt not found. Will need to be mounted."; \
         fi; \
       fi

# ── Non-root user ────────────────────────────────────────────────────────
RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}; fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /work/ws
USER ${USERNAME}

CMD ["bash"]
