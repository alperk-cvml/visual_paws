ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y \
    bash-completion \
    python3-numpy \
    python3-dev \
    python3-pip \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# kiss-icp is pip-only (no rosdep key). Pin numpy to the Debian ROS version
# via PIP_CONSTRAINT so pip does not pull a numpy 2.x that the apt-installed
# ROS Python stack was not built against (same trick as monocular_depth).
RUN python3 -c "import numpy; print('numpy==' + numpy.__version__)" > /tmp/pip-constraints.txt \
    && cat /tmp/pip-constraints.txt \
    && PIP_CONSTRAINT=/tmp/pip-constraints.txt python3 -m pip install \
       --no-cache-dir --break-system-packages \
       "kiss-icp>=1.2,<2"

WORKDIR /work/ws
