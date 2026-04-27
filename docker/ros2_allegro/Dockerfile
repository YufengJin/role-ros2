# Dockerfile for ROS2 Humble + Allegro Hand V4 (official ros2_control driver)
#
# This image bundles:
#   - ROS 2 Humble (Ubuntu 22.04, Python 3.10)
#   - The full ros2_control / ros2_controllers stack required by allegro_hand_ros2
#   - Build deps for allegro_hand_grasp_library (pinocchio, spdlog, yaml-cpp, boost)
#   - SocketCAN userspace tools (can-utils, iproute2) — host owns the can0 interface,
#     container shares it via network_mode: host (no /dev passthrough required).
#
# The Allegro driver itself (allegro_hand_ros2/) and the role-ros2 wrapper are
# NOT baked into the image — they are built from the bind-mounted workspace
# (/app/ros2_ws/src/role-ros2) by the entrypoint's `colcon build`.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONUNBUFFERED=1

# ============================================================================
# Base system
# ============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        build-essential \
        sudo \
        git \
        curl \
        wget \
        ca-certificates \
        gnupg2 \
        lsb-release \
        locales \
        iputils-ping \
        iproute2 \
        net-tools \
        pkg-config \
        python3-pip \
        python3-dev \
        can-utils \
        kmod \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ============================================================================
# ROS 2 Humble
# ============================================================================
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-desktop \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-rmw-cyclonedds-cpp \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && \
    rosdep update || true

# ============================================================================
# ros2_control + controllers + helpers (everything the Allegro driver needs)
# ============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-controller-manager \
        ros-humble-joint-state-broadcaster \
        ros-humble-forward-command-controller \
        ros-humble-imu-sensor-broadcaster \
        ros-humble-position-controllers \
        ros-humble-effort-controllers \
        ros-humble-diagnostic-aggregator \
        ros-humble-diagnostic-updater \
        ros-humble-realtime-tools \
        ros-humble-control-toolbox \
        ros-humble-rclcpp-action \
        ros-humble-pinocchio \
    && rm -rf /var/lib/apt/lists/*

# allegro_hand_grasp_library system deps
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libspdlog-dev \
        libyaml-cpp-dev \
        libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Python deps (system Python 3.10 — same interpreter as Humble's rclpy)
# ============================================================================
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir \
        numpy \
        scipy \
        pyyaml \
        pytest

# ============================================================================
# ROS 2 workspace skeleton (sources are bind-mounted by docker-compose)
# ============================================================================
RUN mkdir -p /app/ros2_ws/src
WORKDIR /app/ros2_ws

# ============================================================================
# Container scripts
# Build context is the repo root (../..) per docker-compose.yaml.
# ============================================================================
COPY docker/ros2_allegro/entrypoint.sh /entrypoint.sh
COPY docker/ros2_allegro/ros2_allegro.env /etc/ros2_allegro/ros2_allegro.env
COPY docker/ros2_allegro/smoke_test.sh /smoke_test.sh
RUN chmod +x /entrypoint.sh /smoke_test.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /app/ros2_ws/install/setup.bash ]; then source /app/ros2_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /etc/ros2_allegro/ros2_allegro.env ]; then source /etc/ros2_allegro/ros2_allegro.env; fi" >> /root/.bashrc && \
    echo "export PS1='[ros2_allegro]\\u@\\h:\\w\\$ '" >> /root/.bashrc

# Build-time sanity check: ros2_control_node binary present + rclpy importable
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    python3 -c 'import rclpy' && \
    which ros2_control_node && \
    which ros2"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
