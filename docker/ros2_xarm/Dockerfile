# Dockerfile for ROS2 Humble + xArm Python SDK
# Based on Ubuntu 22.04 (Jammy) - required for ROS2 Humble
# Unifies ROS2 rclpy and xarm-python-sdk on the same system Python 3.10
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONUNBUFFERED=1

# ============================================================================
# Base System
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
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ============================================================================
# Install ROS2 Humble (keyring method, matching official instructions)
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
        ros-humble-demo-nodes-cpp \
        ros-humble-demo-nodes-py \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# rosdep init / update (safe to fail if already initialized)
RUN rosdep init 2>/dev/null || true && \
    rosdep update || true

# ============================================================================
# Python dependencies — installed on the SAME system Python 3.10 that Humble's
# rclpy uses, so rclpy + xarm-python-sdk share one interpreter.
# ============================================================================
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir \
        xarm-python-sdk \
        pyserial \
        numpy \
        scipy \
        pyyaml \
        pytest

# ============================================================================
# ROS2 workspace skeleton
# ============================================================================
RUN mkdir -p /app/ros2_ws/src
WORKDIR /app/ros2_ws

# ============================================================================
# Copy docker configuration + smoke tests
# Build context is the repo root (../..) per docker-compose.yaml.
# ============================================================================
COPY docker/ros2_xarm/entrypoint.sh /entrypoint.sh
COPY docker/ros2_xarm/ros2_xarm.env /etc/ros2_xarm/ros2_xarm.env
COPY docker/ros2_xarm/smoke_test.sh /smoke_test.sh
COPY docker/ros2_xarm/tests /opt/ros2_xarm_tests
RUN chmod +x /entrypoint.sh /smoke_test.sh

# bashrc: source ROS2 + workspace + env, and set a distinct PS1
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /app/ros2_ws/install/setup.bash ]; then source /app/ros2_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /etc/ros2_xarm/ros2_xarm.env ]; then source /etc/ros2_xarm/ros2_xarm.env; fi" >> /root/.bashrc && \
    echo "export PS1='[ros2_xarm]\\u@\\h:\\w\\$ '" >> /root/.bashrc

# ============================================================================
# Build-time sanity check: rclpy + xarm on the same interpreter
# ============================================================================
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    python3 -c 'import rclpy; from xarm.wrapper import XArmAPI; import xarm; print(f\"import ok, xarm=={xarm.version.__version__}\")'"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
