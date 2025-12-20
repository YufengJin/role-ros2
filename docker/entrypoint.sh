#!/bin/bash
set -e

# Fix git safe directory for Polymetis version detection
git config --global --add safe.directory /app/ros2_ws/src/role-ros2/role_ros2/fairo 2>/dev/null || true
git config --global --add safe.directory /app/ros2_ws/src/role-ros2/role_ros2/fairo/polymetis 2>/dev/null || true
git config --global --add safe.directory /app/ros2_ws/src/role-ros2 2>/dev/null || true

# Source ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Source ROS2 workspace if it exists
if [ -f "/app/ros2_ws/install/setup.bash" ]; then
    source /app/ros2_ws/install/setup.bash
fi

# Initialize and activate micromamba environment
eval "$(micromamba shell hook --shell bash)"
micromamba activate polymetis-local

# Set library paths - prioritize system spdlog over conda libraries
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/foxy/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/openrobots/lib:$CONDA_PREFIX/lib

# Set environment variables
export ROBOT_IP=${ROBOT_IP:-172.17.0.2}
export POLYMETIS_IP=${POLYMETIS_IP:-127.0.0.1}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Set DDS config based on RMW implementation
if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then
    CYCLONE_CONFIG="/app/ros2_ws/src/role-ros2/docker/cyclonedds.xml"
    if [ -f "$CYCLONE_CONFIG" ]; then
        export CYCLONEDDS_URI="file://$CYCLONE_CONFIG"
    fi
elif [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ]; then
    PROFILE_PATH="/app/ros2_ws/src/role-ros2/docker/fastrtps_profile.xml"
    if [ -f "$PROFILE_PATH" ]; then
        export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE_PATH"
    fi
fi

# ROS2 logging settings
export RCUTILS_LOGGING_SEVERITY=INFO
export RCUTILS_LOGGING_USE_STDOUT=1

# Execute command passed to container
exec "$@"
