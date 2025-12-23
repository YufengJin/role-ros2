#!/bin/bash
set -e

# Entrypoint script for ROS2 Humble with CUDA 11.8 Docker container
# This script automatically sources ROS2 and workspace environment

# Source ROS2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "⚠️  Warning: ROS2 Humble not found at /opt/ros/humble/setup.bash"
fi

# Source ROS2 workspace if it exists
if [ -f "/app/ros2_ws/install/setup.bash" ]; then
    source /app/ros2_ws/install/setup.bash
fi

# Source custom environment file if it exists
if [ -f "/app/ros2_ws/src/role-ros2/docker/ros2_cu118/ros2_cu118.env" ]; then
    source /app/ros2_ws/src/role-ros2/docker/ros2_cu118/ros2_cu118.env
fi

# Set library paths
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}

# Set CMAKE and PKG_CONFIG paths
export CMAKE_PREFIX_PATH=/opt/ros/humble:${CMAKE_PREFIX_PATH}

# Set environment variables with defaults
export ROBOT_IP=${ROBOT_IP:-172.17.0.2}
export NUC_IP=${NUC_IP:-172.17.0.1}
export LAPTOP_IP=${LAPTOP_IP:-172.17.0.1}
export ROBOT_TYPE=${ROBOT_TYPE:-fr3}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Set DDS config based on RMW implementation
if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then
    CYCLONE_CONFIG="/app/ros2_ws/src/role-ros2/docker/ros2_cu118/cyclonedds.xml"
    if [ -f "$CYCLONE_CONFIG" ]; then
        export CYCLONEDDS_URI="file://$CYCLONE_CONFIG"
    fi
elif [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ]; then
    PROFILE_PATH="/app/ros2_ws/src/role-ros2/docker/ros2_cu118/fastrtps_profile.xml"
    if [ -f "$PROFILE_PATH" ]; then
        export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE_PATH"
    fi
fi

# ROS2 logging settings
export RCUTILS_LOGGING_SEVERITY=${RCUTILS_LOGGING_SEVERITY:-INFO}
export RCUTILS_LOGGING_USE_STDOUT=1

# CUDA environment (if needed)
if [ -n "$CUDA_VISIBLE_DEVICES" ]; then
    export CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES}
fi

# Execute command passed to container
exec "$@"

