#!/bin/bash
set -e

# Entrypoint script for Polymetis + ROS2 Docker container
# 
# This script can optionally auto-configure the environment.
# To skip auto-configuration, set SKIP_AUTO_ENV=true before running the container.
# Otherwise, manually source polymetis_ros2.env after entering the container.

# Check if auto-configuration should be skipped
if [ "${SKIP_AUTO_ENV}" != "true" ]; then
    # Auto-configure environment (for backward compatibility)
    # Fix git safe directory for Polymetis version detection
    # Note: In ros2_polymetis, entire fairo directory is copied to /opt/fairo
    # This includes the .git directory, so version detection works automatically
    git config --global --add safe.directory /opt/fairo 2>/dev/null || true

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
        CYCLONE_CONFIG="/app/ros2_ws/src/role-ros2/docker/ros2_polymetis/cyclonedds.xml"
        if [ -f "$CYCLONE_CONFIG" ]; then
            export CYCLONEDDS_URI="file://$CYCLONE_CONFIG"
        fi
    elif [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ]; then
        PROFILE_PATH="/app/ros2_ws/src/role-ros2/docker/ros2_polymetis/fastrtps_profile.xml"
        if [ -f "$PROFILE_PATH" ]; then
            export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE_PATH"
        fi
    fi

    # ROS2 logging settings
    export RCUTILS_LOGGING_SEVERITY=INFO
    export RCUTILS_LOGGING_USE_STDOUT=1
else
    # Skip auto-configuration - user will manually source polymetis_ros2.env
    echo "ℹ️  Auto-configuration skipped. Run 'source /app/ros2_ws/src/role-ros2/docker/ros2_polymetis/polymetis_ros2.env' to configure environment."
    
    # Still set critical ROS2 environment variables even when skipping auto-config
    # These are needed for ROS2 to work properly
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
    export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
    
    # Set DDS config if RMW_IMPLEMENTATION is set
    if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then
        CYCLONE_CONFIG="/app/ros2_ws/src/role-ros2/docker/ros2_polymetis/cyclonedds.xml"
        if [ -f "$CYCLONE_CONFIG" ]; then
            export CYCLONEDDS_URI="file://$CYCLONE_CONFIG"
        fi
    elif [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ]; then
        PROFILE_PATH="/app/ros2_ws/src/role-ros2/docker/ros2_polymetis/fastrtps_profile.xml"
        if [ -f "$PROFILE_PATH" ]; then
            export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE_PATH"
        fi
    fi
fi

# Execute command passed to container
exec "$@"
