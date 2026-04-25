#!/bin/bash
# Entrypoint for ROS2 Humble + xArm Docker container.
# Mirrors the franka entrypoint pattern but without libfranka / polymetis logic.

set -e

trap 'cleanup_and_exit' INT TERM

cleanup_and_exit() {
    ros2 daemon stop >/dev/null 2>&1 || true
    exit 130
}

ensure_ros2_daemon() {
    local max_retries=3
    local retry_count=0

    mkdir -p ~/.ros/ros2_daemon

    while [ $retry_count -lt $max_retries ]; do
        ros2 daemon stop >/dev/null 2>&1 || true
        pkill -f "ros2.*daemon"  >/dev/null 2>&1 || true
        pkill -f "ros2_daemon"   >/dev/null 2>&1 || true

        mkdir -p ~/.ros/ros2_daemon
        rm -rf ~/.ros/ros2_daemon/* 2>/dev/null || true

        sleep 1

        if ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
           RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
           ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
           ros2 daemon start >/dev/null 2>&1; then
            sleep 2
            if ros2 daemon status >/dev/null 2>&1; then
                return 0
            fi
        fi

        retry_count=$((retry_count + 1))
        if [ $retry_count -lt $max_retries ]; then
            echo "Warning: ROS2 daemon startup failed, retrying $retry_count/$max_retries..." >&2
            sleep 2
        fi
    done

    echo "Error: Failed to start ROS2 daemon after $max_retries retries" >&2
    return 1
}

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Auto colcon build when a workspace src exists (mounted via docker-compose)
if [ -d "/app/ros2_ws/src" ] && [ -n "$(ls -A /app/ros2_ws/src 2>/dev/null)" ]; then
    echo "Running colcon build --symlink-install..." >&2
    (cd /app/ros2_ws && colcon build --symlink-install) || \
        echo "Warning: colcon build failed. Run 'cd /app/ros2_ws && colcon build --symlink-install' manually." >&2
fi

if [ -f "/app/ros2_ws/install/setup.bash" ]; then
    source /app/ros2_ws/install/setup.bash
fi

# Source xArm/ROS2 env file unless disabled
if [ "${SKIP_AUTO_ENV:-false}" != "true" ] && [ -f "/etc/ros2_xarm/ros2_xarm.env" ]; then
    source /etc/ros2_xarm/ros2_xarm.env >/dev/null 2>&1 || \
    source /etc/ros2_xarm/ros2_xarm.env
fi

ensure_ros2_daemon || {
    echo "Warning: ROS2 daemon initialization failed, continuing..." >&2
    echo "Tip: manually run 'ros2 daemon stop && ros2 daemon start' if topics don't show up." >&2
}

exec "$@"
