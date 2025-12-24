#!/bin/bash
# Entrypoint script for ROS2 Humble with CUDA 11.8 Docker container
# Enhanced daemon management with robust error handling and recovery

set -e

# Enable proper signal handling for Ctrl+C
trap 'cleanup_and_exit' INT TERM

# Cleanup function for graceful exit
cleanup_and_exit() {
    # Stop daemon gracefully on exit
    ros2 daemon stop >/dev/null 2>&1 || true
    exit 130
}

# Function to ensure ROS2 daemon is running
ensure_ros2_daemon() {
    local max_retries=3
    local retry_count=0
    
    # Ensure daemon directory exists (critical for daemon to work)
    mkdir -p ~/.ros/ros2_daemon
    
    while [ $retry_count -lt $max_retries ]; do
        # Step 1: Force stop any existing daemon processes
        ros2 daemon stop >/dev/null 2>&1 || true
        
        # Step 2: Kill any stuck daemon processes (in case stop didn't work)
        pkill -f "ros2.*daemon" >/dev/null 2>&1 || true
        pkill -f "ros2_daemon" >/dev/null 2>&1 || true
        
        # Step 3: Clean daemon state directory (fixes issues with corrupted state)
        # Ensure directory exists before cleaning
        mkdir -p ~/.ros/ros2_daemon
        if [ -d ~/.ros/ros2_daemon ]; then
            rm -rf ~/.ros/ros2_daemon/* 2>/dev/null || true
        fi
        
        # Step 4: Wait a moment for processes to fully terminate
        sleep 1
        
        # Step 5: Start daemon with explicit environment variables
        # Critical: Ensure daemon uses the same DDS configuration as nodes
        # Export current environment variables to ensure they're inherited
        # Note: FastRTPS doesn't need CYCLONEDDS_URI, but we keep it for compatibility
        if ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
           RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
           ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
           ros2 daemon start >/dev/null 2>&1; then
            # Step 6: Wait for daemon to initialize
            sleep 2
            
            # Step 7: Verify daemon is running
            if ros2 daemon status >/dev/null 2>&1; then
                return 0
            fi
        fi
        
        retry_count=$((retry_count + 1))
        if [ $retry_count -lt $max_retries ]; then
            echo "警告: ROS2 daemon 启动失败，重试 $retry_count/$max_retries..." >&2
            sleep 2
        fi
    done
    
    echo "错误: 无法启动 ROS2 daemon 经过 $max_retries 次重试" >&2
    return 1
}

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source ROS2 workspace if it exists
if [ -f "/app/ros2_ws/install/setup.bash" ]; then
    source /app/ros2_ws/install/setup.bash
fi

# Source ROS2 environment configuration (includes DDS settings like CYCLONEDDS_URI)
# This must be done before starting the daemon to ensure proper DDS configuration
if [ -f "/app/ros2_ws/src/role-ros2/docker/ros2_cu118/ros2_cu118.env" ]; then
    # Suppress output when not in interactive terminal (e.g., from entrypoint)
    source /app/ros2_ws/src/role-ros2/docker/ros2_cu118/ros2_cu118.env >/dev/null 2>&1 || \
    source /app/ros2_ws/src/role-ros2/docker/ros2_cu118/ros2_cu118.env
fi

# Initialize and start ROS2 daemon with robust error handling
ensure_ros2_daemon || {
    echo "警告: ROS2 daemon 初始化失败，但继续执行命令..." >&2
    echo "提示: 如果遇到问题，可以手动运行 'ros2 daemon stop && ros2 daemon start'" >&2
}

# Execute command passed to container
exec "$@"
