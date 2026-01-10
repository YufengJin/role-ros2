#!/bin/bash
# Entrypoint script for Polymetis + ROS2 Docker container
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
            echo "Warning: ROS2 daemon startup failed, retrying $retry_count/$max_retries..." >&2
            sleep 2
        fi
    done
    
    echo "Error: Failed to start ROS2 daemon after $max_retries retries" >&2
    return 1
}

# Fix git safe directory for Polymetis version detection
git config --global --add safe.directory /opt/fairo 2>/dev/null || true

# Source ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Source ROS2 workspace if it exists
if [ -f "/app/ros2_ws/install/setup.bash" ]; then
    source /app/ros2_ws/install/setup.bash
fi

# Source ROS2 environment configuration (includes DDS settings)
# This must be done before starting the daemon to ensure proper DDS configuration
if [ -f "/app/ros2_ws/src/role-ros2/docker/ros2_franka_libfranka_0.18.x/polymetis_ros2.env" ]; then
    # Suppress output when not in interactive terminal (e.g., from entrypoint)
    source /app/ros2_ws/src/role-ros2/docker/ros2_franka_libfranka_0.18.x/polymetis_ros2.env >/dev/null 2>&1 || \
    source /app/ros2_ws/src/role-ros2/docker/ros2_franka_libfranka_0.18.x/polymetis_ros2.env
fi

# Set Polymetis Python path
export PYTHONPATH=/opt/fairo/polymetis/polymetis/python:/usr/lib/python3.8/site-packages:${PYTHONPATH}

# Set CONDA_PREFIX for torchcontrol to find shared libraries
export CONDA_PREFIX=/opt/fairo/polymetis/polymetis/build

# Set library paths
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/foxy/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/openrobots/lib:/usr/local/lib:/opt/fairo/polymetis/polymetis/build/torch_isolation

# ROS2 DDS configuration - using FastRTPS with shared memory disabled
# This avoids SIGSEGV issues in Docker environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# ROS2 logging
export RCUTILS_LOGGING_SEVERITY=INFO
export RCUTILS_LOGGING_USE_STDOUT=1

# Set DISPLAY environment variable to suppress GLFW X11 warnings in Docker
# This prevents "X11: Failed to open display" warnings when running in headless mode
export DISPLAY=${DISPLAY:-:0}

# Initialize and start ROS2 daemon with robust error handling
ensure_ros2_daemon || {
    echo "Warning: ROS2 daemon initialization failed, but continuing with command execution..." >&2
    echo "Tip: If you encounter issues, you can manually run 'ros2 daemon stop && ros2 daemon start'" >&2
}

# Execute command passed to container
exec "$@"
