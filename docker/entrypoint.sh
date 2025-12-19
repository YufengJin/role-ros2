#!/bin/bash
set -e

# Source ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Source ROS2 workspace if it exists
if [ -f "/app/ros2_ws/install/setup.bash" ]; then
    source /app/ros2_ws/install/setup.bash
fi

# Initialize and activate micromamba environment
eval "$(micromamba shell hook --shell bash)"
micromamba activate polymetis-local

# Set library paths - prioritize system libraries (ROS2) over conda libraries
# This helps with library resolution, but spdlog conflict is handled via environment variables below
export LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
# Add conda and robotpkg libraries after system libraries
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/openrobots/lib:$CONDA_PREFIX/lib

# Set environment variables
export ROBOT_IP=${ROBOT_IP:-172.17.0.2}
export POLYMETIS_IP=${POLYMETIS_IP:-127.0.0.1}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
# Enable network discovery for cross-container communication
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Set FastRTPS profile for network discovery (if profile file exists)
PROFILE_PATH="/app/ros2_ws/src/role-ros2/docker/fastrtps_profile.xml"
if [ -f "$PROFILE_PATH" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE_PATH"
    echo "FastRTPS Profile: $FASTRTPS_DEFAULT_PROFILES_FILE"
else
    echo "Warning: FastRTPS profile not found at $PROFILE_PATH"
fi

# Fix spdlog conflict: Disable ROS2 logging to avoid spdlog symbol conflicts
# Note: For ROS2 Python nodes, use system Python (/usr/bin/python3) instead of conda Python
# to avoid spdlog symbol conflicts. Conda Python loads conda's spdlog which conflicts with ROS2.
export RCUTILS_LOGGING_SEVERITY=ERROR
export RCUTILS_LOGGING_USE_STDOUT=1
# Set default Python for ROS2 nodes (use system Python to avoid spdlog conflicts)
export ROS2_PYTHON=/usr/bin/python3

# Print environment info
echo "=========================================="
echo "ROS2 Foxy + Polymetis Container"
echo "=========================================="
echo "ROS2 Version: $(ros2 --version 2>/dev/null || echo 'Not available')"
echo "Python Version: $(python --version 2>/dev/null || echo 'Not available')"
echo "Micromamba Environment: $(micromamba env list | grep '*' | awk '{print $1}' || echo 'polymetis-local')"
echo "ROBOT_IP: ${ROBOT_IP}"
echo "POLYMETIS_IP: ${POLYMETIS_IP}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY}"
echo "ROS2 Workspace: /app/ros2_ws"
echo "=========================================="

# Execute command passed to container
exec "$@"
