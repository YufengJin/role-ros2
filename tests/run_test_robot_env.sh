#!/bin/bash
# Run test_robot_env.py with proper ROS2 environment setup
#
# Usage:
#   ./run_test_robot_env.sh [--action-space ACTION_SPACE] [--full-test]
#
# Examples:
#   ./run_test_robot_env.sh
#   ./run_test_robot_env.sh --action-space cartesian_velocity
#   ./run_test_robot_env.sh --full-test

set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "WARNING: ROS2 environment not sourced. Attempting to source..."
    
    # Try to source ROS2 (common locations)
    if [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        echo "✓ Sourced ROS2 Foxy"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo "✓ Sourced ROS2 Humble"
    else
        echo "ERROR: ROS2 not found. Please source ROS2 setup.bash first:"
        echo "  source /opt/ros/foxy/setup.bash"
        exit 1
    fi
fi

# Check if workspace is built
if [ ! -d "$WORKSPACE_ROOT/install" ]; then
    echo "ERROR: Workspace not built. Please build first:"
    echo "  cd $WORKSPACE_ROOT && colcon build"
    exit 1
fi

# Source workspace
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
    echo "✓ Sourced workspace: $WORKSPACE_ROOT"
else
    echo "ERROR: Workspace setup.bash not found: $WORKSPACE_ROOT/install/setup.bash"
    exit 1
fi

# Run test script with all arguments
echo "Running test_robot_env.py..."
echo ""

cd "$SCRIPT_DIR"
python3 test_robot_env.py "$@"

