#!/bin/bash
# Test Polymetis dependencies in ros2_polymetis container

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROLE_ROS2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$ROLE_ROS2_DIR/docker/ros2_polymetis"

cd "$DOCKER_DIR"

print_header() {
    echo "========================================"
    echo "$1"
    echo "========================================"
}

print_header "Testing Polymetis Dependencies in ros2_polymetis"

docker compose run --rm -e SKIP_AUTO_ENV=true ros2_polymetis bash -c "
    cd /app/ros2_ws
    
    # Source ROS2 (required for colcon build)
    if [ -f /opt/ros/foxy/setup.bash ]; then
        source /opt/ros/foxy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # Build workspace if not already built (required for role_ros2 messages/services)
    # IMPORTANT: Build with system Python (SKIP_AUTO_ENV=true means conda is not activated)
    if [ ! -f install/setup.bash ]; then
        echo 'Building ROS2 workspace (this may take a few minutes)...'
        echo '  This generates role_ros2 messages and services'
        colcon build --symlink-install --packages-select role_ros2 || {
            echo 'Warning: Workspace build failed. Some tests may fail.'
            echo '  You can manually build later: cd /app/ros2_ws && colcon build --symlink-install'
        }
    else
        echo 'Workspace already built, skipping build step'
    fi
    
    # Source workspace (makes generated messages/services available)
    if [ -f install/setup.bash ]; then
        source install/setup.bash
        echo '✓ ROS2 workspace sourced'
    else
        echo '⚠️  Warning: Workspace not built. role_ros2 imports will fail.'
    fi
    
    # Source polymetis environment (required for Polymetis imports)
    # This activates conda environment and sets library paths
    if [ -f /app/ros2_ws/src/role-ros2/docker/ros2_polymetis/polymetis_ros2.env ]; then
        source /app/ros2_ws/src/role-ros2/docker/ros2_polymetis/polymetis_ros2.env
    elif [ -f /app/ros2_ws/src/role-ros2/docker/polymetis/polymetis_ros2.env ]; then
        source /app/ros2_ws/src/role-ros2/docker/polymetis/polymetis_ros2.env
    fi
    
    # Set environment
    export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
    export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
    export ROS_LOCALHOST_ONLY=\${ROS_LOCALHOST_ONLY:-0}
    
    echo ''
    echo 'Testing Polymetis dependencies...'
    echo ''
    
    # Run the test script
    python3 /app/ros2_ws/src/role-ros2/tests/test_polymetis_dependencies.py
"

