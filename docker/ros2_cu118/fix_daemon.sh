#!/bin/bash
# ROS2 Daemon repair and cleanup script
# Usage:
#   ./fix_daemon.sh          # Clean and start daemon (default)
#   ./fix_daemon.sh --cleanup-only  # Only cleanup, do not start daemon

set +e  # Allow commands to fail (some commands may fail during cleanup)

# Parse arguments
CLEANUP_ONLY=false
if [ "$1" = "--cleanup-only" ] || [ "$1" = "-c" ]; then
    CLEANUP_ONLY=true
fi

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

if [ "$CLEANUP_ONLY" = true ]; then
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}ROS2 Force Cleanup Script${NC}"
    echo -e "${BLUE}========================================${NC}"
else
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}ROS2 Daemon Repair Tool${NC}"
    echo -e "${BLUE}========================================${NC}"
fi
echo ""

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Step 1: Check current daemon status (only in fix mode)
if [ "$CLEANUP_ONLY" = false ]; then
    echo -e "${CYAN}[1/6] Checking current daemon status${NC}"
    if ros2 daemon status >/dev/null 2>&1; then
        echo -e "${GREEN}✓ Daemon is running${NC}"
        ros2 daemon status
    else
        echo -e "${YELLOW}⚠ Daemon is not running or has issues${NC}"
    fi
    echo ""
fi

# Step 2: Stop all existing daemon processes gracefully
if [ "$CLEANUP_ONLY" = false ]; then
    echo -e "${CYAN}[2/6] Stopping all existing daemon processes${NC}"
else
    echo -e "${CYAN}[1/5] Stopping ROS2 daemon processes${NC}"
fi

# First try graceful stop
ros2 daemon stop >/dev/null 2>&1 || true
sleep 2

# Wait for processes to exit
for i in {1..5}; do
    if ! pgrep -f "ros2.*daemon" >/dev/null 2>&1 && ! pgrep -f "_ros2_daemon" >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

# If processes still exist, use SIGTERM (gentler)
pkill -TERM -f "_ros2_daemon" >/dev/null 2>&1 || true
pkill -TERM -f "ros2.*daemon" >/dev/null 2>&1 || true
pkill -TERM -f "ros2_daemon" >/dev/null 2>&1 || true
sleep 2

# Finally use SIGKILL (forceful)
pkill -9 -f "_ros2_daemon" >/dev/null 2>&1 || true
pkill -9 -f "ros2.*daemon" >/dev/null 2>&1 || true
pkill -9 -f "ros2_daemon" >/dev/null 2>&1 || true
sleep 1

echo -e "${GREEN}✓ Stopped all daemon processes${NC}"
echo ""

# Step 3: Clean stuck CLI and node processes (only in cleanup mode)
if [ "$CLEANUP_ONLY" = true ]; then
    echo -e "${CYAN}[2/5] Cleaning stuck ROS2 CLI and node processes${NC}"
    # First try graceful stop of node processes
    pkill -TERM -f "demo_nodes_py" >/dev/null 2>&1 || true
    sleep 2

    # Wait for processes to exit
    for i in {1..3}; do
        if ! pgrep -f "demo_nodes_py" >/dev/null 2>&1; then
            break
        fi
        sleep 1
    done

    # Clean stuck CLI processes (excluding daemon)
    pkill -TERM -f "ros2 run" >/dev/null 2>&1 || true
    pkill -TERM -f "ros2 topic" >/dev/null 2>&1 || true
    pkill -TERM -f "ros2 node" >/dev/null 2>&1 || true
    sleep 2

    # Finally force cleanup
    pkill -9 -f "demo_nodes_py" >/dev/null 2>&1 || true
    pkill -9 -f "ros2 run" >/dev/null 2>&1 || true
    pkill -9 -f "ros2 topic" >/dev/null 2>&1 || true
    pkill -9 -f "ros2 node" >/dev/null 2>&1 || true
    sleep 1
    echo -e "${GREEN}✓ Done${NC}"
    echo ""
fi

# Step 4: Clean daemon state
if [ "$CLEANUP_ONLY" = false ]; then
    echo -e "${CYAN}[3/6] Cleaning daemon state${NC}"
else
    echo -e "${CYAN}[3/5] Cleaning ROS2 temporary files and logs${NC}"
fi

# ROS2 daemon stores state in ~/.ros/ros2_daemon/
# Ensure directory exists (critical for daemon to work)
mkdir -p ~/.ros/ros2_daemon
if [ -d ~/.ros/ros2_daemon ]; then
    rm -rf ~/.ros/ros2_daemon/* 2>/dev/null || true
    if [ "$CLEANUP_ONLY" = false ]; then
        echo -e "${GREEN}✓ Cleaned daemon state directory${NC}"
    fi
fi

# In cleanup mode, also clean other temporary files
if [ "$CLEANUP_ONLY" = true ]; then
    rm -rf ~/.ros/log 2>/dev/null || true
    rm -rf ~/.ros/daemon 2>/dev/null || true
    echo -e "${GREEN}✓ Done${NC}"
fi
echo ""

# Step 5: Clean DDS discovery files (only in cleanup mode)
if [ "$CLEANUP_ONLY" = true ]; then
    echo -e "${CYAN}[4/5] Cleaning DDS discovery temporary files${NC}"
    rm -rf /tmp/ros_domain_id_echo* 2>/dev/null || true
    rm -rf /tmp/cyclonedds* 2>/dev/null || true
    rm -rf /tmp/dds* 2>/dev/null || true
    echo -e "${GREEN}✓ Done${NC}"
    echo ""
fi

# Step 6: Start daemon (only in fix mode)
if [ "$CLEANUP_ONLY" = false ]; then
    echo -e "${CYAN}[4/6] Starting ROS2 daemon${NC}"
    # Source environment configuration if available
    if [ -f "/app/ros2_ws/src/role-ros2/docker/ros2_cu118/ros2_cu118.env" ]; then
        source /app/ros2_ws/src/role-ros2/docker/ros2_cu118/ros2_cu118.env
    fi

    # Start daemon with explicit environment variables to ensure correct DDS configuration
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
    ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
    ros2 daemon start
    sleep 2

    # Verify daemon is running
    echo -e "${CYAN}[5/6] Verifying daemon status${NC}"
    if ros2 daemon status >/dev/null 2>&1; then
        echo -e "${GREEN}✓ Daemon started successfully${NC}"
        ros2 daemon status
    else
        echo -e "${RED}✗ Daemon startup failed${NC}"
        echo "  Attempting manual start..."
        ros2 daemon start --verbose
        sleep 2
        ros2 daemon status
    fi
    echo ""

    echo -e "${CYAN}[6/6] Done${NC}"
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Repair completed${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "${YELLOW}Test commands:${NC}"
    echo "  ${CYAN}ros2 topic list${NC}"
    echo "  ${CYAN}ros2 node list${NC}"
else
    echo -e "${CYAN}[5/5] Done${NC}"
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Cleanup completed!${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "${YELLOW}Next step: Restart daemon${NC}"
    echo "  ${CYAN}ros2 daemon start${NC}"
    echo "  Or run: ${CYAN}./fix_daemon.sh${NC}"
    echo ""
fi
