#!/bin/bash
# Launch robot server for Polymetis
#
# Usage: 
#   ./launch_robot.sh                    # Use defaults from franka_robot_config_v2.yaml
#   ./launch_robot.sh [robot_ip]         # Specify robot IP
#   ./launch_robot.sh [robot_ip] [port]  # Specify robot IP and port
#
# Environment Variables:
#   ROBOT_IP       - Robot IP address (default: 172.17.0.2)
#   ROBOT_PORT     - Polymetis server port (default: 50051)
#   USE_REAL_TIME  - Enable real-time control: true/false (default: true)
#
# Arguments:
#   $1 - Robot IP address (optional, overrides ROBOT_IP)
#   $2 - Server port (optional, overrides ROBOT_PORT)
#   $3 - Use real-time control: true/false (optional, overrides USE_REAL_TIME)

set -e

# Default values
DEFAULT_ROBOT_IP="172.17.0.2"
DEFAULT_ROBOT_PORT="50051"
DEFAULT_USE_REAL_TIME="true"

# Get values from arguments, environment variables, or defaults
ROBOT_IP="${1:-${ROBOT_IP:-$DEFAULT_ROBOT_IP}}"
ROBOT_PORT="${2:-${ROBOT_PORT:-$DEFAULT_ROBOT_PORT}}"
USE_REAL_TIME="${3:-${USE_REAL_TIME:-$DEFAULT_USE_REAL_TIME}}"

echo "========================================"
echo "Launching Polymetis Robot Server"
echo "========================================"
echo "Robot IP:      ${ROBOT_IP}"
echo "Server Port:   ${ROBOT_PORT}"
echo "Real-time:     ${USE_REAL_TIME}"
echo "========================================"

# Kill any existing processes
echo "Cleaning up existing processes..."
pkill -9 -f "run_server" 2>/dev/null || true
pkill -9 -f "polymetis.*server" 2>/dev/null || true
pkill -9 -f "franka_panda_cl" 2>/dev/null || true
pkill -9 -f "franka_panda_client" 2>/dev/null || true

# Wait for processes to terminate
sleep 1

# Build command with optional real-time flag
CMD="launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=${ROBOT_IP} port=${ROBOT_PORT}"

if [ "$USE_REAL_TIME" = "true" ] || [ "$USE_REAL_TIME" = "True" ] || [ "$USE_REAL_TIME" = "1" ]; then
    CMD="$CMD use_real_time=true"
    echo "Real-time control: ENABLED"
else
    CMD="$CMD use_real_time=false"
    echo "Real-time control: DISABLED"
fi

echo ""
echo "Executing: $CMD"
echo ""

# Launch robot server
exec $CMD
