#!/bin/bash
# Launch gripper server for Polymetis
#
# Usage:
#   ./launch_gripper.sh                              # Use defaults from franka_robot_config_v2.yaml
#   ./launch_gripper.sh [robot_ip]                   # Specify robot IP
#   ./launch_gripper.sh [robot_ip] [port]            # Specify robot IP and port
#   ./launch_gripper.sh [robot_ip] [port] [type]     # Specify robot IP, port, and gripper type
#
# Environment Variables:
#   ROBOT_IP      - Robot IP address (default: 172.17.0.2)
#   GRIPPER_PORT  - Polymetis gripper server port (default: 50052)
#   GRIPPER_TYPE  - Gripper type: franka_hand or robotiq_2f (default: franka_hand)
#   COMPORT       - COM port for Robotiq gripper (default: /dev/ttyUSB0)
#
# Arguments:
#   $1 - Robot IP address (optional, overrides ROBOT_IP)
#   $2 - Server port (optional, overrides GRIPPER_PORT)
#   $3 - Gripper type: franka_hand or robotiq_2f (optional, overrides GRIPPER_TYPE)
#
# Gripper Types:
#   franka_hand - Franka Hand gripper (network connection via robot_ip)
#   robotiq_2f  - Robotiq 2F gripper (USB connection via COMPORT)

set -e

# Default values
DEFAULT_ROBOT_IP="172.17.0.2"
DEFAULT_GRIPPER_PORT="50052"
DEFAULT_GRIPPER_TYPE="franka_hand"
DEFAULT_COMPORT="/dev/ttyUSB0"

# Get values from arguments, environment variables, or defaults
ROBOT_IP="${1:-${ROBOT_IP:-$DEFAULT_ROBOT_IP}}"
GRIPPER_PORT="${2:-${GRIPPER_PORT:-$DEFAULT_GRIPPER_PORT}}"
GRIPPER_TYPE="${3:-${GRIPPER_TYPE:-$DEFAULT_GRIPPER_TYPE}}"
COMPORT="${COMPORT:-$DEFAULT_COMPORT}"

echo "========================================"
echo "Launching Polymetis Gripper Server"
echo "========================================"
echo "Robot IP:      ${ROBOT_IP}"
echo "Server Port:   ${GRIPPER_PORT}"
echo "Gripper Type:  ${GRIPPER_TYPE}"
if [ "$GRIPPER_TYPE" = "robotiq_2f" ]; then
    echo "COM Port:      ${COMPORT}"
fi
echo "========================================"

# Kill any existing processes
echo "Cleaning up existing processes..."
pkill -9 -f "franka_hand_cl" 2>/dev/null || true
pkill -9 -f "franka_hand_client" 2>/dev/null || true
pkill -9 -f "robotiq" 2>/dev/null || true
pkill -9 -f "launch_gripper" 2>/dev/null || true

# Wait for processes to terminate
sleep 1

# Build command based on gripper type
case "$GRIPPER_TYPE" in
    "franka_hand")
        CMD="launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=${ROBOT_IP} port=${GRIPPER_PORT}"
        echo "Using Franka Hand gripper (network connection)"
        ;;
    "robotiq_2f")
        # Ensure USB port permissions
        if [ -e "$COMPORT" ]; then
            chmod a+rw "$COMPORT" 2>/dev/null || echo "Warning: Could not set permissions on $COMPORT"
        else
            echo "Warning: COM port $COMPORT does not exist. Gripper may not connect."
        fi
        CMD="launch_gripper.py gripper=robotiq_2f gripper.comport=${COMPORT} port=${GRIPPER_PORT}"
        echo "Using Robotiq 2F gripper (USB connection)"
        ;;
    *)
        echo "Error: Unknown gripper type '$GRIPPER_TYPE'"
        echo "Supported types: franka_hand, robotiq_2f"
        exit 1
        ;;
esac

echo ""
echo "Executing: $CMD"
echo ""

# Launch gripper server
exec $CMD
