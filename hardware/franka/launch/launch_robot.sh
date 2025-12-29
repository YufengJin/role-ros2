#!/bin/bash
# Launch robot server for Polymetis
# Usage: ./launch_robot.sh [robot_ip]
# Or set ROBOT_IP environment variable

# Get robot IP from argument, environment variable, or default
ROBOT_IP="${1:-${ROBOT_IP:-172.17.0.2}}"

echo "Launching robot server with robot_ip=${ROBOT_IP}"

# Kill any existing processes
pkill -9 run_server 2>/dev/null
pkill -9 franka_panda_cl 2>/dev/null

# Wait a bit for processes to terminate
sleep 1

# Launch robot server
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=${ROBOT_IP}
