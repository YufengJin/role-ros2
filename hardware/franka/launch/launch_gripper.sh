#!/bin/bash
# Launch gripper server for Polymetis
# Usage: ./launch_gripper.sh [robot_ip]
# Or set ROBOT_IP environment variable

# Get robot IP from argument, environment variable, or default
ROBOT_IP="${1:-${ROBOT_IP:-172.17.0.2}}"

echo "Launching gripper server with robot_ip=${ROBOT_IP}"

# Kill any existing processes
pkill -9 gripper 2>/dev/null

# Wait a bit for processes to terminate
sleep 1

# For Robotiq 2F gripper (USB connection), uncomment below:
#chmod a+rw /dev/ttyUSB0
#launch_gripper.py gripper=robotiq_2f gripper.comport=/dev/ttyUSB0

# Launch Franka Hand gripper server (network connection)
launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=${ROBOT_IP}
