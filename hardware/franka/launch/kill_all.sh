#!/bin/bash
# Kill all Polymetis robot and gripper server processes so ports (50051, 50052, etc.) are released.
#
# Usage:
#   ./kill_all.sh
#
# Used for manual cleanup. When using ros2 launch role_ros2 franka_robot.launch.py,
# the launch file runs cleanup_polymetis_servers.sh (same logic) before starting servers.

echo "Killing Polymetis robot and gripper server processes..."
pkill -9 -f 'run_server' 2>/dev/null || true
pkill -9 -f 'polymetis.*server' 2>/dev/null || true
pkill -9 -f 'franka_panda_cl' 2>/dev/null || true
pkill -9 -f 'franka_panda_client' 2>/dev/null || true
pkill -9 -f 'franka_hand_cl' 2>/dev/null || true
pkill -9 -f 'franka_hand_client' 2>/dev/null || true
pkill -9 -f 'launch_gripper\.py' 2>/dev/null || true
pkill -9 -f 'launch_robot\.py' 2>/dev/null || true
pkill -9 -f 'robotiq' 2>/dev/null || true
echo "Done."
