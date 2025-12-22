#!/bin/bash
# Launch Robot Server Script
# This script is designed to run in Docker environment after sourcing polymetis_ros2.env
# The environment (micromamba) should already be activated by polymetis_ros2.env
# For host system, use /home/yjin/miniconda3/envs/polymetis-local/bin/python

set -e  # Exit on error

# ============================================================================
# Configuration
# ============================================================================
# Detect if running in Docker or host system
if [ -f "/app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env" ]; then
    # Docker environment - use system launch_robot.py
    LAUNCH_ROBOT_CMD="launch_robot.py"
    PYTHON_ENV=""
else
    # Host system - use conda environment
    CONDA_ENV="/home/yjin/miniconda3/envs/polymetis-local"
    if [ -d "$CONDA_ENV" ]; then
        LAUNCH_ROBOT_CMD="$CONDA_ENV/bin/launch_robot.py"
        PYTHON_ENV="$CONDA_ENV/bin/python"
        export PATH="$CONDA_ENV/bin:$PATH"
    else
        echo "Error: Conda environment not found at $CONDA_ENV"
        exit 1
    fi
fi

# Use ROBOT_IP environment variable if set, otherwise use default
ROBOT_IP=${ROBOT_IP:-172.17.0.2}

# ============================================================================
# Kill existing processes
# ============================================================================
echo "Cleaning up existing processes..."

# Kill by process name (more reliable)
pkill -9 -f "run_server.*50051" 2>/dev/null || true
pkill -9 -f "franka_panda_client" 2>/dev/null || true
pkill -9 -f "launch_robot.py" 2>/dev/null || true

# Kill by exact process name (fallback)
pkill -9 run_server 2>/dev/null || true
pkill -9 franka_panda_client 2>/dev/null || true
pkill -9 franka_panda_cl 2>/dev/null || true  # Legacy name

# Kill sudo processes that might be running run_server
if command -v pgrep >/dev/null 2>&1; then
    # Find sudo processes running run_server or franka_panda
    SUDO_PIDS=$(pgrep -f "sudo.*run_server|sudo.*franka_panda" 2>/dev/null || true)
    if [ -n "$SUDO_PIDS" ]; then
        echo "Killing sudo processes: $SUDO_PIDS"
        # Try without sudo first, then with sudo if needed
        echo "$SUDO_PIDS" | xargs -r kill -9 2>/dev/null || \
        (command -v sudo >/dev/null 2>&1 && echo "$SUDO_PIDS" | xargs -r sudo kill -9 2>/dev/null) || true
    fi
    # Also find and kill run_server processes that might be running as root
    ROOT_PIDS=$(pgrep -f "run_server.*50051" 2>/dev/null || true)
    if [ -n "$ROOT_PIDS" ]; then
        echo "Killing root run_server processes: $ROOT_PIDS"
        echo "$ROOT_PIDS" | xargs -r kill -9 2>/dev/null || \
        (command -v sudo >/dev/null 2>&1 && echo "$ROOT_PIDS" | xargs -r sudo kill -9 2>/dev/null) || true
    fi
fi

# Wait for processes to terminate
sleep 1

# Check if port 50051 is still in use and kill the process using it
# First, try to find and kill the process
if command -v lsof >/dev/null 2>&1; then
    PORT_PID=$(lsof -ti :50051 2>/dev/null || true)
    if [ -n "$PORT_PID" ]; then
        echo "Killing process using port 50051 (PID: $PORT_PID)"
        kill -9 "$PORT_PID" 2>/dev/null || true
        if command -v sudo >/dev/null 2>&1; then
            sudo kill -9 "$PORT_PID" 2>/dev/null || true
        fi
        sleep 1
    fi
elif command -v fuser >/dev/null 2>&1; then
    if fuser 50051/tcp >/dev/null 2>&1; then
        echo "Killing processes using port 50051 with fuser"
        fuser -k 50051/tcp 2>/dev/null || true
        if command -v sudo >/dev/null 2>&1; then
            sudo fuser -k 50051/tcp 2>/dev/null || true
        fi
        sleep 1
    fi
fi

# Check if port is connectable (socket test - more reliable than process check)
check_port_connectable() {
    python3 -c "
import socket
import sys
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.5)
    result = s.connect_ex(('127.0.0.1', $1))
    s.close()
    sys.exit(0 if result == 0 else 1)
except:
    sys.exit(1)
" 2>/dev/null
}

# Final check: verify port is free (both process and socket)
MAX_RETRIES=15  # Increased to 15 (TIME_WAIT can last up to 120 seconds, but we check every 2 seconds)
RETRY=0
PORT_FREE=false

while [ $RETRY -lt $MAX_RETRIES ]; do
    PORT_IN_USE=false
    
    # Check if port is connectable (socket test)
    if check_port_connectable 50051; then
        PORT_IN_USE=true
        # Try to kill again if we can find the process
        if command -v lsof >/dev/null 2>&1; then
            PORT_PID=$(lsof -ti :50051 2>/dev/null || true)
            if [ -n "$PORT_PID" ]; then
                echo "Port 50051 still connectable, killing PID $PORT_PID again..."
                kill -9 "$PORT_PID" 2>/dev/null || true
                if command -v sudo >/dev/null 2>&1; then
                    sudo kill -9 "$PORT_PID" 2>/dev/null || true
                fi
            fi
        fi
    fi
    
    # Also check with traditional tools
    if command -v lsof >/dev/null 2>&1; then
        if lsof -ti :50051 >/dev/null 2>&1; then
            PORT_IN_USE=true
        fi
    elif command -v netstat >/dev/null 2>&1; then
        if netstat -tuln 2>/dev/null | grep -q ":50051 "; then
            PORT_IN_USE=true
        fi
    elif command -v ss >/dev/null 2>&1; then
        if ss -tuln 2>/dev/null | grep -q ":50051 "; then
            PORT_IN_USE=true
        fi
    fi
    
    if [ "$PORT_IN_USE" = "false" ]; then
        PORT_FREE=true
        break
    fi
    
    RETRY=$((RETRY + 1))
    echo "Port 50051 still in use, waiting... ($RETRY/$MAX_RETRIES)"
    sleep 2  # Wait 2 seconds between retries (TIME_WAIT typically lasts 60-120 seconds)
done

if [ "$PORT_FREE" = "false" ]; then
    echo "Warning: Port 50051 may still be in use after $MAX_RETRIES retries (${MAX_RETRIES}0 seconds)."
    echo "This might be due to TIME_WAIT state. Proceeding anyway..."
fi

echo "Cleanup complete."

# ============================================================================
# Launch robot server
# ============================================================================
echo "Launching robot server with ROBOT_IP=${ROBOT_IP}"

# Launch robot server
# Note: In Docker, disable real-time to avoid SIGSEGV during network connection
# Real-time mode can cause issues with network operations in Docker containers
if [ -n "$PYTHON_ENV" ]; then
    # Host system: use explicit Python path
    "$PYTHON_ENV" "$LAUNCH_ROBOT_CMD" robot_client=franka_hardware robot_client.executable_cfg.robot_ip=${ROBOT_IP}
else
    # Docker: use system command (should be in PATH after sourcing polymetis_ros2.env)
    # Disable real-time in Docker to avoid SIGSEGV during network connection
    $LAUNCH_ROBOT_CMD robot_client=franka_hardware robot_client.executable_cfg.robot_ip=${ROBOT_IP} use_real_time=false
fi
