#!/bin/bash
# Kill All Polymetis Processes Script
# This script kills all robot and gripper related processes and frees ports 50051 and 50052
# Works in both Docker and host system environments

set -e  # Exit on error

echo "============================================================================"
echo "Killing All Polymetis Processes"
echo "============================================================================"

# ============================================================================
# Kill Robot Server Processes
# ============================================================================
echo ""
echo "--- Cleaning up Robot Server processes ---"

# Kill by process name patterns (more reliable)
pkill -9 -f "run_server.*50051" 2>/dev/null && echo "  ✓ Killed run_server (port 50051)" || true
pkill -9 -f "franka_panda_client" 2>/dev/null && echo "  ✓ Killed franka_panda_client" || true
pkill -9 -f "launch_robot.py" 2>/dev/null && echo "  ✓ Killed launch_robot.py" || true

# Kill by exact process name (fallback)
pkill -9 run_server 2>/dev/null && echo "  ✓ Killed run_server (all)" || true
pkill -9 franka_panda_client 2>/dev/null && echo "  ✓ Killed franka_panda_client (all)" || true
pkill -9 franka_panda_cl 2>/dev/null && echo "  ✓ Killed franka_panda_cl (legacy)" || true

# Kill sudo processes that might be running run_server
if command -v pgrep >/dev/null 2>&1; then
    SUDO_PIDS=$(pgrep -f "sudo.*run_server|sudo.*franka_panda" 2>/dev/null || true)
    if [ -n "$SUDO_PIDS" ]; then
        echo "  Killing sudo processes: $SUDO_PIDS"
        echo "$SUDO_PIDS" | xargs -r kill -9 2>/dev/null || true
        if command -v sudo >/dev/null 2>&1; then
            echo "$SUDO_PIDS" | xargs -r sudo kill -9 2>/dev/null || true
        fi
        echo "  ✓ Killed sudo processes"
    fi
    
    # Also find and kill run_server processes that might be running as root
    ROOT_PIDS=$(pgrep -f "run_server.*50051" 2>/dev/null || true)
    if [ -n "$ROOT_PIDS" ]; then
        echo "  Killing root run_server processes: $ROOT_PIDS"
        echo "$ROOT_PIDS" | xargs -r kill -9 2>/dev/null || true
        if command -v sudo >/dev/null 2>&1; then
            echo "$ROOT_PIDS" | xargs -r sudo kill -9 2>/dev/null || true
        fi
        echo "  ✓ Killed root run_server processes"
    fi
fi

# ============================================================================
# Kill Gripper Server Processes
# ============================================================================
echo ""
echo "--- Cleaning up Gripper Server processes ---"

# Kill by process name patterns
pkill -9 -f "launch_gripper.py" 2>/dev/null && echo "  ✓ Killed launch_gripper.py" || true
pkill -9 -f "franka_hand_client" 2>/dev/null && echo "  ✓ Killed franka_hand_client" || true
pkill -9 -f "gripper.*server" 2>/dev/null && echo "  ✓ Killed gripper server" || true

# Kill by exact process name (fallback)
pkill -9 gripper 2>/dev/null && echo "  ✓ Killed gripper (all)" || true

# Kill sudo processes that might be running gripper
if command -v pgrep >/dev/null 2>&1; then
    SUDO_PIDS=$(pgrep -f "sudo.*gripper|sudo.*franka_hand" 2>/dev/null || true)
    if [ -n "$SUDO_PIDS" ]; then
        echo "  Killing sudo processes: $SUDO_PIDS"
        echo "$SUDO_PIDS" | xargs -r kill -9 2>/dev/null || true
        if command -v sudo >/dev/null 2>&1; then
            echo "$SUDO_PIDS" | xargs -r sudo kill -9 2>/dev/null || true
        fi
        echo "  ✓ Killed sudo processes"
    fi
    
    # Also find and kill gripper processes that might be running as root
    ROOT_PIDS=$(pgrep -f "gripper.*50052|franka_hand.*50052" 2>/dev/null || true)
    if [ -n "$ROOT_PIDS" ]; then
        echo "  Killing root gripper processes: $ROOT_PIDS"
        echo "$ROOT_PIDS" | xargs -r kill -9 2>/dev/null || true
        if command -v sudo >/dev/null 2>&1; then
            echo "$ROOT_PIDS" | xargs -r sudo kill -9 2>/dev/null || true
        fi
        echo "  ✓ Killed root gripper processes"
    fi
fi

# Wait for processes to terminate
sleep 1

# ============================================================================
# Kill Processes Using Ports
# ============================================================================
echo ""
echo "--- Cleaning up processes using ports 50051 and 50052 ---"

# Function to kill process using a port
kill_port_process() {
    local PORT=$1
    local KILLED=false
    
    if command -v lsof >/dev/null 2>&1; then
        PORT_PIDS=$(lsof -ti :${PORT} 2>/dev/null || true)
        if [ -n "$PORT_PIDS" ]; then
            echo "  Port ${PORT}: Found PIDs $PORT_PIDS"
            echo "$PORT_PIDS" | xargs -r kill -9 2>/dev/null || true
            if command -v sudo >/dev/null 2>&1; then
                echo "$PORT_PIDS" | xargs -r sudo kill -9 2>/dev/null || true
            fi
            KILLED=true
        fi
    elif command -v fuser >/dev/null 2>&1; then
        if fuser ${PORT}/tcp >/dev/null 2>&1; then
            echo "  Port ${PORT}: Killing with fuser"
            fuser -k ${PORT}/tcp 2>/dev/null || true
            if command -v sudo >/dev/null 2>&1; then
                sudo fuser -k ${PORT}/tcp 2>/dev/null || true
            fi
            KILLED=true
        fi
    fi
    
    if [ "$KILLED" = "true" ]; then
        echo "  ✓ Killed processes using port ${PORT}"
    else
        echo "  ✓ Port ${PORT} is free"
    fi
}

kill_port_process 50051
kill_port_process 50052

sleep 1

# ============================================================================
# Check Port Status (Socket Test)
# ============================================================================
echo ""
echo "--- Verifying ports are free ---"

# Check if port is connectable (socket test)
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
" 2>/dev/null
}

# Check both ports
PORT_50051_FREE=true
PORT_50052_FREE=true

if check_port_connectable 50051; then
    PORT_50051_FREE=false
    echo "  ⚠ Port 50051 is still connectable (may be in TIME_WAIT state)"
else
    echo "  ✓ Port 50051 is free"
fi

if check_port_connectable 50052; then
    PORT_50052_FREE=false
    echo "  ⚠ Port 50052 is still connectable (may be in TIME_WAIT state)"
else
    echo "  ✓ Port 50052 is free"
fi

# ============================================================================
# Final Process Check
# ============================================================================
echo ""
echo "--- Final process check ---"

REMAINING_PROCESSES=false

if command -v pgrep >/dev/null 2>&1; then
    # Check for any remaining processes
    REMAINING=$(pgrep -f "run_server|franka_panda|launch_robot|gripper|franka_hand" 2>/dev/null || true)
    if [ -n "$REMAINING" ]; then
        echo "  ⚠ Remaining processes found: $REMAINING"
        REMAINING_PROCESSES=true
    else
        echo "  ✓ No remaining processes found"
    fi
fi

# ============================================================================
# Summary
# ============================================================================
echo ""
echo "============================================================================"
echo "Cleanup Summary"
echo "============================================================================"

if [ "$PORT_50051_FREE" = "true" ] && [ "$PORT_50052_FREE" = "true" ] && [ "$REMAINING_PROCESSES" = "false" ]; then
    echo "✓ All processes killed successfully"
    echo "✓ All ports are free"
    exit 0
else
    echo "⚠ Some processes or ports may still be in use:"
    [ "$PORT_50051_FREE" = "false" ] && echo "  - Port 50051 may be in TIME_WAIT state (will clear automatically)"
    [ "$PORT_50052_FREE" = "false" ] && echo "  - Port 50052 may be in TIME_WAIT state (will clear automatically)"
    [ "$REMAINING_PROCESSES" = "true" ] && echo "  - Some processes may still be running"
    echo ""
    echo "Note: TIME_WAIT state typically lasts 60-120 seconds and will clear automatically."
    echo "You can wait a few minutes and run this script again if needed."
    exit 0
fi

