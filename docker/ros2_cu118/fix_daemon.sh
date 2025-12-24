#!/bin/bash
# ROS2 Daemon 修复和清理脚本
# 用法:
#   ./fix_daemon.sh          # 清理并启动 daemon（默认）
#   ./fix_daemon.sh --cleanup-only  # 只清理，不启动 daemon

set +e  # 允许命令失败（清理时某些命令可能失败）

# 解析参数
CLEANUP_ONLY=false
if [ "$1" = "--cleanup-only" ] || [ "$1" = "-c" ]; then
    CLEANUP_ONLY=true
fi

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

if [ "$CLEANUP_ONLY" = true ]; then
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}ROS2 强制清理脚本${NC}"
    echo -e "${BLUE}========================================${NC}"
else
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}ROS2 Daemon 修复工具${NC}"
    echo -e "${BLUE}========================================${NC}"
fi
echo ""

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Step 1: Check current daemon status (only in fix mode)
if [ "$CLEANUP_ONLY" = false ]; then
    echo -e "${CYAN}[1/6] 检查当前 daemon 状态${NC}"
    if ros2 daemon status >/dev/null 2>&1; then
        echo -e "${GREEN}✓ Daemon 正在运行${NC}"
        ros2 daemon status
    else
        echo -e "${YELLOW}⚠ Daemon 未运行或有问题${NC}"
    fi
    echo ""
fi

# Step 2: Stop all existing daemon processes gracefully
if [ "$CLEANUP_ONLY" = false ]; then
    echo -e "${CYAN}[2/6] 停止所有现有的 daemon 进程${NC}"
else
    echo -e "${CYAN}[1/5] 停止 ROS2 daemon 进程${NC}"
fi

# 先尝试优雅停止
ros2 daemon stop >/dev/null 2>&1 || true
sleep 2

# 等待进程退出
for i in {1..5}; do
    if ! pgrep -f "ros2.*daemon" >/dev/null 2>&1 && ! pgrep -f "_ros2_daemon" >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

# 如果还有进程，使用 SIGTERM（更温和）
pkill -TERM -f "_ros2_daemon" >/dev/null 2>&1 || true
pkill -TERM -f "ros2.*daemon" >/dev/null 2>&1 || true
pkill -TERM -f "ros2_daemon" >/dev/null 2>&1 || true
sleep 2

# 最后才使用 SIGKILL（强制）
pkill -9 -f "_ros2_daemon" >/dev/null 2>&1 || true
pkill -9 -f "ros2.*daemon" >/dev/null 2>&1 || true
pkill -9 -f "ros2_daemon" >/dev/null 2>&1 || true
sleep 1

echo -e "${GREEN}✓ 已停止所有 daemon 进程${NC}"
echo ""

# Step 3: Clean stuck CLI and node processes (only in cleanup mode)
if [ "$CLEANUP_ONLY" = true ]; then
    echo -e "${CYAN}[2/5] 清理卡住的 ROS2 CLI 和节点进程${NC}"
    # 先尝试优雅停止节点进程
    pkill -TERM -f "demo_nodes_py" >/dev/null 2>&1 || true
    sleep 2

    # 等待进程退出
    for i in {1..3}; do
        if ! pgrep -f "demo_nodes_py" >/dev/null 2>&1; then
            break
        fi
        sleep 1
    done

    # 清理卡住的 CLI 进程（不包括 daemon）
    pkill -TERM -f "ros2 run" >/dev/null 2>&1 || true
    pkill -TERM -f "ros2 topic" >/dev/null 2>&1 || true
    pkill -TERM -f "ros2 node" >/dev/null 2>&1 || true
    sleep 2

    # 最后强制清理
    pkill -9 -f "demo_nodes_py" >/dev/null 2>&1 || true
    pkill -9 -f "ros2 run" >/dev/null 2>&1 || true
    pkill -9 -f "ros2 topic" >/dev/null 2>&1 || true
    pkill -9 -f "ros2 node" >/dev/null 2>&1 || true
    sleep 1
    echo -e "${GREEN}✓ 完成${NC}"
    echo ""
fi

# Step 4: Clean daemon state
if [ "$CLEANUP_ONLY" = false ]; then
    echo -e "${CYAN}[3/6] 清理 daemon 状态${NC}"
else
    echo -e "${CYAN}[3/5] 清理 ROS2 临时文件和日志${NC}"
fi

# ROS2 daemon stores state in ~/.ros/ros2_daemon/
# Ensure directory exists (critical for daemon to work)
mkdir -p ~/.ros/ros2_daemon
if [ -d ~/.ros/ros2_daemon ]; then
    rm -rf ~/.ros/ros2_daemon/* 2>/dev/null || true
    if [ "$CLEANUP_ONLY" = false ]; then
        echo -e "${GREEN}✓ 已清理 daemon 状态目录${NC}"
    fi
fi

# 在 cleanup 模式下，也清理其他临时文件
if [ "$CLEANUP_ONLY" = true ]; then
    rm -rf ~/.ros/log 2>/dev/null || true
    rm -rf ~/.ros/daemon 2>/dev/null || true
    echo -e "${GREEN}✓ 完成${NC}"
fi
echo ""

# Step 5: Clean DDS discovery files (only in cleanup mode)
if [ "$CLEANUP_ONLY" = true ]; then
    echo -e "${CYAN}[4/5] 清理 DDS 发现临时文件${NC}"
    rm -rf /tmp/ros_domain_id_echo* 2>/dev/null || true
    rm -rf /tmp/cyclonedds* 2>/dev/null || true
    rm -rf /tmp/dds* 2>/dev/null || true
    echo -e "${GREEN}✓ 完成${NC}"
    echo ""
fi

# Step 6: Start daemon (only in fix mode)
if [ "$CLEANUP_ONLY" = false ]; then
    echo -e "${CYAN}[4/6] 启动 ROS2 daemon${NC}"
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
    echo -e "${CYAN}[5/6] 验证 daemon 状态${NC}"
    if ros2 daemon status >/dev/null 2>&1; then
        echo -e "${GREEN}✓ Daemon 启动成功${NC}"
        ros2 daemon status
    else
        echo -e "${RED}✗ Daemon 启动失败${NC}"
        echo "  尝试手动启动..."
        ros2 daemon start --verbose
        sleep 2
        ros2 daemon status
    fi
    echo ""

    echo -e "${CYAN}[6/6] 完成${NC}"
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}修复完成${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "${YELLOW}测试命令：${NC}"
    echo "  ${CYAN}ros2 topic list${NC}"
    echo "  ${CYAN}ros2 node list${NC}"
else
    echo -e "${CYAN}[5/5] 完成${NC}"
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}清理完成！${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "${YELLOW}下一步: 重新启动 daemon${NC}"
    echo "  ${CYAN}ros2 daemon start${NC}"
    echo "  或运行: ${CYAN}./fix_daemon.sh${NC}"
    echo ""
fi
