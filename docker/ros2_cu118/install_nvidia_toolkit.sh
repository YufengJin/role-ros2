#!/bin/bash
# Install nvidia-container-toolkit for Docker GPU support
#
# This script installs nvidia-container-toolkit which is required
# for Docker containers to access NVIDIA GPUs.
#
# Usage:
#   sudo ./install_nvidia_toolkit.sh

set -e

echo "=========================================="
echo "NVIDIA Container Toolkit 安装脚本"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "ERROR: Please run as root (use sudo)"
    exit 1
fi

# Detect distribution
if [ -f /etc/os-release ]; then
    . /etc/os-release
    DISTRIBUTION=$ID$VERSION_ID
else
    echo "ERROR: Cannot detect Linux distribution"
    exit 1
fi

echo "Detected distribution: $DISTRIBUTION"
echo ""

# Add NVIDIA Container Toolkit repository
echo "Step 1: Adding NVIDIA Container Toolkit repository..."

# Remove existing repository files to avoid conflicts
rm -f /etc/apt/sources.list.d/nvidia-container-toolkit.list
rm -f /etc/apt/sources.list.d/nvidia-*.list 2>/dev/null || true

# Add GPG key (overwrite if exists, use batch mode to avoid prompts)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
    gpg --dearmor --batch --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Detect architecture
ARCH=$(dpkg --print-architecture)

# Create repository configuration manually (more reliable than downloading template)
# Use stable/deb path which works for all Debian/Ubuntu distributions
cat > /etc/apt/sources.list.d/nvidia-container-toolkit.list <<EOF
deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/$ARCH /
EOF

echo "✓ Repository added (distribution: $DISTRIBUTION, arch: $ARCH)"
echo ""

# Update package list
echo "Step 2: Updating package list..."
apt-get update

echo "✓ Package list updated"
echo ""

# Install nvidia-container-toolkit
echo "Step 3: Installing nvidia-container-toolkit..."
apt-get install -y nvidia-container-toolkit

echo "✓ nvidia-container-toolkit installed"
echo ""

# Configure Docker
echo "Step 4: Configuring Docker..."
nvidia-ctk runtime configure --runtime=docker

echo "✓ Docker configured"
echo ""

# Restart Docker daemon
echo "Step 5: Restarting Docker daemon..."
systemctl restart docker

echo "✓ Docker daemon restarted"
echo ""

# Verify installation
echo "Step 6: Verifying installation..."
echo ""
echo "Testing GPU access..."
if docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi >/dev/null 2>&1; then
    echo "✓ GPU access verified successfully!"
    echo ""
    echo "=========================================="
    echo "安装完成！"
    echo "=========================================="
    echo ""
    echo "现在可以使用 docker-compose.yaml 启动容器了："
    echo "  cd /home/yjin/repos/ros2_ws/src/role-ros2/docker/ros2_cu118"
    echo "  docker compose up -d"
else
    echo "⚠️  GPU 测试失败，但工具包已安装"
    echo "请检查："
    echo "  1. NVIDIA 驱动是否正确安装: nvidia-smi"
    echo "  2. Docker 是否正确重启: sudo systemctl restart docker"
    echo "  3. 重新运行测试: docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi"
fi

