# ROS2 Humble with CUDA 11.8 Docker 配置

## 概述

此目录包含用于运行 ROS2 Humble 和 CUDA 11.8 的 Docker 配置文件。

## 文件说明

- **Dockerfile.ros2_cu118**: 基于 NVIDIA CUDA 11.8 和 Ubuntu 22.04 的 Docker 镜像
- **docker-compose.yaml**: Docker Compose 配置文件，包含 GPU 支持和网络配置
- **entrypoint.sh**: 容器入口脚本，自动配置 ROS2 环境
- **ros2_cu118.env**: 环境配置脚本，可手动 source 以配置环境
- **cyclonedds.xml**: CycloneDDS 配置文件
- **fastrtps_profile.xml**: FastRTPS 配置文件

## 快速开始

### 构建镜像

```bash
cd /home/yjin/repos/role_ws/src/role-ros2/docker/ros2_cu118
docker compose build
```

### 运行容器

```bash
# 进入容器（自动配置环境）
docker compose run --rm ros2_cu118 bash

# 或者跳过自动配置（更快启动）
SKIP_AUTO_ENV=true docker compose run --rm ros2_cu118 bash
```

### 构建 ROS2 工作空间

在容器内：

```bash
# 1. 如果跳过了自动配置，先 source 环境
source /app/ros2_ws/src/role-ros2/docker/ros2_cu118/ros2_cu118.env

# 2. 构建工作空间
cd /app/ros2_ws
colcon build --symlink-install

# 3. Source 工作空间
source install/setup.bash
```

## 环境变量

可以通过环境变量或 docker-compose.yaml 配置：

- `ROS_DOMAIN_ID`: ROS2 域 ID（默认: 0）
- `RMW_IMPLEMENTATION`: RMW 实现（默认: rmw_cyclonedds_cpp）
- `ROBOT_IP`: 机器人 IP 地址（默认: 172.17.0.2）
- `NUC_IP`: NUC IP 地址（默认: 172.17.0.1）
- `LAPTOP_IP`: 笔记本电脑 IP 地址（默认: 172.17.0.1）
- `ROBOT_TYPE`: 机器人类型（默认: fr3）
- `SKIP_AUTO_ENV`: 跳过自动环境配置（默认: false）

## GPU 支持

此配置已启用 NVIDIA GPU 支持。确保：

1. 已安装 NVIDIA Container Toolkit（参考 laptop_setup.sh）
2. 主机有可用的 NVIDIA GPU
3. Docker 已配置为使用 NVIDIA runtime

## 与 ros2_polymetis 的区别

- **ROS2 版本**: Humble（vs Foxy）
- **基础镜像**: NVIDIA CUDA 11.8（vs Ubuntu 20.04）
- **不包含**: libfranka、Polymetis、Micromamba/Conda
- **用途**: 适用于需要 GPU 加速但不使用 Polymetis 的场景

## 注意事项

1. **路径结构**: 构建上下文是 `role_ws` 目录，所有路径都是相对于此目录
2. **开发模式**: docker-compose.yaml 中已配置源代码挂载，支持开发时实时修改
3. **X11 转发**: 已配置 X11 转发，支持 GUI 应用程序
4. **网络模式**: 使用 host 网络模式，便于 ROS2 节点间通信

