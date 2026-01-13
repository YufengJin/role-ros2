# ROS2 CUDA 11.8 + Polymetis (libfranka 0.18.x) Docker 配置

## 概述

此目录包含用于同时运行 ROS2 Humble with CUDA 11.8 和 ROS2 Polymetis with libfranka 0.18.x 的组合 Docker Compose 配置。

这个配置启动两个容器：
- **ros2_cu118**: ROS2 Humble with CUDA 11.8 和 ZED SDK（用于相机支持）
- **ros2_polymetis**: ROS2 Polymetis with libfranka 0.18.x（用于机器人控制）

两个容器通过 `network_mode: host` 共享网络，使用相同的 `ROS_DOMAIN_ID` 进行 ROS2 通信。

## 版本信息

- **ROS2**: Humble Hawksbill
- **ZED SDK**: 4.1 (CUDA 11.8)
- **zed-ros2-wrapper**: humble-v4.1.4
- **libfranka**: 0.18.2
- **Polymetis**: Compatible with robot system 5.9.0+
- **C++ Standard**: C++17 (requires fmt library)

## 快速开始

### 1. 构建镜像

```bash
cd /home/yjin/repos/role-ros2/docker/ros2_cu118_franka_0.18.x
docker-compose build
```

### 2. 启动容器

```bash
# 启动两个容器
docker-compose up -d

# 查看日志
docker-compose logs -f

# 进入 ros2_cu118 容器
docker-compose exec ros2_cu118 bash

# 进入 ros2_polymetis 容器
docker-compose exec ros2_polymetis bash
```

### 3. 环境变量配置

可以通过环境变量或 `.env` 文件配置：

```bash
# 设置机器人 IP
export ROBOT_IP=172.17.0.2

# 设置 ROS2 Domain ID（两个容器必须相同）
export ROS_DOMAIN_ID=0

# 设置机器人类型
export ROBOT_TYPE=fr3

# 启动
docker-compose up -d
```

### 4. 验证 ROS2 通信

在两个容器中分别运行：

```bash
# 在 ros2_cu118 容器中
ros2 topic list

# 在 ros2_polymetis 容器中
ros2 topic list
```

应该能看到相同的 topic 列表，说明两个容器可以正常通信。

## 使用场景

这个组合配置适用于：
- 需要同时使用 ZED 相机和 Franka 机器人的场景
- 机器人学习任务（需要相机感知和机器人控制）
- 多传感器融合应用
- 需要 libfranka 0.18.x 的新特性（C++17, fmt library）

## 注意事项

1. **ROS_DOMAIN_ID**: 两个容器必须使用相同的 `ROS_DOMAIN_ID` 才能通信
2. **网络模式**: 使用 `host` 网络模式，容器直接使用主机网络
3. **GPU 支持**: ros2_cu118 需要 NVIDIA GPU 和 nvidia-container-toolkit
4. **实时性**: ros2_polymetis 需要实时调度能力（已配置 `cap_add: SYS_NICE`）
5. **设备访问**: 两个容器都需要访问 `/dev` 设备（相机和机器人硬件）
6. **C++17**: 此版本需要 C++17 支持，确保代码兼容

## 停止容器

```bash
# 停止并删除容器
docker-compose down

# 停止但保留容器
docker-compose stop
```

## 故障排除

### 容器无法通信

1. 检查 `ROS_DOMAIN_ID` 是否相同
2. 检查 `RMW_IMPLEMENTATION` 是否相同（都是 `rmw_fastrtps_cpp`）
3. 检查网络模式是否为 `host`

### GPU 不可用

1. 检查 nvidia-container-toolkit 是否安装
2. 检查 NVIDIA 驱动是否正常
3. 运行 `nvidia-smi` 验证 GPU 可用性

### 实时调度失败

1. 检查容器是否以 `privileged` 模式运行
2. 检查 `cap_add: SYS_NICE` 是否设置
3. 检查 `ulimits` 配置是否正确

### C++17 编译错误

1. 确保代码使用 C++17 标准
2. 检查是否包含 fmt library（libfranka 0.18.x 需要）
