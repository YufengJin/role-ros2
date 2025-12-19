# ROS2 跨容器通信配置指南

## 问题描述

当容器内运行 ROS2 Foxy，主机系统运行 ROS2 Humble 时，需要配置 DDS 设置以确保两者能够通信。

## 解决方案

### 1. 容器配置（已自动配置）

容器已配置以下环境变量：
- `ROS_DOMAIN_ID=0`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_LOCALHOST_ONLY=0`

### 2. 主机系统配置（ROS2 Humble）

在主机系统运行 ROS2 节点之前，**必须**设置以下环境变量：

```bash
# 设置与容器相同的 DDS 配置
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Source ROS2 Humble
source /opt/ros/humble/setup.bash  # 根据你的安装路径调整
```

### 3. 安装 FastRTPS（如果未安装）

如果主机系统没有安装 FastRTPS，需要安装：

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install ros-humble-rmw-fastrtps-cpp
```

### 4. 验证配置

#### 在容器内发布消息：

```bash
cd /path/to/ros2_ws/src/role-ros2/docker
docker compose -f docker-compose.yaml run --rm ros2_polymetis bash

# 在容器内
/usr/bin/python3 /app/ros2_ws/src/role-ros2/scripts/test_publish.py
```

#### 在主机系统订阅消息：

```bash
# 在主机终端（新开一个终端）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash

# 查看 topic 列表
ros2 topic list

# 订阅消息
ros2 topic echo /test_topic
```

### 5. 使用测试脚本

提供了一个测试脚本来简化测试：

```bash
# 在主机系统运行
cd /path/to/ros2_ws/src/role-ros2/scripts
./test_cross_container_comm.sh
```

## 故障排查

### 问题 1: 主机系统看不到容器的 topic

**检查清单：**
1. ✅ `ROS_DOMAIN_ID` 是否相同（应该是 0）
2. ✅ `RMW_IMPLEMENTATION` 是否相同（应该是 `rmw_fastrtps_cpp`）
3. ✅ `ROS_LOCALHOST_ONLY` 是否设置为 0
4. ✅ 容器是否使用 `network_mode: host`
5. ✅ 防火墙是否阻止 DDS 端口（UDP 7400-7500）

**验证命令：**
```bash
# 在容器内
echo "Container: RMW=$RMW_IMPLEMENTATION, DOMAIN=$ROS_DOMAIN_ID, LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"

# 在主机系统
echo "Host: RMW=$RMW_IMPLEMENTATION, DOMAIN=$ROS_DOMAIN_ID, LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
```

### 问题 2: FastRTPS 未安装

如果主机系统提示找不到 FastRTPS：

```bash
# 检查是否安装
dpkg -l | grep fastrtps

# 安装
sudo apt-get install ros-humble-rmw-fastrtps-cpp
```

### 问题 3: 网络问题

如果使用 `network_mode: host` 仍然无法通信：

```bash
# 检查网络接口
ip addr show

# 检查防火墙
sudo ufw status

# 临时禁用防火墙测试（不推荐生产环境）
sudo ufw disable
```

## 永久配置（可选）

如果经常需要在主机系统运行 ROS2，可以将环境变量添加到 `~/.bashrc`：

```bash
# 添加到 ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc

# 重新加载
source ~/.bashrc
```

## 注意事项

1. **DDS 版本兼容性**：
   - ROS2 Foxy 默认使用 FastRTPS
   - ROS2 Humble 默认使用 CycloneDDS
   - 两者不兼容，必须统一使用 FastRTPS

2. **网络模式**：
   - 容器使用 `network_mode: host` 以允许网络发现
   - 这允许容器和主机在同一网络上通信

3. **性能考虑**：
   - FastRTPS 在跨版本通信时性能可能不如 CycloneDDS
   - 如果可能，建议统一使用相同版本的 ROS2

## 参考

- [ROS2 DDS Configuration](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html)
- [ROS2 RMW Implementation](https://docs.ros.org/en/foxy/Guides/Working-with-multiple-RMW-implementations.html)

