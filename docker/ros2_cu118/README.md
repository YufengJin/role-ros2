# ROS2 Humble with CUDA 11.8 Docker 配置

## 概述

此目录包含用于运行 ROS2 Humble 和 CUDA 11.8 的 Docker 配置文件，适用于需要 GPU 加速的机器人学习场景。

## 文件说明

- **Dockerfile.ros2_cu118**: 基于 NVIDIA CUDA 11.8 和 Ubuntu 22.04 的 Docker 镜像
- **docker-compose.yaml**: Docker Compose 配置文件，包含 GPU 支持、网络配置和开发模式挂载
- **entrypoint.sh**: 容器入口脚本，自动配置 ROS2 环境
- **ros2_cu118.env**: 环境配置脚本，包含额外的环境变量设置
- **cyclonedds.xml**: CycloneDDS 配置文件
- **fastrtps_profile.xml**: FastRTPS 配置文件

## 版本信息

- **ROS2**: Humble Hawksbill
- **ZED SDK**: 4.1 (CUDA 11.8)
- **zed-ros2-wrapper**: humble-v4.1.4 (与 ZED SDK 4.1 兼容)
- **基础镜像**: nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

## 快速开始

### 1. 构建镜像

```bash
cd /home/yjin/repos/role_ws/src/role-ros2/docker/ros2_cu118
docker compose build
```

### 2. 运行容器

```bash
# 进入容器（自动配置环境）
docker compose run --rm ros2_cu118 bash
```

容器启动时会自动：
- Source ROS2 Humble 环境
- Source 工作空间环境（如果已构建）
- Source 自定义环境配置（ros2_cu118.env）
- 设置所有必要的环境变量

### 3. 构建工作空间

在容器内：

```bash
# 进入工作空间目录
cd /app/ros2_ws

# 构建所有包（包括 role_ros2，通过 volume mount 挂载）
colcon build --symlink-install

# 如果只需要构建特定包
colcon build --packages-select role_ros2 --symlink-install
```

## 开发模式

### 源代码挂载

`docker-compose.yaml` 已配置将 `role-ros2` 源代码目录挂载到容器中：

```yaml
volumes:
  - ../../:/app/ros2_ws/src/role-ros2:rw
```

这意味着：
- ✅ 可以在容器外部编辑代码
- ✅ 修改会立即反映到容器内
- ✅ 在容器内编译，无需重新构建镜像
- ✅ 支持热重载和快速迭代开发

### 工作流程

1. **在容器外编辑代码**：使用你喜欢的编辑器修改 `role-ros2` 源代码
2. **在容器内编译**：进入容器运行 `colcon build`
3. **测试运行**：在容器内运行 ROS2 节点进行测试

## 环境变量

可以通过环境变量或 `docker-compose.yaml` 配置：

| 变量名 | 默认值 | 说明 |
|--------|--------|------|
| `ROS_DOMAIN_ID` | `0` | ROS2 域 ID |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | RMW 实现 |
| `ROS_LOCALHOST_ONLY` | `0` | 是否仅本地通信 |
| `ROBOT_IP` | `172.17.0.2` | 机器人 IP 地址 |
| `NUC_IP` | `172.17.0.1` | NUC IP 地址 |
| `LAPTOP_IP` | `172.17.0.1` | 笔记本电脑 IP 地址 |
| `ROBOT_TYPE` | `fr3` | 机器人类型 |
| `CUDA_VISIBLE_DEVICES` | - | 可见的 CUDA 设备 |

### 自定义环境变量

在运行容器时设置：

```bash
ROS_DOMAIN_ID=1 docker compose run --rm ros2_cu118 bash
```

或在 `docker-compose.yaml` 中修改默认值。

## GPU 支持

此配置已启用 NVIDIA GPU 支持。确保：

1. **已安装 NVIDIA Container Toolkit**
   ```bash
   # 验证安装
   nvidia-ctk --version
   ```

2. **主机有可用的 NVIDIA GPU**
   ```bash
   # 验证 GPU 可见
   nvidia-smi
   ```

3. **Docker 已配置为使用 NVIDIA runtime**
   ```bash
   # 验证配置
   docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
   ```

## 网络配置

- **网络模式**: `host` - 使用主机网络，便于 ROS2 节点间通信
- **DDS 配置**: 支持 CycloneDDS 和 FastRTPS
  - CycloneDDS: 通过 `cyclonedds.xml` 配置
  - FastRTPS: 通过 `fastrtps_profile.xml` 配置

## 与 ros2_polymetis 的区别

| 特性 | ros2_cu118 | ros2_polymetis |
|------|------------|----------------|
| ROS2 版本 | Humble | Foxy |
| 基础镜像 | NVIDIA CUDA 11.8 | Ubuntu 20.04 |
| GPU 支持 | ✅ | ❌ |
| ZED SDK | ✅ 4.1 | ❌ |
| Polymetis | ❌ | ✅ |
| libfranka | ❌ | ✅ |
| 用途 | GPU 加速场景 | 机器人控制场景 |

## 常见问题

### Q: 构建时出现链接错误（undefined reference to cuvidDestroyVideoParser）

**A**: 这是正常的。NVIDIA 驱动库（`libcuda.so`、`libnvcuvid.so` 等）是运行时库，由 NVIDIA 驱动在运行时提供。Dockerfile 已配置链接器标志 `-Wl,--allow-shlib-undefined` 来允许这些符号在运行时解析。

### Q: 如何更改 zed-ros2-wrapper 版本？

**A**: 在构建时指定版本：

```bash
docker compose build --build-arg ZED_ROS2_WRAPPER_VERSION=<version>
```

### Q: 如何跳过自动环境配置？

**A**: 不再支持跳过。entrypoint.sh 会自动配置所有环境。如果需要自定义，可以修改 `ros2_cu118.env` 文件。

### Q: 代码修改后需要重新构建镜像吗？

**A**: 不需要。`role-ros2` 源代码通过 volume mount 挂载，修改代码后只需在容器内重新编译：

```bash
cd /app/ros2_ws
colcon build --packages-select role_ros2 --symlink-install
```

## 注意事项

1. **路径结构**: 构建上下文是 `role_ws` 目录，所有路径都是相对于此目录
2. **开发模式**: `docker-compose.yaml` 中已配置源代码挂载，支持开发时实时修改
3. **X11 转发**: 已配置 X11 转发，支持 GUI 应用程序
4. **网络模式**: 使用 host 网络模式，便于 ROS2 节点间通信
5. **构建优化**: 镜像中只构建 `zed-ros2-wrapper`，`role_ros2` 通过 volume mount 挂载，便于开发

## 目录结构

```
ros2_cu118/
├── Dockerfile.ros2_cu118      # Docker 镜像定义
├── docker-compose.yaml         # Docker Compose 配置
├── entrypoint.sh               # 容器入口脚本
├── ros2_cu118.env              # 环境变量配置
├── cyclonedds.xml              # CycloneDDS 配置
├── fastrtps_profile.xml        # FastRTPS 配置
└── README.md                   # 本文档
```

## 参考资源

- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [ZED SDK 文档](https://www.stereolabs.com/docs/)
- [zed-ros2-wrapper GitHub](https://github.com/stereolabs/zed-ros2-wrapper)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)
