# ROS2 Humble with CUDA 11.8 Docker 配置

## 概述

此目录包含用于运行 ROS2 Humble 和 CUDA 11.8 的 Docker 配置文件，适用于需要 GPU 加速的机器人学习场景。

## 文件说明

- **Dockerfile.ros2_cu118**: 基于 NVIDIA CUDA 11.8 和 Ubuntu 22.04 的 Docker 镜像
- **docker-compose.yaml**: Docker Compose 配置文件，包含 GPU 支持、网络配置和开发模式挂载
- **entrypoint.sh**: 容器入口脚本，自动配置 ROS2 环境并管理 daemon
- **ros2_cu118.env**: 环境配置脚本，包含 ROS2 和 DDS 环境变量设置
- **fix_daemon.sh**: ROS2 daemon 修复和清理脚本（合并了原 force_cleanup.sh 的功能）
- **cyclonedds.xml**: CycloneDDS 配置文件（备用，当前使用 FastRTPS）

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

有两种方式运行容器：

#### 方式 1: 使用 docker compose run（推荐用于首次启动）

```bash
# 启动并进入容器（自动配置环境）
docker compose run --rm ros2_cu118 bash
```

#### 方式 2: 使用 docker compose up + docker exec（推荐用于已运行的容器）

```bash
# 1. 启动容器（后台运行）
docker compose up -d

# 2. 检查容器状态
docker ps | grep ros2_cu118_container

# 3. 进入已运行的容器
docker exec -it ros2_cu118_container bash
```

**两种方式的区别：**
- `docker compose run --rm`: 每次创建新容器，退出后自动删除（适合一次性任务）
- `docker compose up -d` + `docker exec`: 容器持续运行，可多次进入（适合开发调试）

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
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | RMW 实现（使用 FastRTPS，与官方镜像一致） |
| `ROS_LOCALHOST_ONLY` | `0` | 是否仅本地通信（0=允许多机通信） |
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
- **PID 模式**: `host` - 使用主机 PID 命名空间，确保 ROS2 daemon 正确运行
- **IPC 模式**: `host` - 使用主机 IPC 命名空间，支持 DDS zero-copy 和共享内存
- **共享内存**: `/dev/shm` 已挂载，用于 FastRTPS 性能优化
- **DDS 实现**: 使用 **FastRTPS** (`rmw_fastrtps_cpp`)，与官方 ROS2 Humble 镜像一致
  - FastRTPS 是 ros-humble-desktop 的默认实现，无需额外配置
  - 支持多机通信和多网卡环境
  - 性能优化：使用共享内存进行零拷贝通信
  - **注意**: `cyclonedds.xml` 保留作为备用配置，如需切换回 CycloneDDS 可修改 `RMW_IMPLEMENTATION`

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

## 使用 docker exec 管理容器

### 检查容器状态

```bash
# 查看所有容器（包括停止的）
docker ps -a | grep ros2_cu118

# 只查看运行中的容器
docker ps | grep ros2_cu118
```

### 启动容器

如果容器未运行，使用以下命令启动：

```bash
cd /home/yjin/repos/role_ws/src/role-ros2/docker/ros2_cu118
docker compose up -d
```

### 进入容器

```bash
# 进入交互式 bash shell
docker exec -it ros2_cu118_container bash

# 执行单个命令（不进入交互式 shell）
docker exec ros2_cu118_container ros2 node list

# 在容器中执行命令并保持环境
docker exec -it ros2_cu118_container bash -c "cd /app/ros2_ws && source install/setup.bash && ros2 topic list"
```

### 停止容器

```bash
# 停止容器（但保留容器）
docker compose stop

# 停止并删除容器
docker compose down
```

### 查看容器日志

```bash
# 查看实时日志
docker compose logs -f

# 查看最近 100 行日志
docker compose logs --tail=100
```

### 在容器中运行 ROS2 命令

```bash
# 列出所有节点
docker exec ros2_cu118_container ros2 node list

# 列出所有话题
docker exec ros2_cu118_container ros2 topic list

# 查看话题内容
docker exec ros2_cu118_container ros2 topic echo /test/string_topic

# 运行 ROS2 节点
docker exec -it ros2_cu118_container bash -c "cd /app/ros2_ws && source install/setup.bash && ros2 run role_ros2 test_topic_publisher"
```

## 故障排除

### 快速诊断

如果遇到问题，首先运行诊断脚本：

```bash
cd /home/yjin/repos/role_ws/src/role-ros2/scripts
./diagnose_ros2.sh ros2_cu118_container
```

### 常见问题

#### Q: `ros2 topic list` 没有反应或卡住

**A**: 这通常是 ROS2 daemon 或 DDS 配置问题。已改进 entrypoint 脚本以自动处理 daemon 问题。

快速修复：
```bash
# 方法 1: 重启容器（推荐，会自动修复 daemon）
docker compose restart ros2_cu118

# 方法 2: 手动修复 daemon（推荐使用 fix_daemon.sh）
docker exec ros2_cu118_container bash /app/ros2_ws/src/role-ros2/docker/ros2_cu118/fix_daemon.sh

# 方法 3: 只清理不启动（如果 daemon 完全卡死，需要先清理）
docker exec ros2_cu118_container bash /app/ros2_ws/src/role-ros2/docker/ros2_cu118/fix_daemon.sh --cleanup-only
# 然后启动 daemon
docker exec ros2_cu118_container bash /app/ros2_ws/src/role-ros2/docker/ros2_cu118/fix_daemon.sh

# 方法 4: 检查环境变量
docker exec ros2_cu118_container bash -c "echo \$ROS_DOMAIN_ID \$RMW_IMPLEMENTATION"

# 方法 5: 使用超时测试
docker exec ros2_cu118_container bash -c "cd /app/ros2_ws && source install/setup.bash && timeout 5 ros2 topic list"
```

**为什么 daemon 停止后无法重启？**
- 原因：
  1. Daemon 状态目录 `~/.ros/ros2_daemon/` 不存在，导致 daemon 无法存储状态
  2. Daemon 状态文件可能被锁定或损坏
  3. 进程未完全终止
- 解决方案：
  - entrypoint 脚本现在会自动创建 daemon 目录（如果不存在）
  - 自动清理状态文件并强制终止进程
  - 支持重试机制（最多 3 次）
  - `fix_daemon.sh` 脚本也会确保目录存在

#### Q: `ros2 node list` 显示为空，但进程在运行

**A**: 这通常是 ROS2 daemon 或 RMW 实现配置问题。

**检查方法：**
```bash
# 1. 检查 RMW 实现是否正确
docker exec ros2_cu118_container bash -c "echo \$RMW_IMPLEMENTATION"
# 应该显示: rmw_fastrtps_cpp

# 2. 检查 daemon 状态
docker exec ros2_cu118_container ros2 daemon status

# 3. 检查节点进程
docker exec ros2_cu118_container ps aux | grep ros2
```

**修复步骤：**
```bash
# 方法 1: 使用修复脚本（推荐）
docker exec ros2_cu118_container bash /app/ros2_ws/src/role-ros2/docker/ros2_cu118/fix_daemon.sh

# 方法 2: 手动修复
docker exec -it ros2_cu118_container bash
source /app/ros2_ws/src/role-ros2/docker/ros2_cu118/ros2_cu118.env
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 3
ros2 node list
```

**根本原因：**
- Daemon 和节点必须使用相同的 RMW 实现（当前为 `rmw_fastrtps_cpp`）
- Daemon 状态目录可能损坏或不存在
- 环境变量未正确设置（新 shell 会自动 source `ros2_cu118.env`）

#### Q: Ctrl+C 无法终止命令

**A**: 已修复 entrypoint.sh 以支持信号处理。如果仍有问题：

1. 使用 `Ctrl+\` (SIGQUIT) 强制终止
2. 重启容器：`docker compose restart ros2_cu118`
3. 使用超时命令：`timeout 10 <command>`

详细说明请参考 [故障排除指南](../TROUBLESHOOTING.md)。

#### Q: 测试脚本显示 "No executable found"

**A**: 需要重新构建工作空间：

```bash
docker exec -it ros2_cu118_container bash
cd /app/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables role_ros2 | grep test
```

### Q: 构建时出现链接错误（undefined reference to cuvidDestroyVideoParser）

**A**: 这是正常的。NVIDIA 驱动库（`libcuda.so`、`libnvcuvid.so` 等）是运行时库，由 NVIDIA 驱动在运行时提供。Dockerfile 已配置链接器标志 `-Wl,--allow-shlib-undefined` 来允许这些符号在运行时解析。

### Q: `colcon build` 时出现警告 "The path '/app/ros2_ws/install/xxx' doesn't exist"

**A**: 这是正常现象，可以安全忽略。这些警告出现的原因是：

1. **原因**：删除 `install/` 目录后，环境变量（`AMENT_PREFIX_PATH`、`CMAKE_PREFIX_PATH`）中仍保留旧的路径
2. **影响**：不影响构建，colcon 会自动忽略不存在的路径
3. **解决方法**（可选）：
   ```bash
   # 方法 1: 重新启动 shell（最简单）
   exit
   docker exec -it ros2_cu118_container bash
   
   # 方法 2: 手动清理环境变量
   unset AMENT_PREFIX_PATH
   unset CMAKE_PREFIX_PATH
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

**注意**：构建完成后，这些警告会自动消失，因为新的 `install/` 目录会被创建。

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
├── docker-compose.yaml         # Docker Compose 配置（包含 pid: host, ipc: host）
├── entrypoint.sh               # 容器入口脚本（自动配置环境并管理 daemon）
├── ros2_cu118.env              # 环境变量配置（ROS2 和 DDS 设置）
├── fix_daemon.sh               # Daemon 修复和清理脚本（支持 --cleanup-only 模式）
├── cyclonedds.xml              # CycloneDDS 配置文件（备用，当前使用 FastRTPS）
└── README.md                   # 本文档
```

## 最新修复（2024）

### 1. DDS 实现切换（关键修复）
- ✅ **切换到 FastRTPS** (`rmw_fastrtps_cpp`)，与官方 ROS2 Humble 镜像一致
- ✅ 解决了节点无法注册的问题（CycloneDDS 配置问题）
- ✅ FastRTPS 是 ros-humble-desktop 的默认实现，无需额外配置
- ✅ 保留 `cyclonedds.xml` 作为备用配置，便于将来切换

### 2. ROS2 Daemon 管理改进
- ✅ 添加了 `pid: host` 配置，确保 daemon PID 正确检测
- ✅ 添加了 `ipc: host` 配置，支持 DDS zero-copy
- ✅ 改进了 entrypoint.sh，实现健壮的 daemon 管理：
  - **自动创建 daemon 目录**（`~/.ros/ros2_daemon/`），解决 daemon 无法启动的问题
  - 自动清理损坏的状态文件
  - 强制终止卡住的进程
  - 重试机制（最多 3 次）
  - 更好的错误处理和日志
- ✅ 改进了 `fix_daemon.sh`，确保 daemon 目录存在并正确配置环境变量

### 3. Docker Compose 配置优化
- ✅ 添加 `pid: host` - 解决 daemon PID 检测问题
- ✅ 添加 `ipc: host` - 解决 DDS 共享内存问题
- ✅ 添加 `/dev/shm` 挂载 - 优化 FastRTPS 性能

### 4. 环境变量管理改进
- ✅ **entrypoint.sh 现在会自动 source `ros2_cu118.env`**，确保环境变量正确设置
- ✅ **Dockerfile 在 `.bashrc` 中添加了 source `ros2_cu118.env`**，确保每次新 shell 启动时都自动设置环境变量
- ✅ 合并了 `force_cleanup.sh` 到 `fix_daemon.sh`，支持 `--cleanup-only` 模式

## 参考资源

- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [ZED SDK 文档](https://www.stereolabs.com/docs/)
- [zed-ros2-wrapper GitHub](https://github.com/stereolabs/zed-ros2-wrapper)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)
