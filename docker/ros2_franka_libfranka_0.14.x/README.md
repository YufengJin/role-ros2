# ROS2 Foxy + Polymetis Docker 环境 - libfranka 0.14.x

此目录包含用于运行 ROS2 Foxy 和 Polymetis 机器人控制的 Docker 配置文件（libfranka 0.14.x 版本）。

## 特性

- **ROS2 Foxy** + FastRTPS (与官方镜像一致)
- **Polymetis** 机器人控制框架（源码构建）
- **libfranka 0.14.1** Franka 机器人支持
  - ✅ 使用 **C++14** 标准（libfranka 默认）
  - ✅ **不需要 fmt 库**
  - ✅ 兼容机器人系统 **5.7.1**
- **Python 3.8** + PyTorch 1.13.1
- **无 conda/micromamba** - 所有包通过 apt/pip 安装
- **protobuf 3.18.1** - 与源码编译版本一致，避免版本冲突

## 快速开始

```bash
# 1. 构建镜像
cd /home/yjin/repos/role-ros2/docker/ros2_franka_libfranka_0.14.x
docker compose build

# 2. 启动容器（两种方式）

# 方式 1: 使用 docker compose run（推荐用于首次启动）
docker compose run --rm ros2_polymetis bash

# 方式 2: 使用 docker compose up + docker exec（推荐用于已运行的容器）
docker compose up -d
docker exec -it ros2_polymetis_container bash

# 3. 在容器内构建 ROS2 workspace
cd /app/ros2_ws
colcon build --symlink-install

# 4. Source workspace（构建完成后）
source install/setup.bash
```

**两种启动方式的区别：**
- `docker compose run --rm`: 每次创建新容器，退出后自动删除（适合一次性任务）
- `docker compose up -d` + `docker exec`: 容器持续运行，可多次进入（适合开发调试）

## 文件说明

| 文件 | 说明 |
|------|------|
| `Dockerfile` | Docker 镜像定义（libfranka 0.14.x，C++14，无需 fmt）|
| `docker-compose.yaml` | Docker Compose 配置 |
| `entrypoint.sh` | 容器入口脚本（自动配置环境）|
| `polymetis_ros2.env` | 环境配置脚本（可手动 source）|
| `README.md` | 本文档 |

## 版本选择

本项目支持两个 libfranka 版本，分别位于不同的目录：

| 目录 | libfranka 版本 | C++ 标准 | fmt 库 | 机器人系统版本 |
|------|----------------|----------|--------|----------------|
| `../ros2_franka_libfranka_0.14.x` | 0.14.x | C++14 | ❌ 不需要 | 5.7.1 |
| `../ros2_franka_libfranka_0.18.x` | 0.18.x | C++17 | ✅ 必需 | 5.9.0+ |

**请根据您的机器人系统版本选择对应的目录：**
- 机器人系统 **5.7.1** → 使用 `ros2_franka_libfranka_0.14.x`（当前目录）✅
- 机器人系统 **5.9.0 或更高** → 使用 `ros2_franka_libfranka_0.18.x`

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_IP` | `172.17.0.2` | Franka 机器人 IP |
| `POLYMETIS_IP` | `127.0.0.1` | Polymetis gRPC 服务器 IP |
| `ROS_DOMAIN_ID` | `0` | ROS2 域 ID |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | DDS 实现（使用 FastRTPS，与官方镜像一致）|
| `ROS_LOCALHOST_ONLY` | `0` | 是否仅本地通信（0=允许多机通信）|
| `LIBFRANKA_VERSION` | `0.14.1` | Libfranka 版本（在构建时使用）|

可在 `docker-compose.yaml` 中修改或运行时覆盖：

```bash
ROBOT_IP=192.168.1.100 docker compose run --rm ros2_polymetis bash
```

## 目录结构

```
容器内目录:
/app/ros2_ws/
├── src/role-ros2/          # role_ros2 包（bind mount）
└── install/                # 构建产物

/opt/fairo/
└── polymetis/              # Polymetis 安装目录
    └── polymetis/
        ├── python/         # Python 模块
        └── build/          # C++ 构建产物（libfranka 0.14.x with C++14）
```

## 开发工作流

### 修改代码后重新构建

```bash
# 在容器内
cd /app/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 清理重建

```bash
cd /app/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## 验证环境

```bash
# 检查 ROS2
ros2 pkg list | grep role_ros2

# 检查 Polymetis
python3 -c "from polymetis import RobotInterface; print('✅ Polymetis OK')"

# 检查 torchcontrol
python3 -c "from torchcontrol.models import RobotModelPinocchio; print('✅ torchcontrol OK')"

# 检查 ROS2 + Polymetis 兼容性
python3 -c "import rclpy; from polymetis import RobotInterface; print('✅ Both OK')"

# 验证 protobuf 版本
python3 -c "import google.protobuf; print(f'Protobuf: {google.protobuf.__version__}')"
# 应该显示: Protobuf: 3.18.1
```

## 常见问题

### 如何切换到 libfranka 0.18.x？

如果您需要 libfranka 0.18.x 版本（兼容机器人系统 5.9.0+），请使用另一个目录：

```bash
cd /home/yjin/repos/role-ros2/docker/ros2_franka_libfranka_0.18.x
docker compose build
docker compose up -d
```

### 版本兼容性检查

确保您的机器人系统版本与 libfranka 版本兼容：
- **libfranka 0.14.x** ↔ 机器人系统 **5.7.1**
- **libfranka 0.18.x** ↔ 机器人系统 **5.9.0+**

详细兼容性信息请参考 [LIBFRANKA_COMPATIBILITY.md](../../../docs/LIBFRANKA_COMPATIBILITY.md)

### Protobuf 版本冲突

如果遇到 protobuf 序列化错误（`TypeError: descriptor 'SerializeToString' ... doesn't apply to a 'NoneType' object`）：

**原因**: Python protobuf 版本与 C++ 编译版本不匹配

**解决方案**: 确保 Dockerfile 中所有 `protobuf==` 版本都设置为 `3.18.1`（与源码编译版本一致）

**验证**:
```bash
# 在容器内检查
python3 -c "import google.protobuf; print(google.protobuf.__version__)"
protoc --version
# 两者应该匹配（均为 3.18.1）
```

### 其他问题

更多常见问题请参考：
- [故障排除指南](../../TROUBLESHOOTING.md)（如果存在）
- [Docker 信号处理](./DOCKER_SIGNAL_HANDLING.md)（如果存在）

## 技术细节

### 构建配置

- **libfranka**: 版本 0.14.1，使用 C++14（默认），不需要 fmt 库
- **Polymetis**: 使用 C++14（与 libfranka 匹配，使用 Polymetis 默认设置）
- **protobuf**: 版本 3.18.1（Python 和 C++ 一致）
- **gRPC**: 版本 1.41.1

### 关键构建参数

```dockerfile
# libfranka 构建（0.14.x - 使用默认 C++14，不设置 fmt_DIR）
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    # 不设置 CMAKE_CXX_STANDARD，使用 libfranka 默认的 C++14
    # 不设置 fmt_DIR，因为 0.14.x 不需要 fmt

# Polymetis 构建（使用 Polymetis 默认的 C++14，匹配 libfranka）
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_FRANKA=ON
    # 不设置 CMAKE_CXX_STANDARD，使用 Polymetis 默认的 C++14（CMakeLists.txt line 8）
    # 不设置 fmt_DIR，因为 0.14.x 不需要 fmt
```
