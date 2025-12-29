# ROS2 Foxy + Polymetis Docker 环境

此目录包含用于运行 ROS2 Foxy 和 Polymetis 机器人控制的 Docker 配置文件。

## 特性

- **ROS2 Foxy** + FastRTPS (与官方镜像一致)
- **Polymetis** 机器人控制框架（源码构建）
- **libfranka 0.14.1** Franka 机器人支持
- **Python 3.8** + PyTorch 1.13.1
- **无 conda/micromamba** - 所有包通过 apt/pip 安装

## 快速开始

```bash
# 1. 构建镜像
cd /home/yjin/repos/role_ws/src/role-ros2/docker/ros2_polymetis
docker compose build ros2_polymetis

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
| `Dockerfile.ros2_polymetis` | Docker 镜像定义 |
| `docker-compose.yaml` | Docker Compose 配置 |
| `entrypoint.sh` | 容器入口脚本（自动配置环境）|
| `polymetis_ros2.env` | 环境配置脚本（可手动 source）|
| `cyclonedds.xml` | CycloneDDS 配置文件（备用，当前使用 FastRTPS）|
| `fastrtps_profile.xml` | FastRTPS 配置文件（备用）|

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_IP` | `172.17.0.2` | Franka 机器人 IP |
| `POLYMETIS_IP` | `127.0.0.1` | Polymetis gRPC 服务器 IP |
| `ROS_DOMAIN_ID` | `0` | ROS2 域 ID |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | DDS 实现（使用 FastRTPS，与官方镜像一致）|
| `ROS_LOCALHOST_ONLY` | `0` | 是否仅本地通信（0=允许多机通信）|

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
        └── build/          # C++ 构建产物
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
```

## 跨容器通信（ROS2 Foxy <-> Humble）

容器使用 **FastRTPS** 作为默认 DDS 实现，与官方 ROS2 镜像一致。

### 网络配置

- **网络模式**: `host` - 使用主机网络，便于 ROS2 节点间通信
- **DDS 实现**: FastRTPS (`rmw_fastrtps_cpp`) - ros-foxy-desktop 的默认实现
- **多机通信**: 确保所有设备使用相同的 `ROS_DOMAIN_ID`

### 如果需要切换到 CycloneDDS（跨版本兼容）

如果需要与使用 CycloneDDS 的容器通信，可以临时切换：

```bash
# 在容器内
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///app/ros2_ws/src/role-ros2/docker/ros2_polymetis/cyclonedds.xml
ros2 daemon stop
ros2 daemon start
```

**注意**: 所有节点必须使用相同的 RMW 实现才能通信。

## 使用 docker exec 管理容器

### 检查容器状态

```bash
# 查看所有容器（包括停止的）
docker ps -a | grep ros2_polymetis

# 只查看运行中的容器
docker ps | grep ros2_polymetis
```

### 启动容器

如果容器未运行，使用以下命令启动：

```bash
cd /home/yjin/repos/role_ws/src/role-ros2/docker/ros2_polymetis
docker compose up -d
```

### 进入容器

```bash
# 进入交互式 bash shell
docker exec -it ros2_polymetis_container bash

# 执行单个命令（不进入交互式 shell）
docker exec ros2_polymetis_container ros2 node list

# 在容器中执行命令并保持环境
docker exec -it ros2_polymetis_container bash -c "cd /app/ros2_ws && source install/setup.bash && ros2 topic list"
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
docker exec ros2_polymetis_container ros2 node list

# 列出所有话题
docker exec ros2_polymetis_container ros2 topic list

# 查看话题内容
docker exec ros2_polymetis_container ros2 topic echo /test/string_topic

# 运行 ROS2 节点
docker exec -it ros2_polymetis_container bash -c "cd /app/ros2_ws && source install/setup.bash && ros2 run role_ros2 test_service_server"
```

### ROS2 常用命令测试

在容器内直接运行 ROS2 命令进行测试：

```bash
# 进入容器
docker exec -it ros2_polymetis_container bash

# 测试基本命令
ros2 node list
ros2 topic list
ros2 service list

# 运行测试节点
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener &
sleep 3
ros2 node list
```

**提示：**
- 使用 **Ctrl+C** 正常终止命令
- 如果 Ctrl+C 无效，使用 **Ctrl+\\** (SIGQUIT) 强制终止
- 使用 `timeout` 命令避免命令卡住：`timeout 5 ros2 topic list`

### Docker 容器内信号处理（Ctrl+C 问题）

**为什么 Ctrl+C 可能不管用？**

Docker 容器内 Ctrl+C 失效的常见原因：

1. **缺少 `-it` 标志** - 必须使用 `docker exec -it` 才能正确传递信号
2. **进程组问题** - 子进程可能在不同的进程组，信号无法传递
3. **信号传递延迟** - Docker daemon 转发信号时的延迟
4. **Python 信号缓冲** - ROS2 Python 节点的信号处理延迟

**解决方案：**

```bash
# ✅ 正确：使用 -it 标志
docker exec -it ros2_polymetis_container bash

# ❌ 错误：缺少 -it，信号无法传递
docker exec ros2_polymetis_container bash
```

如果 Ctrl+C 无效：
- 使用 **Ctrl+\\** (SIGQUIT) 强制终止（更可靠）
- 使用 `timeout` 命令设置超时
- 在另一个终端：`docker stop ros2_polymetis_container`

详细说明请参考 [DOCKER_SIGNAL_HANDLING.md](./DOCKER_SIGNAL_HANDLING.md)

## 故障排除

### 快速诊断

如果遇到问题，首先运行诊断脚本：

```bash
cd /home/yjin/repos/role_ws/src/role-ros2/scripts
./diagnose_ros2.sh ros2_polymetis_container
```

### 常见问题

#### Q: `ros2 topic list` 没有反应或卡住

**A**: 这通常是 ROS2 daemon 或 DDS 配置问题。

快速修复：
```bash
# 1. 检查环境变量
docker exec ros2_polymetis_container bash -c "echo \$ROS_DOMAIN_ID \$RMW_IMPLEMENTATION"
# 应该显示: 0 rmw_fastrtps_cpp

# 2. 重启 daemon
docker exec ros2_polymetis_container bash -c "ros2 daemon stop && sleep 2 && ros2 daemon start"

# 3. 使用超时测试
docker exec ros2_polymetis_container bash -c "cd /app/ros2_ws && source install/setup.bash && timeout 5 ros2 topic list"
```

#### Q: `robot_state_publisher` 或 `polymetis_bridge` 进程崩溃（SIGSEGV, exit code -11）

**A**: 这通常是由于在节点初始化时执行阻塞操作导致的。

**解决方案**：`polymetis_bridge_node` 现在使用延迟 reset 机制来避免这个问题。

```bash
# 禁用自动 reset（如果仍有问题）
ros2 run role_ros2 polymetis_bridge --ros-args -p auto_reset_on_startup:=false

# 或者增加延迟时间
ros2 run role_ros2 polymetis_bridge --ros-args -p auto_reset_delay:=5.0

# 手动调用 reset service
ros2 service call /polymetis/reset role_ros2/srv/Reset "{randomize: false}"
```

**说明**：
- 默认情况下，节点会在启动后 2 秒自动 reset 机器人
- 这避免了在初始化阶段执行阻塞操作导致的 SIGSEGV
- 可以通过参数 `auto_reset_on_startup` 和 `auto_reset_delay` 控制行为

#### Q: Ctrl+C 无法终止命令

**A**: 这是一个常见的 Docker 信号处理问题。解决方法：

1. **确保使用 `-it` 标志**：
   ```bash
   docker exec -it ros2_polymetis_container bash
   ```

2. **使用 `Ctrl+\` (SIGQUIT) 强制终止**（比 Ctrl+C 更可靠）：
   ```bash
   # 在运行命令时按 Ctrl+\
   ```

3. **使用超时命令**：
   ```bash
   timeout 10 ros2 topic list
   ```

4. **重启容器**（最后手段）：
   ```bash
   docker compose restart ros2_polymetis
   ```

详细说明请参考：
- [DOCKER_SIGNAL_HANDLING.md](./DOCKER_SIGNAL_HANDLING.md) - Docker 信号处理详细说明
- [故障排除指南](../TROUBLESHOOTING.md) - 通用故障排除

#### Q: 测试脚本显示 "No executable found"

**A**: 需要重新构建工作空间：

```bash
docker exec -it ros2_polymetis_container bash
cd /app/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables role_ros2 | grep test
```

### ros2 命令找不到

```bash
source /opt/ros/foxy/setup.bash
```

### role_ros2 模块找不到

```bash
cd /app/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 新增消息/服务导入失败

重新构建 workspace：
```bash
cd /app/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Polymetis 导入失败

检查 PYTHONPATH：
```bash
export PYTHONPATH=/opt/fairo/polymetis/polymetis/python:$PYTHONPATH
```

## 最新修复（2024）

### DDS 实现切换（关键修复）
- ✅ **切换到 FastRTPS** (`rmw_fastrtps_cpp`)，与官方 ROS2 Foxy 镜像一致
- ✅ FastRTPS 是 ros-foxy-desktop 的默认实现，无需额外配置
- ✅ 保留 `cyclonedds.xml` 作为备用配置，便于将来切换
- ✅ 更新了所有配置文件（docker-compose.yaml, entrypoint.sh, polymetis_ros2.env）

### 配置优化
- ✅ 统一使用 FastRTPS 作为默认 DDS 实现
- ✅ 简化了环境变量配置
- ✅ 移除了临时测试脚本，保持代码库整洁

### Reset 功能增强（2024-12-29）
- ✅ **延迟自动 reset**：避免初始化时的 SIGSEGV 问题
- ✅ **支持随机化 reset**：Reset service 现在支持 `randomize` 参数
- ✅ **可配置参数**：`auto_reset_on_startup` 和 `auto_reset_delay` 参数
- ✅ **与 robot_env.py 一致**：使用相同的 reset_joints 角度和噪声范围
