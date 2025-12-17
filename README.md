# DROID ROS2 Package

这是 DROID 机器人平台的 ROS2 模块化包。

## 版本信息

- **ROS2 发行版**: Humble Hawksbill
- **zed-ros2-wrapper 版本**: humble-v5.1.0 (tag: `humble-v5.1.0`)
  - 仓库: https://github.com/stereolabs/zed-ros2-wrapper.git
  - 路径: `ros2_ws/src/zed-ros2-wrapper`
- **franka_description 版本**: 0.5.1 (tag: `0.5.1`)
  - 仓库: https://github.com/frankarobotics/franka_description.git
  - 路径: `ros2_ws/src/franka_description`
- **franka_ros2 版本**: b79ce40 (commit: `b79ce40`)
  - 仓库: https://github.com/souljaboy764/franka_ros2.git
  - 路径: `ros2_ws/src/franka_ros2`
- **droid_ros2 版本**: 1.0.0

## 功能

- 启动两个 ZED 相机（hand_camera 和 static_camera）
- 自动配置相机序列号
- 禁用 IMU 数据发布
- Oculus Quest 控制器数据发布节点
- Franka 机械臂 ROS2 支持（通过 franka_ros2 和 franka_description）

## 相机配置

- **Hand Camera**: 序列号 `11022812`（安装在机械臂末端）
- **Static Camera**: 序列号 `24285872`（固定位置）

## 环境设置

### 前置要求

1. **安装 Conda**（如果尚未安装）：
   - 从 [Conda 官网](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html) 安装 Miniconda 或 Anaconda

2. **创建并激活 Conda 环境**：
```bash
# 注意：ROS2 Humble 需要 Python 3.10
# 如果已有 Python 3.8 环境，请参考 MIGRATION_PYTHON310.md 进行迁移
conda create -n "robot" python=3.10
conda activate robot
```

3. **安装项目依赖到 Conda 环境**：
```bash
# 确保在项目根目录
cd ~/repos/droid

# 安装 Oculus Reader
pip install -e ./droid/oculus_reader

# 安装主项目
pip install -e .

# 安装 dm-robotics 相关依赖（避免依赖冲突，使用 --no-deps）
pip install dm-robotics-moma==0.5.0 --no-deps
pip install dm-robotics-transformations==0.5.0 --no-deps
pip install dm-robotics-agentflow==0.5.0 --no-deps
pip install dm-robotics-geometry==0.5.0 --no-deps
pip install dm-robotics-manipulation==0.5.0 --no-deps
pip install dm-robotics-controllers==0.5.0 --no-deps
```

4. **安装系统依赖**：
```bash
sudo apt update
sudo apt install -y build-essential
gcc --version  # 验证安装
```

5. **安装 ROS2 Humble**：
   - 按照 [ROS2 Humble 官方安装指南](https://docs.ros.org/en/humble/Installation.html) 安装
   - 确保安装完整桌面版本：`ros-humble-desktop`

6. **安装 ZED SDK 和 Python API**：
   - 按照 [ZED SDK 官方指南](https://www.stereolabs.com/docs/installation/linux) 安装
   - 在 conda 环境中验证：`python -c "import pyzed"`

7. **安装 Oculus Reader 系统依赖**：
```bash
# 安装 Android Debug Bridge（用于连接 Oculus Quest）
sudo apt install android-tools-adb
```

**注意**：Oculus Reader Python 包已在步骤 3 中安装。

## 安装

### 前置依赖

#### 方法 1: 使用 Git Submodules（推荐）

如果仓库已配置 Git Submodules，依赖包会自动管理：

```bash
# 克隆主仓库和所有 submodules
git clone --recursive <repository-url>
cd droid

# 或者如果已克隆主仓库，更新 submodules
git submodule update --init --recursive

# 确保 submodules 在正确的版本
cd ros2_ws/src/zed-ros2-wrapper && git checkout humble-v5.1.0 && cd ../../..
cd ros2_ws/src/franka_description && git checkout 0.5.1 && cd ../../..
cd ros2_ws/src/franka_ros2 && git checkout b79ce40 && cd ../../..
```

#### 方法 2: 手动安装

1. 安装 ROS2 Humble 依赖包：
```bash
sudo apt update
sudo apt install -y ros-humble-zed-msgs ros-humble-nmea-msgs
```

2. 安装依赖包（使用 Git Submodules，推荐）：
```bash
# 如果使用 Git Submodules，依赖包会自动克隆
cd ~/repos/droid
git submodule update --init --recursive
```

**或者手动安装依赖包**：

2a. 安装 zed_ros2_wrapper（使用指定版本）：
```bash
cd ~/repos/droid/ros2_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd zed-ros2-wrapper
git checkout humble-v5.1.0
cd ..
```

2b. 安装 franka_description（使用指定版本）：
```bash
cd ~/repos/droid/ros2_ws/src
git clone https://github.com/frankarobotics/franka_description.git
cd franka_description
git checkout 0.5.1
cd ..
```

2c. 安装 franka_ros2（使用指定提交）：
```bash
cd ~/repos/droid/ros2_ws/src
git clone https://github.com/souljaboy764/franka_ros2.git
cd franka_ros2
git checkout b79ce40
cd ..
```

3. 安装其他依赖（使用 rosdep）：
```bash
cd ~/repos/droid/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. 构建工作空间：
```bash
cd ~/repos/droid/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
```

**注意**：
- 构建产物（`build/`, `install/`, `log/`）已添加到 `.gitignore`，不会被提交
- 如果使用 Git Submodules，确保 submodules 在正确的版本/标签上

### 启动环境

**推荐方式：使用 `start_env.sh` 脚本**

这个脚本会自动激活 Conda 环境和 ROS2 工作空间：

```bash
# 从项目根目录
source ros2_ws/src/droid_ros2/start_env.sh

# 或者使用点命令
. ros2_ws/src/droid_ros2/start_env.sh
```

**自定义 Conda 环境名称**：
```bash
export CONDA_ENV_NAME=your_env_name
source ros2_ws/src/droid_ros2/start_env.sh
```

**手动方式**：
```bash
# 1. 激活 Conda 环境
source ~/miniconda3/etc/profile.d/conda.sh  # 或 ~/anaconda3/etc/profile.d/conda.sh
conda activate robot

# 2. Source ROS2 系统安装
source /opt/ros/humble/setup.bash

# 3. Source ROS2 工作空间
source ~/repos/droid/ros2_ws/install/setup.bash

# 4. 设置环境变量（可选）
export ROS_DOMAIN_ID=0
export RCUTILS_COLORIZED_OUTPUT=1
```

## 使用方法

**重要**: 在使用 ROS2 命令前，请确保已激活环境（见上面的"启动环境"部分）。

### 启动相机

启动两个相机：
```bash
ros2 launch droid_ros2 droid_cameras.launch.py
```

指定相机型号（如果需要）：
```bash
ros2 launch droid_ros2 droid_cameras.launch.py camera_model:=zed2
```

### 启动 Oculus Quest 控制器节点

#### USB 连接（默认）
```bash
ros2 launch droid_ros2 oculus_reader.launch.py
```

#### 网络连接
```bash
ros2 launch droid_ros2 oculus_reader.launch.py oculus_ip_address:=192.168.1.100
```

#### 自定义参数
```bash
ros2 launch droid_ros2 oculus_reader.launch.py \
    publish_rate:=100.0 \
    publish_tf:=true \
    frame_id:=oculus_base
```

**注意**: 使用 Oculus Reader 节点前，需要安装 `oculus_reader` Python 包：
```bash
pip install -e /path/to/droid/droid/oculus_reader
```

**启动参数**：
- `publish_rate` (默认: 50.0) - 发布频率（Hz）
- `publish_tf` (默认: true) - 是否发布 TF 变换
- `publish_markers` (默认: false) - 是否发布 RViz 可视化标记
- `oculus_ip_address` (默认: '') - Oculus Quest IP 地址（空字符串表示 USB 连接）
- `oculus_port` (默认: 5555) - ADB 网络连接端口
- `frame_id` (默认: 'oculus_base') - 发布的姿态和 TF 的坐标系 ID

**启用 RViz 可视化**：
```bash
# 启动节点并启用标记可视化
ros2 launch droid_ros2 oculus_reader.launch.py publish_markers:=true

# 在 RViz 中添加 MarkerArray 显示
# Add -> MarkerArray -> Topic: /oculus/controllers/markers
# Fixed Frame: oculus_base (或你设置的 frame_id)
```

**注意**：
- Marker 话题使用 `RELIABLE` QoS（RViz 要求），其他话题使用 `BEST_EFFORT`（更低延迟）
- 如果 RViz 中看不到标记，请检查：
  1. Fixed Frame 是否设置为正确的坐标系（默认：`oculus_base`）
  2. MarkerArray 显示是否已添加到 RViz
  3. 话题名称是否正确：`/oculus/controllers/markers`

### 测试 Oculus Reader 节点

**重要**：测试前必须确保工作空间已正确 source！

**快速测试（一行命令）**：
```bash
# 先 source 工作空间
source ros2_ws/src/droid_ros2/start_env.sh
# 然后测试
ros2 topic echo /oculus/buttons --once && echo "✓ 节点运行正常！"
```

**完整测试步骤**：
```bash
# 终端 1：启动节点（会自动 source 工作空间）
source ros2_ws/src/droid_ros2/start_env.sh
ros2 launch droid_ros2 oculus_reader.launch.py

# 终端 2：测试（也需要 source 工作空间）
source ros2_ws/src/droid_ros2/start_env.sh

# 检查节点
ros2 node list | grep oculus_reader_node

# 检查话题
ros2 topic list | grep oculus

# 验证消息类型是否正确注册
ros2 interface list | grep OculusButtons

# 查看按钮状态
ros2 topic echo /oculus/buttons

# 检查发布频率
ros2 topic hz /oculus/buttons
```

**常见错误修复**：

**错误：`The message type 'droid_ros2/msg/OculusButtons' is invalid`**
- **原因**：工作空间未正确 source
- **解决方法**：
  ```bash
  # 方法 1：使用 start_env.sh（推荐）
  source ros2_ws/src/droid_ros2/start_env.sh
  
  # 方法 2：手动 source
  source /opt/ros/humble/setup.bash
  source ros2_ws/install/setup.bash
  
  # 验证消息类型已注册
  ros2 interface list | grep OculusButtons
  ```
- **验证**：运行 `ros2 interface show droid_ros2/msg/OculusButtons` 应该能显示消息定义

## 话题

### ZED 相机话题

启动后，两个相机将发布以下话题：

#### Hand Camera
- `/hand_camera/zed_node/rgb/image_rect_color`
- `/hand_camera/zed_node/depth/depth_registered`
- `/hand_camera/zed_node/point_cloud/cloud`

#### Static Camera
- `/static_camera/zed_node/rgb/image_rect_color`
- `/static_camera/zed_node/depth/depth_registered`
- `/static_camera/zed_node/point_cloud/cloud`

### Oculus Quest 控制器话题

Oculus Reader 节点发布以下话题：

- `/oculus/right_controller/pose` (geometry_msgs/PoseStamped) - 右手控制器姿态
- `/oculus/left_controller/pose` (geometry_msgs/PoseStamped) - 左手控制器姿态
- `/oculus/buttons` (droid_ros2/OculusButtons) - 所有按钮和摇杆状态
- `/oculus/controllers/markers` (visualization_msgs/MarkerArray) - RViz 可视化标记（可选，通过 `publish_markers` 参数启用）

#### TF 变换（如果启用 `publish_tf:=true`）

- `oculus_base` → `oculus_right_controller` - 右手控制器变换
- `oculus_base` → `oculus_left_controller` - 左手控制器变换

#### OculusButtons 消息内容

- **布尔按钮**: `a`, `b`, `x`, `y`, `right_thumb_up`, `left_thumb_up`, `right_joystick_pressed`, `left_joystick_pressed`, `right_grip_pressed`, `left_grip_pressed`, `right_trigger_pressed`, `left_trigger_pressed`
- **模拟值** (0.0-1.0): `right_grip_value`, `left_grip_value`, `right_trigger_value`, `left_trigger_value`
- **摇杆位置** (-1.0 到 1.0): `right_joystick_x`, `right_joystick_y`, `left_joystick_x`, `left_joystick_y`

## 配置

### ZED 相机配置

相机参数可以通过以下配置文件进行覆盖：
- `config/hand_camera_params.yaml` - Hand camera 参数
- `config/static_camera_params.yaml` - Static camera 参数

默认配置已禁用 IMU 数据发布。

### Oculus Reader 节点参数

- `publish_rate` (float, 默认: 50.0) - 发布频率（Hz）
- `publish_tf` (bool, 默认: true) - 是否发布 TF 变换
- `oculus_ip_address` (string, 默认: '') - Oculus Quest IP 地址（空字符串表示 USB 连接）
- `oculus_port` (int, 默认: 5555) - ADB 网络连接端口
- `frame_id` (string, 默认: 'oculus_base') - 发布的姿态和 TF 的坐标系名称

## 环境管理最佳实践

### Conda 环境隔离

为了保持环境隔离，建议：

1. **Python 包通过 Conda/Pip 管理**：
   - 所有 Python 依赖安装在 conda 环境中
   - 避免使用系统级 `pip install --user` 或 `sudo pip install`

2. **ROS2 系统包**：
   - ROS2 核心包（`/opt/ros/humble`）需要系统级安装（通过 apt）
   - 这是 ROS2 的标准安装方式，无法避免

3. **ROS2 工作空间**：
   - 工作空间可以安装在任意位置（推荐在项目目录下）
   - 使用 `colcon build` 构建，不污染系统

4. **环境变量管理**：
   - `start_env.sh` 脚本自动设置必要的环境变量
   - 退出 conda 环境时，ROS2 环境变量会自动清理

### 验证环境设置

运行以下命令验证环境是否正确配置：

```bash
# 激活环境
source ros2_ws/src/droid_ros2/start_env.sh

# 验证 ROS2
ros2 --help

# 验证 Python 包
python -c "import rclpy; print('rclpy:', rclpy.__version__)"
python -c "import pyzed; print('pyzed available')"  # 如果安装了 ZED SDK

# 验证工作空间
ros2 pkg list | grep droid_ros2
```

### 故障排除

**问题：`ros2` 命令未找到**
- 确保已 source ROS2 系统安装：`source /opt/ros/humble/setup.bash`
- 检查 ROS2 是否正确安装：`ls /opt/ros/humble/`

**问题：`rclpy` 模块未找到**
- **重要**：ROS2 Humble 需要 Python 3.10。如果使用 Python 3.8 或 3.9，会出现 C 扩展模块不兼容的错误（`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`）
- 解决方案：创建新的 conda 环境使用 Python 3.10：
  ```bash
  conda create -n robot python=3.10
  conda activate robot
  # 重新安装所有依赖
  pip install -e ./droid/oculus_reader
  pip install -e .
  ```
- 如果必须使用其他 Python 版本，尝试：`conda install -c conda-forge ros-humble-rclpy`（可能不兼容）

**问题：工作空间包未找到**
- 确保已构建工作空间：`cd ros2_ws && colcon build`
- 确保已 source 工作空间：`source ros2_ws/install/setup.bash`

**问题：Conda 环境未激活**
- 检查 conda 是否正确安装：`which conda`
- 手动 source conda：`source ~/miniconda3/etc/profile.d/conda.sh`

**问题：`colcon build` 构建错误**

1. **错误：`std_msgs` has not been found before using find_package()**
   - 原因：CMakeLists.txt 中缺少 `find_package(std_msgs REQUIRED)`
   - 解决：已修复，确保 CMakeLists.txt 中包含：
     ```cmake
     find_package(std_msgs REQUIRED)
     ```

2. **错误：`rosidl_interface_packages` member_of_group missing**
   - 原因：package.xml 中缺少接口包声明
   - 解决：确保 package.xml 中包含：
     ```xml
     <member_of_group>rosidl_interface_packages</member_of_group>
     ```

3. **错误：`ModuleNotFoundError: No module named 'ament_package'`**
   - 原因：构建时使用了错误的 Python 环境
   - 解决：使用系统 Python 进行构建，而不是 conda 环境：
     ```bash
     # 构建时使用系统 Python（不要激活 conda 环境）
     source /opt/ros/humble/setup.bash
     cd ~/repos/droid/ros2_ws
     colcon build --symlink-install
     ```

4. **错误：`ImportError: cannot import name 'generate_py' from 'rosidl_generator_py'`**
   - 原因：Python 环境冲突（conda 和系统 Python 混用）
   - 解决：
     ```bash
     # 方法 1：构建时停用 conda 环境
     conda deactivate
     source /opt/ros/humble/setup.bash
     colcon build --symlink-install
     
     # 方法 2：清理构建缓存后重新构建
     cd ~/repos/droid/ros2_ws
     rm -rf build/ install/ log/
     source /opt/ros/humble/setup.bash
     colcon build --symlink-install
     ```

5. **错误：依赖包未找到（如 `zed_wrapper`, `zed_components`）**
   - 原因：需要先构建依赖包
   - 解决：按顺序构建依赖包：
     ```bash
     source /opt/ros/humble/setup.bash
     cd ~/repos/droid/ros2_ws
     # 先构建依赖包
     colcon build --packages-select zed_components zed_wrapper --symlink-install
     # 然后构建 droid_ros2
     colcon build --packages-select droid_ros2 --symlink-install
     # 或一次性构建所有包
     colcon build --symlink-install
     ```

6. **错误：Python 库路径冲突警告**
   - 警告信息：`runtime library [libpython3.10.so.1.0] may be hidden by files in: /home/user/miniconda3/envs/robot/lib`
   - 原因：conda 环境和系统 Python 路径冲突
   - 解决：构建时停用 conda 环境（见问题 3 和 4 的解决方案）

**构建最佳实践**：
- 构建 ROS2 包时，使用系统 Python（通过 `source /opt/ros/humble/setup.bash`）
- 运行时可以使用 conda 环境中的 Python
- 如果遇到构建问题，先清理构建缓存：`rm -rf build/ install/ log/`

## Git Submodules 管理

本项目使用 Git Submodules 管理依赖包。所有依赖包位于 `ros2_ws/src/` 目录下：

### Submodules 列表

- `ros2_ws/src/zed-ros2-wrapper` - ZED 相机 ROS2 包装器
  - 版本: `humble-v5.1.0` (tag)
  - 仓库: https://github.com/stereolabs/zed-ros2-wrapper.git
- `ros2_ws/src/franka_description` - Franka 机器人描述文件
  - 版本: `0.5.1` (tag)
  - 仓库: https://github.com/frankarobotics/franka_description.git
- `ros2_ws/src/franka_ros2` - Franka 机器人 ROS2 驱动
  - 版本: `b79ce40` (commit)
  - 仓库: https://github.com/souljaboy764/franka_ros2.git

### 常用 Submodule 命令

```bash
# 初始化并克隆所有 submodules
git submodule update --init --recursive

# 更新所有 submodules 到最新提交（在各自的分支上）
git submodule update --remote

# 更新特定 submodule
git submodule update --remote ros2_ws/src/zed-ros2-wrapper

# 检查 submodule 状态
git submodule status

# 进入 submodule 目录并切换版本
cd ros2_ws/src/zed-ros2-wrapper
git checkout humble-v5.1.0
cd ../../..
git add ros2_ws/src/zed-ros2-wrapper  # 提交版本变更
```

### 构建产物管理

构建产物（`ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`）已添加到 `.gitignore`，不会被 Git 跟踪。这些目录会在每次构建时重新生成。

