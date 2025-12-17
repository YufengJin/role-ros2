# Cartesian Pose Control 测试说明

## 配置要求

`cartesian_pose_example_controller` 需要在控制器配置文件中定义才能使用。

### 配置文件位置

配置文件：`ros2_ws/src/franka_ros2/franka_fr3_moveit_config/config/fr3_ros_controllers.yaml`

### 已添加的配置

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    # ... 其他控制器 ...

    cartesian_pose_example_controller:
      type: franka_example_controllers/CartesianPoseExampleController
```

## 使用方法

### 1. 重新构建并重启系统

```bash
# 重新构建配置
cd ~/repos/droid/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select franka_fr3_moveit_config --symlink-install

# 重启机器人系统（重要！）
# 停止当前的 launch 文件，然后重新启动
ros2 launch role_ros2 franka_robot.launch.py
```

### 2. 运行测试

在另一个终端运行：

```bash
source ~/repos/droid/ros2_ws/install/setup.bash
ros2 run role_ros2 test_cartesian_pose_control
```

## 测试流程

测试脚本会执行以下步骤：

1. **列出当前控制器**：检查哪些控制器正在运行
2. **停用当前机械臂控制器**：停用 `fr3_arm_controller` 或其他激活的控制器
3. **加载并激活 cartesian_pose_example_controller**：
   - 如果控制器未加载，先加载它
   - 然后激活控制器
4. **运行控制器 5 秒**：控制器会执行小的圆形运动模式
5. **测试夹爪**：打开和关闭夹爪
6. **停用控制器并返回安全状态**

## 安全注意事项

⚠️ **重要**：
- 测试时机器人会移动（小的圆形运动模式）
- 确保紧急停止按钮可用
- 确保机器人周围没有障碍物
- 测试移动距离很小（半径约 0.1m），但仍需小心

## 故障排除

### 问题：控制器加载失败

**错误信息**：
```
Failed to load cartesian_pose_example_controller
Note: This controller must be defined in controllers.yaml
```

**解决方案**：
1. 确认 `fr3_ros_controllers.yaml` 中已添加控制器定义
2. **重新构建配置包**：
   ```bash
   colcon build --packages-select franka_fr3_moveit_config --symlink-install
   ```
3. **重启机器人 launch 文件**（必须！配置文件只在启动时加载）

### 问题：控制器激活失败

**可能原因**：
- 控制器类型不存在
- 硬件接口不匹配
- 其他控制器仍在使用相同的接口

**解决方案**：
- 检查 `ros2 control list_controllers` 查看当前状态
- 确保所有其他控制器已停用
- 检查控制器日志获取详细错误信息

## 控制器行为

`cartesian_pose_example_controller` 会执行以下运动：
- 在 x 和 z 方向上进行小的圆形运动
- 运动半径：约 0.1 米
- 运动模式：平滑的正弦/余弦轨迹

这是用于测试笛卡尔空间位置控制的示例控制器。

