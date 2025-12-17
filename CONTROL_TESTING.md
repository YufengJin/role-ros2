# Franka 机器人控制模式测试指南

本指南说明如何测试 Franka 机器人的4种控制模式，包括机械臂和夹爪控制。

## ⚠️ 安全警告

**在真机上测试前，请确保：**
1. 机器人周围有足够的空间
2. 紧急停止按钮随时可用
3. 机器人工作范围内没有人员或障碍物
4. 所有测试都只进行小幅度的安全移动

## 前置要求

1. **启动机器人系统**：
   ```bash
   ros2 launch role_ros2 franka_robot.launch.py
   ```

2. **确保控制器已定义**：
   这些测试需要以下控制器在 `franka_bringup/config/controllers.yaml` 中定义：
   - `cartesian_pose_example_controller`
   - `joint_position_example_controller`
   - `joint_velocity_example_controller`
   - `cartesian_velocity_example_controller`

   如果使用 `franka_fr3_moveit_config/config/fr3_ros_controllers.yaml`，需要添加这些控制器定义。

## 测试脚本

### 1. Cartesian Pose Control（笛卡尔位姿控制）

测试笛卡尔空间位姿控制模式。

```bash
ros2 run role_ros2 test_cartesian_pose_control
```

**测试内容**：
- 切换到 `cartesian_pose_example_controller`
- 运行5秒（机器人执行小幅度的圆形运动）
- 测试夹爪开合控制
- 返回安全状态

**预期行为**：
- 机器人末端执行器在小范围内做圆形运动
- 夹爪可以正常开合

### 2. Joint Position Control（关节位置控制）

测试关节空间位置控制模式。

```bash
ros2 run role_ros2 test_joint_position_control
```

**测试内容**：
- 切换到 `joint_position_example_controller`
- 运行5秒（机器人关节做周期性位置运动）
- 测试夹爪开合控制
- 返回安全状态

**预期行为**：
- 机器人关节在小范围内做周期性位置运动
- 夹爪可以正常开合

### 3. Joint Velocity Control（关节速度控制）

测试关节空间速度控制模式。

```bash
ros2 run role_ros2 test_joint_velocity_control
```

**测试内容**：
- 切换到 `joint_velocity_example_controller`
- 运行5秒（机器人关节做周期性速度运动）
- 测试夹爪开合控制
- 返回安全状态

**预期行为**：
- 机器人关节在小范围内做周期性速度运动
- 夹爪可以正常开合

### 4. Cartesian Velocity Control（笛卡尔速度控制）

测试笛卡尔空间速度控制模式。

```bash
ros2 run role_ros2 test_cartesian_velocity_control
```

**测试内容**：
- 切换到 `cartesian_velocity_example_controller`
- 运行5秒（机器人末端执行器做速度运动）
- 测试夹爪开合控制
- 返回安全状态

**预期行为**：
- 机器人末端执行器在小范围内做速度运动
- 夹爪可以正常开合

## 测试流程

每个测试脚本的执行流程：

1. **列出当前控制器**：检查哪些控制器当前处于活动状态
2. **停用当前控制器**：安全地停用当前活动的机械臂控制器
3. **激活测试控制器**：激活要测试的控制器
4. **运行测试**：让控制器运行5秒（小幅度安全移动）
5. **测试夹爪**：测试夹爪的开合控制
6. **返回安全状态**：停用测试控制器，返回安全状态

## 安全特性

所有测试脚本都包含以下安全特性：

1. **小幅度移动**：控制器只运行5秒，移动范围很小
2. **紧急停止**：支持 Ctrl+C 中断，会自动停用控制器
3. **错误处理**：任何错误都会自动停用控制器
4. **状态检查**：在切换控制器前检查当前状态

## 故障排除

### 问题：控制器激活失败

**错误信息**：
```
Failed to activate <controller_name>
Note: This controller must be defined in controllers.yaml
```

**解决方案**：
1. 检查控制器是否在 `controllers.yaml` 中定义
2. 确保 `franka_bringup` 包的 `controllers.yaml` 包含所需的控制器
3. 如果使用 `fr3_ros_controllers.yaml`，需要添加这些控制器定义

### 问题：夹爪控制失败

**错误信息**：
```
Gripper action server not available
```

**解决方案**：
1. 确保夹爪已启动：`load_gripper:=true`
2. 检查夹爪节点是否运行：`ros2 node list | grep gripper`
3. 检查夹爪服务：`ros2 service list | grep gripper`

### 问题：控制器切换超时

**解决方案**：
1. 检查控制器管理器是否运行：`ros2 service list | grep controller_manager`
2. 增加超时时间（修改脚本中的 `timeout_sec` 参数）
3. 检查机器人连接状态

## 手动测试控制器

如果自动测试脚本不可用，可以手动测试：

### 1. 列出可用控制器
```bash
ros2 control list_controllers
```

### 2. 切换控制器
```bash
# 停用当前控制器
ros2 control set_controller_state <current_controller> inactive

# 激活新控制器
ros2 control set_controller_state <new_controller> active
```

### 3. 测试夹爪
```bash
ros2 action send_goal /fr3_gripper/move franka_msgs/action/Move "{width: 0.04, speed: 0.1}"
```

### 4. 返回安全状态
```bash
ros2 control set_controller_state <test_controller> inactive
```

## 注意事项

1. **测试顺序**：建议按顺序测试，每次测试后等待机器人完全停止
2. **工作空间**：确保机器人有足够的空间进行小幅度移动
3. **监控**：测试时密切观察机器人运动，随时准备按紧急停止
4. **日志**：查看终端输出了解测试进度和状态

## 控制器说明

### Example Controllers

这些控制器是 `franka_example_controllers` 包提供的示例控制器：

- **cartesian_pose_example_controller**：在笛卡尔空间执行小幅度的圆形运动
- **joint_position_example_controller**：在关节空间执行周期性位置运动
- **joint_velocity_example_controller**：在关节空间执行周期性速度运动
- **cartesian_velocity_example_controller**：在笛卡尔空间执行速度运动

这些控制器主要用于演示和测试，实际应用应该使用 `fr3_arm_controller`（joint trajectory controller）进行轨迹控制。

## 相关文档

- [Franka ROS2 文档](https://github.com/frankarobotics/franka_ros2)
- [Controller Manager 文档](https://control.ros.org/)
- [Franka Example Controllers](https://github.com/frankarobotics/franka_ros2/tree/main/franka_example_controllers)

