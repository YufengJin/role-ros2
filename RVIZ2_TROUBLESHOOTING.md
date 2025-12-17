# RViz2 机器人模型显示问题排查指南

## 问题：RViz2 中显示的机器人模型不是当前状态

### 问题原因

RViz2 中的 `RobotModel` 显示依赖于：
1. **TF 变换**：`robot_state_publisher` 根据 `/joint_states` 发布 TF 变换
2. **Joint States**：`/joint_states` 话题必须包含所有关节的当前状态
3. **Fixed Frame**：RViz2 的 Fixed Frame 必须设置为正确的 base frame

### 数据流

```
joint_state_broadcaster → franka/joint_states
fr3_gripper → fr3_gripper/joint_states
         ↓
joint_state_publisher (合并) → /joint_states
         ↓
robot_state_publisher → /tf (TF 变换)
         ↓
RViz2 RobotModel (显示)
```

## 诊断步骤

### 1. 检查 `/joint_states` 话题是否有数据

```bash
# 检查话题信息
ros2 topic info /joint_states

# 查看数据流
ros2 topic echo /joint_states

# 检查发布频率
ros2 topic hz /joint_states
```

**预期结果**：
- 应该看到所有关节（7个 arm joints + 2个 gripper joints）的状态
- 发布频率应该 > 0 Hz

### 2. 检查 `robot_state_publisher` 节点

```bash
# 检查节点是否运行
ros2 node list | grep robot_state_publisher

# 检查节点信息
ros2 node info /robot_state_publisher

# 检查订阅的话题
ros2 node info /robot_state_publisher | grep -A 5 "Subscribers:"
```

**预期结果**：
- 节点应该存在
- 应该订阅 `/joint_states` 话题

### 3. 检查 TF 变换

```bash
# 检查 TF 树
ros2 run tf2_tools view_frames
# 这会生成 frames.pdf 文件，查看 TF 树结构

# 检查特定变换
ros2 run tf2_ros tf2_echo base fr3_link8

# 列出所有 frames
ros2 run tf2_ros tf2_monitor
```

**预期结果**：
- 应该看到从 `base` 到 `fr3_link8` 的完整 TF 链
- 所有关节的变换都应该存在

### 4. 检查 RViz2 设置

在 RViz2 中：
1. **Fixed Frame**：设置为 `base`（不是 `base_link`）
2. **RobotModel**：
   - Description Topic: `/robot_description`
   - TF Prefix: 留空（或设置为正确的 prefix）

### 5. 检查各个组件

```bash
# 检查 joint_state_broadcaster 是否激活
ros2 control list_controllers

# 检查 joint_state_publisher 节点
ros2 node list | grep joint_state_publisher

# 检查各个 joint_states 话题
ros2 topic list | grep joint_states
```

## 常见问题和解决方案

### 问题 1：`/joint_states` 没有数据

**可能原因**：
- `joint_state_broadcaster` 未激活
- `joint_state_publisher` 未运行
- 话题名称不匹配

**解决方案**：
```bash
# 激活 joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active

# 检查 joint_state_publisher 的 source_list 配置
ros2 param get /joint_state_publisher source_list
```

### 问题 2：TF 变换不存在

**可能原因**：
- `robot_state_publisher` 未运行
- `robot_state_publisher` 未订阅 `/joint_states`
- URDF 中的 frame 名称不匹配

**解决方案**：
- 确保 `robot_state_publisher` 节点正在运行
- 检查 launch 文件中的 remapping 配置
- 验证 URDF 中的 base frame 名称（应该是 `base`）

### 问题 3：RViz2 中 Fixed Frame 错误

**解决方案**：
- 在 RViz2 左上角，将 Fixed Frame 设置为 `base`
- 如果 `base` 不存在，尝试 `fr3_link0`

### 问题 4：机器人模型显示但位置不对

**可能原因**：
- TF 变换数据过时
- `/joint_states` 数据不准确

**解决方案**：
- 检查 `/joint_states` 的时间戳是否更新
- 验证关节状态数据是否来自真实硬件

## 验证修复

修复后，运行以下命令验证：

```bash
# 1. 检查所有必需的话题
ros2 topic list | grep -E "(joint_states|tf|robot_description)"

# 2. 检查 TF 变换
ros2 run tf2_ros tf2_echo base fr3_link8

# 3. 在 RViz2 中验证
# - Fixed Frame 设置为 "base"
# - RobotModel 应该显示并跟随真实机器人移动
```

## 相关文件

- Launch 文件：`ros2_ws/src/role-ros2/launch/franka_robot.launch.py`
- 配置文件：`ros2_ws/src/role-ros2/config/franka_robot_config.yaml`
- URDF 文件：`ros2_ws/src/franka_description/robots/fr3/fr3.urdf.xacro`

