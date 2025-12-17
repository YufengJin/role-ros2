# ZED ROS2 Wrapper 可覆盖参数说明

## 相机识别参数

### serial_number vs camera_id

- **serial_number**: 使用相机的实际序列号（字符串格式），例如 `"11022812"`
  - 这是唯一标识符，推荐在多相机系统中使用
  - 可以通过 `ZED_Explorer -a` 命令获取所有相机的序列号
  
- **camera_id**: 使用整数索引（0, 1, 2...）来标识相机
  - 默认值：`-1`（自动选择第一个可用相机）
  - 在多相机系统中，可以使用 `0`, `1` 等来区分相机
  - 注意：camera_id 可能在不同启动时变化，不如 serial_number 稳定

**建议**：在多相机系统中，优先使用 `serial_number` 来确保稳定识别。

## 可通过 Launch Arguments 覆盖的参数

根据 `zed_camera.launch.py`，以下参数可以通过 launch arguments 直接覆盖：

### 基本参数
- `camera_name`: 相机名称（字符串）
- `camera_model`: 相机型号（'zed', 'zedm', 'zed2', 'zed2i', 'zedx', 等）
- `serial_number`: 相机序列号（字符串，如 "11022812"）
- `camera_id`: 相机ID（整数，如 0, 1, -1 表示自动）
- `namespace`: 节点命名空间
- `node_name`: 节点名称

### TF 相关
- `publish_tf`: 发布 `odom -> camera_link` TF (true/false)
- `publish_map_tf`: 发布 `map -> odom` TF (true/false)
- `publish_imu_tf`: 发布 IMU TF (true/false)

### SVO 相关
- `svo_path`: SVO 文件路径（默认 'live'）
- `publish_svo_clock`: 发布 SVO 时钟 (true/false)

### 仿真相关
- `sim_mode`: 仿真模式 (true/false)
- `sim_address`: 仿真服务器地址
- `sim_port`: 仿真服务器端口

### 流媒体相关
- `stream_address`: 流媒体服务器地址
- `stream_port`: 流媒体服务器端口

### 其他
- `ros_params_override_path`: 参数覆盖文件路径（YAML格式）
- `enable_ipc`: 启用进程内通信 (true/false)
- `use_sim_time`: 使用仿真时间 (true/false)
- `enable_gnss`: 启用 GNSS 融合 (true/false)

## 可通过参数覆盖文件（YAML）覆盖的参数

所有在 `common_stereo.yaml` 或 `common_mono.yaml` 中定义的参数都可以通过 `ros_params_override_path` 指定的 YAML 文件覆盖。

### 深度相关参数
```yaml
depth:
    depth_mode: 'NEURAL'  # 可选值: 'NONE', 'PERFORMANCE', 'QUALITY', 'ULTRA', 'NEURAL', 'NEURAL_PLUS', 'NEURAL_LIGHT'
    depth_stabilization: 30
    min_depth: 0.01
    max_depth: 15.0
    depth_confidence: 95
```

### 传感器参数
```yaml
sensors:
    publish_imu: true/false
    publish_imu_raw: true/false
    publish_imu_tf: true/false
    sensors_pub_rate: 100.0
```

### 视频参数
```yaml
video:
    brightness: 4
    contrast: 4
    saturation: 4
    exposure: 80
    gain: 80
```

### 位置跟踪参数
```yaml
pos_tracking:
    pos_tracking_enabled: true/false
    pos_tracking_mode: 'GEN_2'  # 'GEN_1', 'GEN_2', 'GEN_3'
    publish_tf: true/false
    publish_map_tf: true/false
```

## 示例：使用 camera_id 替代 serial_number

如果 serial_number 无法正常工作，可以尝试使用 camera_id：

```python
launch_arguments={
    'camera_name': 'hand_camera',
    'camera_model': 'zedm',
    'camera_id': '0',  # 使用 camera_id 而不是 serial_number
    'serial_number': '0',  # 设置为默认值，让 camera_id 生效
    # ...
}
```

## 深度模式说明

支持的深度模式（`depth.depth_mode`）：
- `NONE`: 禁用深度计算
- `PERFORMANCE`: 性能模式（最快）
- `QUALITY`: 质量模式
- `ULTRA`: 超高质量模式
- `NEURAL`: 神经网络模式（推荐）
- `NEURAL_PLUS`: 增强神经网络模式
- `NEURAL_LIGHT`: 轻量神经网络模式（默认）

