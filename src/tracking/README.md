# ROS2 Object Tracking Package

基于 ROS2 的物体跟踪包，用于 Freenove 4WD 智能小车的自主跟随功能。

## 功能特性

- **PID 控制**: 使用比例 - 积分 - 微分控制器实现平滑的运动控制
- **状态机**: 多状态跟踪（LOST、SEARCHING、TRACKING、FOLLOWING）
- **障碍物避免**: 集成超声波传感器进行障碍物检测和规避
- **电池监控**: 低电量检测和警告
- **自适应速度**: 根据物体距离自动调整跟随速度

## 包结构

```
tracking/
├── package.xml              # ROS2 包配置
├── setup.py                 # Python 包安装配置
├── launch/
│   └── object_tracking.launch.py  # 启动文件
├── resource/
└── tracking/
    ├── __init__.py
    ├── object_tracker_node.py     # 主跟踪节点
    ├── pid_controller_node.py     # PID 控制器节点
    └── tracking_utils.py          # 工具函数
```

## 安装

1. 将包复制到 ROS2 工作区的 src 目录：
```bash
cp -r tracking ~/ros2_ws/src/
```

2. 构建工作区：
```bash
cd ~/ros2_ws
colcon build --packages-select tracking
source install/setup.bash
```

## 使用方法

### 单独启动跟踪节点

```bash
ros2 launch tracking object_tracking.launch.py
```

### 自定义参数启动

```bash
# 调整目标物体大小和 PID 参数
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=80.0 \
    angular_kp:=0.02 \
    linear_kp:=0.01
```

### 与其他节点一起启动

```bash
# 启动小车基础节点、相机节点和跟踪节点
ros2 launch freenove_4wd_nodes bringup.launch.py
ros2 launch vision_nodes ball_detection.launch.py
ros2 launch tracking object_tracking.launch.py
```

## 订阅的话题

- `/ball_detection/status` (std_msgs/Bool): 物体检测状态
- `/ball_detection/position` (geometry_msgs/Point): 物体在图像中的位置
- `/ball_detection/radius` (std_msgs/Float32MultiArray): 物体大小/距离估计
- `/ultrasonic/range` (sensor_msgs/Range): 超声波传感器距离
- `/battery` (sensor_msgs/BatteryState): 电池状态

## 发布的话题

- `/cmd_vel` (geometry_msgs/Twist): 机器人速度命令
- `/tracking/state` (std_msgs/Int32): 当前跟踪状态

## 参数说明

### 基本参数

- `frame_width`: 相机图像宽度（像素），默认 400
- `frame_height`: 相机图像高度（像素），默认 300
- `target_object_size`: 目标物体大小（像素），默认 60.0
- `min_object_size`: 最小可检测物体大小，默认 10.0
- `max_object_size`: 最大可检测物体大小，默认 150.0
- `center_tolerance`: 中心容忍度（像素），默认 30.0

### PID 参数

#### 角速度控制（左右转向）
- `angular_kp`: 比例增益，默认 0.015
- `angular_ki`: 积分增益，默认 0.0005
- `angular_kd`: 微分增益，默认 0.0001

#### 线速度控制（前后移动）
- `linear_kp`: 比例增益，默认 0.008
- `linear_ki`: 积分增益，默认 0.0002
- `linear_kd`: 微分增益，默认 0.00005

### 速度限制

- `max_linear_speed`: 最大线速度 (m/s)，默认 0.25
- `max_angular_speed`: 最大角速度 (rad/s)，默认 0.8
- `min_linear_speed`: 最小线速度 (m/s)，默认 0.08
- `min_angular_speed`: 最小角速度 (rad/s)，默认 0.15

### 安全参数

- `obstacle_distance_m`: 障碍物警告距离，默认 0.3
- `obstacle_critical_m`: 障碍物危险距离，默认 0.15
- `low_battery_v`: 低电量阈值 (V)，默认 6.5

## 状态机说明

```
LOST (0) - 未检测到物体，停止运动
    ↓ 检测到物体
SEARCHING (1) - 最近丢失物体，主动搜索
    ↓ 重新检测到物体
TRACKING (2) - 检测到物体，调整位置
    ↓ 物体居中且大小合适
FOLLOWING (3) - 跟踪物体，保持距离跟随
```

## PID 控制器调优指南

### 调整步骤

1. **从默认参数开始**
   - 使用包中提供的默认 PID 参数

2. **调整角速度 PID（转向控制）**
   - 增加 `angular_kp` 直到响应快速但不过冲
   - 如果有稳态误差，小幅增加 `angular_ki`
   - 如果振荡，增加 `angular_kd`

3. **调整线速度 PID（前后控制）**
   - 增加 `linear_kp` 直到距离保持稳定
   - 如果有稳态误差，小幅增加 `linear_ki`
   - 如果振荡，增加 `linear_kd`

4. **测试和验证**
   - 在不同光照条件下测试
   - 测试不同距离的跟随性能
   - 测试障碍物规避功能

### 常见问题

- **振荡**: 降低 Kp 或增加 Kd
- **响应慢**: 增加 Kp
- **稳态误差**: 小幅增加 Ki
- **过冲**: 降低 Ki 或增加 Kd

## 工具模块功能

### tracking_utils.py

提供以下实用工具：

- `estimate_distance()`: 基于物体大小估算距离
- `calculate_focal_length()`: 相机焦距计算
- `ExponentialMovingAverage`: 指数移动平均滤波器
- `KalmanFilter1D`: 一维卡尔曼滤波器
- `MotionProfiler`: 运动剖面生成器
- `normalize_angle()`: 角度归一化
- `calculate_steering_angle()`: 转向角计算

## 故障排除

### 问题：机器人不移动

1. 检查 `/cmd_vel` 话题是否有发布
2. 确认 `car_base_node` 正在运行
3. 检查电池电量是否充足

### 问题：跟踪不稳定

1. 调整 PID 参数
2. 检查相机帧率是否稳定
3. 确认检测算法参数合适

### 问题：障碍物规避不工作

1. 检查超声波传感器是否正常工作
2. 验证 `/ultrasonic/range` 话题有数据发布
3. 调整 `obstacle_distance_m` 参数

## 许可证

MIT License

## 贡献

欢迎提交问题和拉取请求！
