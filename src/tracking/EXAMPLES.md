# ROS2 Object Tracking - 使用示例

本文档提供详细的使用示例，帮助您快速上手 ROS2 物体跟踪功能。

## 目录

1. [基础示例](#基础示例)
2. [高级配置](#高级配置)
3. [实际应用场景](#实际应用场景)
4. [故障排除](#故障排除)

---

## 基础示例

### 示例 1: 最简单的启动方式

```bash
# 单个命令启动跟踪系统
ros2 launch tracking object_tracking.launch.py
```

这将使用默认参数启动跟踪节点。

### 示例 2: 查看节点输出

```bash
# 在一个终端启动跟踪节点
ros2 launch tracking object_tracking.launch.py

# 在另一个终端查看节点日志
ros2 node info /object_tracker

# 查看发布的话题
ros2 topic list

# 实时监控速度命令
ros2 topic echo /cmd_vel
```

### 示例 3: 使用测试脚本（无需硬件）

```bash
# 终端 1: 启动跟踪节点
ros2 launch tracking object_tracking.launch.py

# 终端 2: 运行测试脚本
cd ~/ros2_ws
python3 src/tracking/test_local/test_tracker.py
```

测试脚本会模拟以下场景：
- 0-3 秒：无检测（机器人应停止）
- 3-6 秒：物体在右侧（机器人应左转）
- 6-9 秒：物体居中（机器人应前进）
- 9-12 秒：物体太近（机器人应后退）
- 12-15 秒：物体丢失（机器人应搜索）

---

## 高级配置

### 示例 4: 自定义 PID 参数

```bash
# 调整角速度 PID 参数以获得更快的响应
ros2 launch tracking object_tracking.launch.py \
    angular_kp:=0.025 \
    angular_ki:=0.001 \
    angular_kd:=0.0002
```

### 示例 5: 配置目标物体大小

```bash
# 如果跟踪的物体较大，增加目标大小
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=80.0 \
    min_object_size:=20.0 \
    max_object_size:=200.0
```

### 示例 6: 使用配置文件

创建 `custom_params.yaml`:

```yaml
/object_tracker:
  ros__parameters:
    # 基本参数
    target_object_size: 70.0
    center_tolerance: 35.0
    
    # PID 参数 - 角速度
    angular_kp: 0.018
    angular_ki: 0.0006
    angular_kd: 0.00015
    
    # PID 参数 - 线速度
    linear_kp: 0.010
    linear_ki: 0.0003
    linear_kd: 0.00008
    
    # 速度限制
    max_linear_speed: 0.30
    max_angular_speed: 1.0
    
    # 安全参数
    obstacle_distance_m: 0.4
    obstacle_critical_m: 0.2
```

使用配置文件启动：

```bash
ros2 launch tracking object_tracking.launch.py \
    --ros-args --params-file custom_params.yaml
```

### 示例 7: 实时调整参数

```bash
# 启动节点
ros2 launch tracking object_tracking.launch.py

# 在另一个终端调整参数
# 增加角速度比例增益
ros2 param set /object_tracker angular_kp 0.02

# 调整目标物体大小
ros2 param set /object_tracker target_object_size 75.0

# 查看当前所有参数
ros2 param dump /object_tracker

# 将当前参数保存到文件
ros2 param dump /object_tracker > current_params.yaml
```

---

## 实际应用场景

### 场景 1: 室内跟随（光线良好）

```bash
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=60.0 \
    center_tolerance:=30.0 \
    angular_kp:=0.015 \
    linear_kp:=0.008 \
    max_linear_speed:=0.25 \
    max_angular_speed:=0.8
```

**特点**: 标准参数设置，平衡性能和稳定性。

### 场景 2: 室内跟随（光线较差）

```bash
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=70.0 \
    center_tolerance:=40.0 \
    angular_kp:=0.012 \
    angular_kd:=0.0002 \
    linear_kp:=0.006 \
    max_linear_speed:=0.20 \
    max_angular_speed:=0.6
```

**特点**: 更保守的参数，减少振荡，提高稳定性。

### 场景 3: 户外快速跟随

```bash
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=65.0 \
    angular_kp:=0.018 \
    linear_kp:=0.010 \
    max_linear_speed:=0.30 \
    max_angular_speed:=1.0 \
    obstacle_distance_m:=0.4
```

**特点**: 更高的速度和响应性，更早的障碍物检测。

### 场景 4: 高精度跟踪

```bash
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=60.0 \
    center_tolerance:=20.0 \
    angular_kp:=0.020 \
    angular_ki:=0.0008 \
    linear_kp:=0.012 \
    max_linear_speed:=0.15 \
    max_angular_speed:=0.5
```

**特点**: 更严格的中心容忍度，更高的 PID 增益，适合精密操作。

### 场景 5: 安全保守模式

```bash
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=65.0 \
    center_tolerance:=40.0 \
    angular_kp:=0.010 \
    linear_kp:=0.005 \
    max_linear_speed:=0.15 \
    max_angular_speed:=0.4 \
    obstacle_distance_m:=0.5 \
    obstacle_critical_m:=0.25
```

**特点**: 低速运行，早期障碍物检测，适合人群密集环境。

---

## 完整系统启动示例

### 示例 8: 启动完整的小车系统

```bash
# 终端 1: 启动小车基础节点（电机、传感器、相机）
ros2 launch freenove_4wd_nodes bringup.launch.py

# 终端 2: 启动物体检测节点
ros2 launch vision_nodes ball_detection.launch.py \
    camera_topic:=/image_raw/compressed \
    publish_debug_images:=True

# 终端 3: 启动跟踪节点
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=60.0 \
    angular_kp:=0.015
```

### 示例 9: 监控系统状态

```bash
# 查看所有活跃节点
ros2 node list

# 查看跟踪节点详情
ros2 node info /object_tracker

# 查看话题和发布频率
ros2 topic list
ros2 topic hz /cmd_vel
ros2 topic hz /ball_detection/status
ros2 topic hz /ball_detection/position

# 查看传感器数据
ros2 topic echo /ultrasonic/range
ros2 topic echo /battery

# 查看跟踪状态
ros2 topic echo /tracking/state
```

### 示例 10: 使用 rqt 工具可视化

```bash
# 启动 rqt 图形界面
rqt

# 或者启动特定的 rqt 插件
# 话题监控
rqt_topic

# 绘图
rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z

# 参数配置
rqt_reconfigure
```

---

## 故障排除示例

### 问题 1: 机器人不移动

**检查步骤**:

```bash
# 1. 检查跟踪节点是否运行
ros2 node list

# 2. 检查是否有速度命令发布
ros2 topic echo /cmd_vel

# 3. 检查检测数据是否正常
ros2 topic echo /ball_detection/status
ros2 topic echo /ball_detection/position

# 4. 检查小车基础节点
ros2 node info /car_base
```

**解决方案**:

```bash
# 确保所有节点都正确启动
ros2 launch freenove_4wd_nodes bringup.launch.py &
ros2 launch vision_nodes ball_detection.launch.py &
ros2 launch tracking object_tracking.launch.py &
```

### 问题 2: 跟踪不稳定（振荡）

**调整参数**:

```bash
# 降低比例增益
ros2 param set /object_tracker angular_kp 0.010
ros2 param set /linear_kp 0.005

# 增加微分增益（阻尼）
ros2 param set /object_tracker angular_kd 0.0003
ros2 param set /object_tracker linear_kd 0.00015

# 增加中心容忍度
ros2 param set /object_tracker center_tolerance 40.0
```

### 问题 3: 响应太慢

**调整参数**:

```bash
# 增加比例增益
ros2 param set /object_tracker angular_kp 0.025
ros2 param set /object_tracker linear_kp 0.012

# 提高最大速度
ros2 param set /object_tracker max_linear_speed 0.30
ros2 param set /object_tracker max_angular_speed 1.0

# 减少中心容忍度
ros2 param set /object_tracker center_tolerance 20.0
```

### 问题 4: 障碍物规避不工作

**检查步骤**:

```bash
# 1. 检查超声波传感器数据
ros2 topic echo /ultrasonic/range

# 2. 检查障碍物参数
ros2 param get /object_tracker obstacle_distance_m
ros2 param get /object_tracker obstacle_critical_m
```

**调整参数**:

```bash
# 增加障碍物检测距离
ros2 param set /object_tracker obstacle_distance_m 0.5
ros2 param set /object_tracker obstacle_critical_m 0.25
```

---

## 性能调优示例

### 示例 11: 使用不同场景的预设参数

```bash
# 从 params.yaml 加载预设参数

# 室内良好光线
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=60.0 \
    angular_kp:=0.015 \
    linear_kp:=0.008

# 户外环境
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=65.0 \
    angular_kp:=0.018 \
    linear_kp:=0.010 \
    max_linear_speed:=0.30

# 高精度模式
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=60.0 \
    center_tolerance:=20.0 \
    angular_kp:=0.020 \
    linear_kp:=0.012
```

### 示例 12: 记录和回放数据

```bash
# 记录跟踪数据
ros2 bag record -a -o tracking_session_1

# 回放数据
ros2 bag play tracking_session_1

# 只记录特定话题
ros2 bag record /cmd_vel /ball_detection/status /tracking/state
```

---

## 高级技巧

### 技巧 1: 创建自定义启动文件

创建 `my_tracking.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tracking',
            executable='object_tracker',
            name='object_tracker',
            output='screen',
            parameters=[{
                'target_object_size': 70.0,
                'angular_kp': 0.02,
                'linear_kp': 0.01,
                'max_linear_speed': 0.25,
            }]
        )
    ])
```

使用：

```bash
ros2 launch my_tracking.launch.py
```

### 技巧 2: 编写参数调优脚本

创建 `tune_params.sh`:

```bash
#!/bin/bash

echo "Starting parameter tuning..."

# 启动节点
ros2 launch tracking object_tracking.launch.py &
NODE_PID=$!

# 等待节点启动
sleep 2

# 测试不同的 Kp 值
for kp in 0.010 0.015 0.020 0.025; do
    echo "Testing angular_kp=$kp"
    ros2 param set /object_tracker angular_kp $kp
    sleep 5
done

# 停止节点
kill $NODE_PID
```

### 技巧 3: 自动化测试

创建测试脚本自动验证跟踪性能：

```bash
#!/bin/bash

echo "=== Running automated tracking test ==="

# 启动节点
ros2 launch tracking object_tracking.launch.py &
sleep 2

# 运行测试脚本
python3 src/tracking/test_local/test_tracker.py &
sleep 15

# 检查日志
ros2 node info /object_tracker

# 停止所有
pkill -f object_tracker
pkill -f test_tracker

echo "=== Test complete ==="
```

---

## 总结

本文档提供了丰富的使用示例，涵盖了从基础到高级的各种场景。关键要点：

1. **从默认参数开始**: 使用默认参数启动，然后根据实际表现调整
2. **逐步调优**: 一次只调整一个参数，观察效果
3. **记录最佳参数**: 为不同场景保存最佳参数配置
4. **定期测试**: 使用测试脚本验证系统性能
5. **监控日志**: 密切关注节点输出和错误信息

祝您使用愉快！如有问题，请参考故障排除部分或查看 README.md 获取更多详细信息。
