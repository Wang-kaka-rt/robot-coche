# ROS2 Object Tracking - 快速参考卡片

## 🚀 快速启动

### 最简单启动
```bash
ros2 launch tracking object_tracking.launch.py
```

### 完整系统启动
```bash
# 终端 1
ros2 launch freenove_4wd_nodes bringup.launch.py

# 终端 2
ros2 launch vision_nodes ball_detection.launch.py

# 终端 3
ros2 launch tracking object_tracking.launch.py
```

### 测试模式（无需硬件）
```bash
# 终端 1
ros2 launch tracking object_tracking.launch.py

# 终端 2
python3 src/tracking/test_local/test_tracker.py
```

---

## 🎯 常用命令

### 监控和调试
```bash
# 查看节点
ros2 node list

# 查看节点信息
ros2 node info /object_tracker

# 查看话题
ros2 topic list

# 监控速度命令
ros2 topic echo /cmd_vel

# 监控话题频率
ros2 topic hz /cmd_vel
ros2 topic hz /ball_detection/status
```

### 参数调整
```bash
# 查看参数
ros2 param list

# 获取参数值
ros2 param get /object_tracker angular_kp

# 设置参数
ros2 param set /object_tracker angular_kp 0.02

# 保存参数
ros2 param dump /object_tracker > my_params.yaml
```

### 构建和安装
```bash
# 构建包
cd ~/ros2_ws
colcon build --packages-select tracking

# 设置环境
source install/setup.bash

# 清理重建
colcon clean
colcon build
```

---

## ⚙️ 关键参数

### 基本参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_object_size` | 60.0 | 目标物体大小（像素） |
| `center_tolerance` | 30.0 | 中心容忍度（像素） |
| `frame_width` | 400 | 图像宽度 |
| `frame_height` | 300 | 图像高度 |

### PID 参数 - 角速度
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `angular_kp` | 0.015 | 比例增益 |
| `angular_ki` | 0.0005 | 积分增益 |
| `angular_kd` | 0.0001 | 微分增益 |

### PID 参数 - 线速度
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `linear_kp` | 0.008 | 比例增益 |
| `linear_ki` | 0.0002 | 积分增益 |
| `linear_kd` | 0.00005 | 微分增益 |

### 速度限制
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_linear_speed` | 0.25 | 最大线速度 (m/s) |
| `max_angular_speed` | 0.8 | 最大角速度 (rad/s) |
| `min_linear_speed` | 0.08 | 最小线速度 (m/s) |
| `min_angular_speed` | 0.15 | 最小角速度 (rad/s) |

### 安全参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `obstacle_distance_m` | 0.3 | 障碍物警告距离 |
| `obstacle_critical_m` | 0.15 | 障碍物危险距离 |
| `low_battery_v` | 6.5 | 低电量阈值 (V) |

---

## 🔧 场景配置

### 室内良好光线
```bash
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=60.0 \
    angular_kp:=0.015 \
    linear_kp:=0.008
```

### 户外快速跟随
```bash
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=65.0 \
    angular_kp:=0.018 \
    linear_kp:=0.010 \
    max_linear_speed:=0.30
```

### 高精度模式
```bash
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=60.0 \
    center_tolerance:=20.0 \
    angular_kp:=0.020 \
    linear_kp:=0.012
```

### 安全保守模式
```bash
ros2 launch tracking object_tracking.launch.py \
    target_object_size:=65.0 \
    center_tolerance:=40.0 \
    angular_kp:=0.010 \
    linear_kp:=0.005 \
    obstacle_distance_m:=0.5
```

---

## 🐛 故障排除

### 机器人不移动
```bash
# 检查节点
ros2 node list

# 检查速度命令
ros2 topic echo /cmd_vel

# 检查检测状态
ros2 topic echo /ball_detection/status
```

### 跟踪不稳定
```bash
# 降低 Kp
ros2 param set /object_tracker angular_kp 0.010

# 增加 Kd
ros2 param set /object_tracker angular_kd 0.0003

# 增加容忍度
ros2 param set /object_tracker center_tolerance 40.0
```

### 响应太慢
```bash
# 增加 Kp
ros2 param set /object_tracker angular_kp 0.025

# 提高最大速度
ros2 param set /object_tracker max_linear_speed 0.30
```

### 障碍物规避不工作
```bash
# 检查超声波
ros2 topic echo /ultrasonic/range

# 增加检测距离
ros2 param set /object_tracker obstacle_distance_m 0.5
```

---

## 📊 状态机

```
LOST (0) ──检测到物体──> SEARCHING (1)
                            │
                      重新检测到物体
                            ▼
                      TRACKING (2)
                            │
                    物体居中且大小合适
                            ▼
                      FOLLOWING (3)
```

---

## 📝 代码结构

```
tracking/
├── tracking/
│   ├── object_tracker_node.py    # 主跟踪节点
│   ├── pid_controller_node.py    # PID 控制器
│   └── tracking_utils.py         # 工具函数
├── launch/
│   └── object_tracking.launch.py # 启动文件
├── test_local/
│   └── test_tracker.py           # 测试脚本
└── 文档...
```

---

## 📚 文档导航

- **README.md**: 功能说明和使用指南
- **INSTALL.md**: 详细安装步骤
- **EXAMPLES.md**: 丰富的使用示例
- **PROJECT_STRUCTURE.md**: 项目结构详解
- **params.yaml**: 推荐参数配置

---

## 💡 提示和技巧

### 实时调参
```bash
# 启动 rqt 重新配置
rqt_reconfigure
```

### 数据记录
```bash
# 记录数据
ros2 bag record /cmd_vel /ball_detection/status /tracking/state

# 回放数据
ros2 bag play <bag_file>
```

### 可视化
```bash
# 绘图
rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z

# 话题监控
rqt_topic
```

---

## 🎓 学习路径

1. **入门**: 阅读 README.md，运行最简单启动
2. **理解**: 查看 EXAMPLES.md，尝试不同场景配置
3. **调试**: 使用测试脚本，无需硬件测试
4. **调优**: 参考 params.yaml，调整 PID 参数
5. **深入**: 阅读 PROJECT_STRUCTURE.md，理解内部原理

---

## 📞 获取帮助

```bash
# 查看节点帮助
ros2 run tracking object_tracker --help

# 查看包信息
ros2 pkg info tracking

# 查看可执行文件
ros2 pkg executables tracking
```

---

**版本**: 0.0.1  
**许可证**: MIT  
**维护者**: robot-coche
