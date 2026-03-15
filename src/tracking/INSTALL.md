# ROS2 Tracking Package - Installation and Build Guide

## 系统要求

- ROS2 Humble Hawksbill 或更高版本
- Python 3.10+
- OpenCV (cv_bridge)
- NumPy

## 安装步骤

### 1. 克隆或复制包到工作区

将 `tracking` 文件夹复制到您的 ROS2 工作区的 `src` 目录：

```bash
# 假设您的 ROS2 工作区在 ~/ros2_ws
cp -r tracking ~/ros2_ws/src/
```

或者，如果您使用 git：

```bash
cd ~/ros2_ws/src
git clone <repository-url> tracking
```

### 2. 安装依赖

确保已安装 ROS2 和相关依赖：

```bash
# 更新包列表
sudo apt update

# 安装 ROS2 核心包（如果尚未安装）
sudo apt install ros-humble-desktop

# 安装视觉相关依赖
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# 安装 Python 依赖
pip3 install opencv-python numpy
```

### 3. 构建工作区

```bash
cd ~/ros2_ws

# 清理之前的构建（可选，但推荐）
colcon clean

# 构建 tracking 包
colcon build --packages-select tracking

# 或者构建整个工作区
colcon build
```

### 4. 设置环境变量

```bash
# 设置 ROS2 环境
source /opt/ros/humble/setup.bash

# 设置工作区环境
source ~/ros2_ws/install/setup.bash
```

建议将这些命令添加到您的 `~/.bashrc` 文件中：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. 验证安装

```bash
# 列出 tracking 包中的可执行文件
ros2 pkg executables tracking

# 应该看到：
# tracking object_tracker
# tracking pid_controller
```

## 快速启动指南

### 完整系统启动

启动完整的小车跟踪系统：

```bash
# 终端 1: 启动小车基础节点和相机
ros2 launch freenove_4wd_nodes bringup.launch.py

# 终端 2: 启动物体检测节点
ros2 launch vision_nodes ball_detection.launch.py

# 终端 3: 启动跟踪节点
ros2 launch tracking object_tracking.launch.py
```

### 仅测试跟踪节点（无需硬件）

```bash
# 终端 1: 启动跟踪节点
ros2 launch tracking object_tracking.launch.py

# 终端 2: 运行测试脚本模拟检测
python3 src/tracking/test_local/test_tracker.py
```

## 参数调整

### 实时调整参数

在节点运行时，可以使用以下命令调整参数：

```bash
# 调整角速度比例增益
ros2 param set /object_tracker angular_kp 0.02

# 调整目标物体大小
ros2 param set /object_tracker target_object_size 70.0

# 查看所有参数
ros2 param list
```

### 使用配置文件

创建自定义参数文件 `my_params.yaml`：

```yaml
/object_tracker:
  ros__parameters:
    target_object_size: 70.0
    angular_kp: 0.02
    linear_kp: 0.01
    max_linear_speed: 0.2
```

使用配置文件启动：

```bash
ros2 launch tracking object_tracking.launch.py \
  --ros-args --params-file my_params.yaml
```

## 故障排除

### 问题：找不到包

```bash
# 确保已正确设置环境
source ~/ros2_ws/install/setup.bash

# 检查包是否已构建
ros2 pkg list | grep tracking
```

### 问题：导入错误

```bash
# 检查 Python 依赖
pip3 list | grep opencv
pip3 list | grep numpy

# 重新安装依赖
pip3 install --upgrade opencv-python numpy
```

### 问题：构建失败

```bash
# 清理并重新构建
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select tracking
```

### 问题：节点无法启动

检查 ROS2 域 ID：

```bash
# 确保所有终端使用相同的 ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# 如果需要，设置相同的域 ID
export ROS_DOMAIN_ID=0
```

## 性能优化

### 1. 调整控制频率

编辑 `object_tracker_node.py`，修改定时器频率：

```python
# 默认 20 Hz
self.create_timer(0.05, self._control_loop)

# 提高到 50 Hz（更快的响应）
self.create_timer(0.02, self._control_loop)
```

### 2. 优化 PID 参数

参考 `params.yaml` 中的推荐配置，根据您的应用场景选择合适的参数。

### 3. 减少计算延迟

- 降低相机分辨率
- 使用压缩图像话题
- 减少调试日志输出

## 测试和验证

### 单元测试

```bash
cd ~/ros2_ws
pytest src/tracking/test/
```

### 集成测试

```bash
# 运行测试脚本
python3 src/tracking/test_local/test_tracker.py
```

### 性能监控

```bash
# 监控节点 CPU 和内存使用
top -d 1

# 监控话题频率
ros2 topic hz /cmd_vel
ros2 topic hz /ball_detection/status
```

## 卸载

```bash
# 从工作区删除包
cd ~/ros2_ws/src
rm -rf tracking/

# 重新构建工作区
cd ~/ros2_ws
colcon build
```

## 获取帮助

```bash
# 查看节点帮助
ros2 run tracking object_tracker --help

# 查看节点信息
ros2 node info /object_tracker

# 查看参数
ros2 param dump /object_tracker
```

## 参考资源

- [ROS2 官方文档](https://docs.ros.org/)
- [colcon 文档](https://colcon.readthedocs.io/)
- [PID 控制理论](https://en.wikipedia.org/wiki/PID_controller)
