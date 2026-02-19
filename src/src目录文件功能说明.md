# Robot_coche/src 目录文件功能说明

本文说明 `Robot_coche/src` 下每个文件/目录的用途。该目录是 ROS2 工作空间的源码区，包含两个 ROS2 Python 包：

- `freenove_4wd_hw`：硬件驱动/算法模块（从原 `Code/Server` 复制过来），作为库被节点调用
- `freenove_4wd_nodes`：ROS2（rclpy）节点包，发布/订阅 ROS2 Topic，启动后驱动小车与相机

---

## 1) freenove_4wd_hw（硬件模块包）

路径：`Robot_coche/src/freenove_4wd_hw/`

### 1.1 构建与包元数据文件

- `package.xml`
  - ROS2 包清单文件，声明包名、依赖、license 等；colcon/ament 会读取它来解析依赖关系。
- `setup.py`
  - Python 包安装入口（setuptools）。用于 `colcon build` 期间把 Python 模块安装到 `install/` 下。
- `setup.cfg`
  - ament_python 常用配置，指定脚本安装目录等。
- `resource/freenove_4wd_hw`
  - ament 索引资源文件（内容通常就是包名）。用于 ROS2 在环境中发现该包。

### 1.2 Python 模块目录：freenove_4wd_hw/

路径：`Robot_coche/src/freenove_4wd_hw/freenove_4wd_hw/`

- `__init__.py`
  - 包初始化文件。这里额外把本包目录加入 `sys.path`，目的是兼容原始代码中大量“同目录导入”的写法（例如 `from pca9685 import PCA9685`），避免逐个重写所有 import。
- `adc.py`
  - ADS7830 的 I2C ADC 读取封装：读取光敏电阻、电池电压等模拟量。
- `buzzer.py`
  - 蜂鸣器 GPIO 控制（开/关）。
- `car.py`
  - 小车整车封装：组合电机、舵机、超声波、红外、ADC；包含避障/循迹/跟光/旋转等模式逻辑。
- `command.py`
  - 原 TCP 协议时代的命令字符串常量集合（在 ROS2 节点中通常不再需要，但保留用于复用或扩展）。
- `infrared.py`
  - 三路红外循迹传感器读取（gpiozero LineSensor），提供单路/多路读取与合并值。
- `led.py`
  - LED 效果层（跟随/呼吸/彩虹等），根据硬件版本选择 WS281x 或 SPI 灯带驱动。
- `message.py`
  - 原 TCP 协议消息解析器（`CMD_xxx#...`）。当前 ROS2 版本不依赖它，但保留便于后续做 TCP→ROS2 bridge。
- `motor.py`
  - 4 轮电机控制封装：把四个占空比映射到 PCA9685 的 PWM 通道，支持正转/反转/刹停。
- `parameter.py`
  - 硬件版本参数管理（`params.json`）：连接板版本/PCB 版本/树莓派版本，用于决定 LED 驱动方式、ADC 系数等。
- `pca9685.py`
  - PCA9685 16 路 PWM 驱动（I2C），提供设置频率/通道 PWM/舵机脉宽等基础能力。
- `rpi_ledpixel.py`
  - 基于 `rpi_ws281x` 的灯带驱动封装（WS281x 类灯带）。
- `servo.py`
  - 舵机控制封装：把角度转换为 PWM 脉宽并输出到 PCA9685 的指定通道。
- `spi_ledpixel.py`
  - 基于 SPI 的 WS2812 灯带驱动封装（通过 SPI 波形模拟 WS2812 时序）。
- `ultrasonic.py`
  - 超声波测距封装（gpiozero DistanceSensor），返回厘米/米距离。

---

## 2) freenove_4wd_nodes（ROS2 节点包）

路径：`Robot_coche/src/freenove_4wd_nodes/`

### 2.1 构建与包元数据文件

- `package.xml`
  - ROS2 包清单文件，声明对 `rclpy`、消息类型（std_msgs/geometry_msgs/sensor_msgs）以及 `freenove_4wd_hw` 的运行依赖。
- `setup.py`
  - Python 包安装入口，并通过 `entry_points.console_scripts` 注册可执行节点命令（`ros2 run ...`）。
- `setup.cfg`
  - ament_python 常用配置。
- `resource/freenove_4wd_nodes`
  - ament 索引资源文件（用于 ROS2 发现该包）。
- `launch/bringup.launch.py`
  - 启动文件：一条命令同时启动底盘节点与相机节点（bringup）。

### 2.2 Python 节点目录：freenove_4wd_nodes/

路径：`Robot_coche/src/freenove_4wd_nodes/freenove_4wd_nodes/`

- `__init__.py`
  - 包初始化文件（空文件即可，表示这是一个 Python package）。
- `car_base_node.py`
  - 底盘/硬件 ROS2 节点（rclpy）：
    - 订阅 `/cmd_vel`（Twist）驱动电机
    - 发布超声波、红外循迹、光敏、电池电压等话题
    - 订阅蜂鸣器、舵机、LED 模式等控制话题
    - 内部调用 `freenove_4wd_hw` 中的 Car/Led/Buzzer 等模块
- `camera_driver.py`
  - 相机驱动（从原 `Code/Server/camera.py` 复制），基于 Picamera2 输出 JPEG 帧。
  - 作为“驱动层”被 `camera_node.py` 调用，减少 ROS2 代码与底层相机实现的耦合。
- `camera_node.py`
  - 相机 ROS2 节点（rclpy）：
    - 后台线程持续取帧（因为 `get_frame()` 会阻塞等待新帧）
    - 发布 `/image_raw/compressed`（sensor_msgs/CompressedImage，JPEG）
    - 支持参数化：分辨率、翻转、发布间隔等

