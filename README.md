# Robot Coche - Freenove 4WD Smart Car (Captur)

Robótica Interactiva 2025-2026. Freenove 4WD Smart Car with RPi 4 (8GB), Ubuntu 24.04, ROS2 Jazzy.

## Network

- Robot IP: `172.20.10.5`
- SSH: `ssh captur@172.20.10.5` (password: `robotica`)

## Quick Start (on the robot)

### Option A: Start everything with one command

```bash
ssh captur@172.20.10.5
bash ~/start_all.sh
```

This starts all 4 ROS2 nodes in order: car_base, camera, ball_detector, ball_follower.

### Option B: Start nodes individually

```bash
ssh captur@172.20.10.5
source ~/launch_robot.sh

# Terminal 1 - Hardware base (motors, servos, sensors)
ros2 run freenove_4wd_nodes car_base_node

# Terminal 2 - Camera (libcamera, ~20 Hz)
ros2 run freenove_4wd_nodes libcamera_camera_node

# Terminal 3 - Red card detector
ros2 run vision_nodes ball_detector

# Terminal 4 - Card follower (autonomous driving)
ros2 run vision_nodes ball_follower
```

## From laptop (SSH commands)

### Check nodes are running

```bash
sshpass -p 'robotica' ssh captur@172.20.10.5 'source ~/launch_robot.sh 2>/dev/null && ros2 node list'
```

### Monitor detection status

```bash
sshpass -p 'robotica' ssh captur@172.20.10.5 'source ~/launch_robot.sh 2>/dev/null && ros2 topic echo /ball_detection/status'
```

### Check camera publish rate

```bash
sshpass -p 'robotica' ssh captur@172.20.10.5 'source ~/launch_robot.sh 2>/dev/null && timeout 5 ros2 topic hz /image_raw/compressed'
```

### Check ultrasonic sensor

```bash
sshpass -p 'robotica' ssh captur@172.20.10.5 'source ~/launch_robot.sh 2>/dev/null && ros2 topic echo /ultrasonic/range --once'
```

### View node logs

```bash
sshpass -p 'robotica' ssh captur@172.20.10.5 'cat /tmp/ball_detector.log'
sshpass -p 'robotica' ssh captur@172.20.10.5 'cat /tmp/ball_follower.log'
sshpass -p 'robotica' ssh captur@172.20.10.5 'cat /tmp/camera.log'
sshpass -p 'robotica' ssh captur@172.20.10.5 'cat /tmp/car_base.log'
```

### Kill everything

```bash
sshpass -p 'robotica' ssh captur@172.20.10.5 'pkill -f "ros2 run"'
```

## Deploying Code Changes

From laptop, after editing files in `src/vision_nodes/`:

```bash
# 1. Copy source to robot
sshpass -p 'robotica' rsync -avz --exclude='venv/' --exclude='__pycache__/' \
  src/vision_nodes/ captur@172.20.10.5:~/Robot_coche/src/vision_nodes/

# 2. Rebuild on robot
sshpass -p 'robotica' ssh captur@172.20.10.5 \
  'source ~/launch_robot.sh 2>/dev/null && cd ~/Robot_coche && colcon build --packages-select vision_nodes'

# 3. Restart nodes
sshpass -p 'robotica' ssh captur@172.20.10.5 'pkill -f "ros2 run"; sleep 2; nohup bash ~/start_all.sh > /tmp/start_all.log 2>&1 &'
```

## Web Control Interface

```bash
ssh captur@172.20.10.5
source ~/launch_robot.sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then open `lan-car-control/index.html` in a browser.
