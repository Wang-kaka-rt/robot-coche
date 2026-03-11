#!/bin/bash
export LD_LIBRARY_PATH=$HOME/Robot_coche/install/libcamera/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$HOME/Robot_coche/install/libcamera/lib/python3/dist-packages:$PYTHONPATH
export LIBCAMERA_IPA_MODULE_PATH=$HOME/Robot_coche/install/libcamera/lib/libcamera/ipa
export LIBCAMERA_IPA_CONFIG_PATH=$HOME/Robot_coche/install/libcamera/share/libcamera/ipa
export LIBCAMERA_IPA_FORCE_ISOLATION=0
source /opt/ros/jazzy/setup.bash
source ~/Robot_coche/install/setup.bash
export AMENT_PREFIX_PATH=$HOME/Robot_coche/install/vision_nodes:$AMENT_PREFIX_PATH
export PYTHONPATH=$HOME/Robot_coche/install/vision_nodes/lib/python3.12/site-packages:$PYTHONPATH
export PATH=$HOME/Robot_coche/install/vision_nodes/lib/vision_nodes:$PATH

echo "[1/4] Starting car_base_node..."
ros2 run freenove_4wd_nodes car_base_node > /tmp/car_base.log 2>&1 &
sleep 2

echo "[2/4] Starting camera..."
ros2 run freenove_4wd_nodes libcamera_camera_node > /tmp/camera.log 2>&1 &
sleep 10

echo "[3/4] Starting ball_detector..."
ros2 run vision_nodes ball_detector > /tmp/ball_detector.log 2>&1 &
sleep 2

echo "[4/4] Starting ball_follower..."
ros2 run vision_nodes ball_follower > /tmp/ball_follower.log 2>&1 &

echo "All started. Waiting..."
wait
