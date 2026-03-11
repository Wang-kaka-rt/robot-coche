#!/bin/bash
# Libcamera 0.6
export LD_LIBRARY_PATH=$HOME/Robot_coche/install/libcamera/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$HOME/Robot_coche/install/libcamera/lib/python3/dist-packages:$PYTHONPATH
export LIBCAMERA_IPA_MODULE_PATH=$HOME/Robot_coche/install/libcamera/lib/libcamera/ipa
export LIBCAMERA_IPA_CONFIG_PATH=$HOME/Robot_coche/install/libcamera/share/libcamera/ipa
export LIBCAMERA_IPA_FORCE_ISOLATION=0

# ROS2
source /opt/ros/jazzy/setup.bash
source ~/Robot_coche/install/setup.bash

# Vision nodes (manually, since COLCON_IGNORE blocks auto-discovery)
export AMENT_PREFIX_PATH=$HOME/Robot_coche/install/vision_nodes:$AMENT_PREFIX_PATH
export PYTHONPATH=$HOME/Robot_coche/install/vision_nodes/lib/python3.12/site-packages:$PYTHONPATH
export PATH=$HOME/Robot_coche/install/vision_nodes/lib/vision_nodes:$PATH

exec "$@"
