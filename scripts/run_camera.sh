#!/usr/bin/env bash
set -eo pipefail
cd ~/ros2_iron
source install/setup.bash

# Execute the camera node with the CORRECT parameter names
ros2 run camera_ros camera_node --ros-args \
    -p width:=640 \
    -p height:=480 \
    -p framerate:=15.0 \
    -p camera_id:=0
