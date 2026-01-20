#!/usr/bin/env/ bash
set -eo pipefail

# Navigate to workspace
cd ~/ros2_iron

#activate Venv (for pigpio)
source ~/ros2_venv/bin/activate

#source ros 2 
source install/setup.bash

# run node
python3 src/motor_driver/motor_driver/motor_driver_py.py