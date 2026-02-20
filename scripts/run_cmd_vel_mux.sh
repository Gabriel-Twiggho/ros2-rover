#!/usr/bin/env bash
set -eo pipefail
cd ~/ros2_iron
source ~/ros2_venv/bin/activate
source install/setup.bash
python3 src/cmd_vel_mux/cmd_vel_mux/cmd_vel_mux_py.py
