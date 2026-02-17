#!/usr/bin/env bash
set -eo pipefail
cd ~/ros2_iron
source ~/ros2_venv/bin/activate
source install/setup.bash
python3 src/follow_controller/follow_controller/follow_controller_py.py
