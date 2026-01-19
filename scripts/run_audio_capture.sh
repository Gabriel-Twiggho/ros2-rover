#!/usr/bin/env bash
set -eo pipefail
cd ~/ros2_iron
source ~/ros2_venv/bin/activate
source install/setup.bash
python3 src/audio_capture_node/audio_capture_node/audio_capture_py.py
