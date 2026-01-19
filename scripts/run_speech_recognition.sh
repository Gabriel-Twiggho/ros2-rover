#!/usr/bin/env bash
set -eo pipefail
cd ~/ros2_iron
source ~/ros2_venv/bin/activate
source install/setup.bash
python3 src/speech_recognition_node/speech_recognition_node/speech_recognition_py.py
