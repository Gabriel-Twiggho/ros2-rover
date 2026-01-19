#!/usr/bin/env bash
# This script runs the speech command parser node.

# Exit immediately if a command exits with a non-zero status.
set -eo pipefail

# Navigate to the ROS2 workspace root
cd ~/ros2_iron

# Source the ROS2 workspace setup file
source install/setup.bash

# Source the Python Virtual Environment
source ~/ros2_venv/bin/activate

# Run the Python node directly
python3 src/speech_cmd_parser/speech_cmd_parser/speech_cmd_parser_py.py
