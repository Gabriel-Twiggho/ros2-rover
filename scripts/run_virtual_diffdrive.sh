#!/usr/bin/env bash
# This script runs the virtual differential drive node.

# Exit immediately if a command exits with a non-zero status.
set -eo pipefail

# Navigate to the ROS2 workspace root
cd ~/ros2_iron

# Source the Python Virtual Environment
source ~/ros2_venv/bin/activate

# Source the ROS2 workspace setup file
source install/setup.bash

# Run the Python node directly
python3 src/virtual_diffdrive/virtual_diffdrive/virtual_diffdrive_py.py

