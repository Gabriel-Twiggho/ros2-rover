"""
vision_pc.launch.py - Launch file for the YOLO detection node on Windows.

Usage:
    ros2 launch yolo_detect vision_pc.launch.py

What this does:
    Instead of using the standard 'Node' action (which has Python version conflicts
    on Windows), we use 'ExecuteProcess' to run our PowerShell wrapper script.
    The wrapper script handles sourcing ROS 2, setting up the environment, and
    running the node with the correct Python 3.8 interpreter.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    """
    ROS 2 calls this function to get the list of things to launch.
    Returns a LaunchDescription containing all the actions (nodes, processes, etc.)
    """
    
    # Path to our PowerShell wrapper script
    # We use raw strings (r'...') so backslashes aren't treated as escape characters
    script_path = r'C:\dev\ros2_ws\scripts\run_yolo_detect.ps1'

    return LaunchDescription([
        # ExecuteProcess runs an external command (like running something in terminal)
        # We're using it to run PowerShell, which then runs our wrapper script
        ExecuteProcess(
            cmd=[
                'powershell',               # The program to run
                '-ExecutionPolicy', 'Bypass',  # Allow running .ps1 scripts without policy restrictions
                '-File', script_path        # The script to execute
            ],
            name='yolo_detect_process',     # Name shown in logs
            output='screen'                 # Show output in the terminal (vs 'log' which hides it)
        )
    ])