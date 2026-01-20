from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    
    # Define the path to the scripts directory
    scripts_dir = os.path.expanduser('~/ros2_iron/scripts')

    return LaunchDescription([
        # Execute each of our five nodes using their wrapper scripts.
        # This ensures each node starts in its own clean, correctly configured environment.
        
        ExecuteProcess(
            cmd=['bash', os.path.join(scripts_dir, 'run_camera.sh')],
            name='camera_node_process',
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['bash', os.path.join(scripts_dir, 'run_audio_capture.sh')],
            name='audio_capture_process',
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['bash', os.path.join(scripts_dir, 'run_speech_recognition.sh')],
            name='speech_recognition_process',
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['bash', os.path.join(scripts_dir, 'run_speech_cmd_parser.sh')],
            name='speech_cmd_parser_process',
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['bash', os.path.join(scripts_dir, 'run_virtual_diffdrive.sh')],
            name='virtual_diffdrive_process',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['bash', os.path.join(scripts_dir, 'run_motor_driver.sh')],
            name='motor_driver_process',
            output='screen'
        )
    ])
