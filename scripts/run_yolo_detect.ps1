<#
.SYNOPSIS
    Wrapper script to run the YOLOv8 node on Windows.
    Safely sources ROS 2 and runs the node with the correct Python executable.

.DESCRIPTION
    This script is functionally equivalent to:
        $env:ROS_DOMAIN_ID="42"
        . "C:\dev\ros2_iron\local_setup.ps1"
        . "$venv\Scripts\Activate.ps1"
        . "C:\dev\ros2_ws\install\local_setup.ps1"
        python yolo_detect_py.py

    However, instead of activating the venv (which modifies PATH), we directly
    invoke the venv's python.exe. This is more robust on Windows where multiple
    Python versions (3.8, 3.10, 3.12, 3.13) compete for PATH priority.
#>

# 1. Set the ROS Domain ID
$env:ROS_DOMAIN_ID="42"

# 2. Define Paths
# Using $env:USERPROFILE ensures this works for any user
$VENV_PATH = "$env:USERPROFILE\ros2venv"
$ROS_BASE = "C:\dev\ros2_iron\local_setup.ps1"
$ROS_WS   = "C:\dev\ros2_ws\install\local_setup.ps1"

# 3. Source Base ROS 2
if (Test-Path $ROS_BASE) {
    Write-Host "Sourcing ROS 2 Base..." -ForegroundColor Green
    . $ROS_BASE
} else {
    Write-Error "ROS 2 Base not found at $ROS_BASE"
    exit 1
}

# 4. Source Our Workspace (yolo_detect)
if (Test-Path $ROS_WS) {
    Write-Host "Sourcing Workspace..." -ForegroundColor Green
    . $ROS_WS
} else {
    Write-Error "Workspace setup not found at $ROS_WS. Did you run colcon build?"
    exit 1
}

# 5. Get the venv Python Executable (WITHOUT activating the venv)
# WHY NOT ACTIVATE? Activating a venv just prepends its Scripts folder to PATH
# so that 'python' resolves to the venv's python.exe. But on this system we have
# Python 3.8, 3.10, 3.12, and 3.13 installed — PATH conflicts are common.
# By directly calling the full path to python.exe, we guarantee the correct
# interpreter regardless of PATH state. Same result, more reliable.
$PYTHON_EXE = "$VENV_PATH\Scripts\python.exe"

if (-not (Test-Path $PYTHON_EXE)) {
    Write-Error "Virtual Environment Python not found at $PYTHON_EXE"
    exit 1
}

# 6. Define the Node Script
$NODE_SCRIPT = "C:\dev\ros2_ws\src\yolo_detect\yolo_detect\yolo_detect_py.py"

# 7. Execute
Write-Host "Starting YOLO Detect Node..." -ForegroundColor Cyan
Write-Host "Using Python: $PYTHON_EXE" -ForegroundColor Gray

# The '&' operator runs the command string provided
& $PYTHON_EXE $NODE_SCRIPT