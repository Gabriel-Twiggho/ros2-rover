<#
.SYNOPSIS
    Runs the debug image viewer to see YOLO detections.
    
.DESCRIPTION
    Opens a window showing the camera feed with bounding boxes drawn.
    Press 'q' in the window to quit.
#>

$env:ROS_DOMAIN_ID="42"

$VENV_PATH = "$env:USERPROFILE\ros2venv"
$ROS_BASE = "C:\dev\ros2_iron\local_setup.ps1"
$ROS_WS   = "C:\dev\ros2_ws\install\local_setup.ps1"

# Source ROS 2
if (Test-Path $ROS_BASE) {
    Write-Host "Sourcing ROS 2 Base..." -ForegroundColor Green
    . $ROS_BASE
} else {
    Write-Error "ROS 2 Base not found at $ROS_BASE"
    exit 1
}

# Source Workspace
if (Test-Path $ROS_WS) {
    Write-Host "Sourcing Workspace..." -ForegroundColor Green
    . $ROS_WS
} else {
    Write-Error "Workspace setup not found at $ROS_WS"
    exit 1
}

# Run the viewer
$PYTHON_EXE = "$VENV_PATH\Scripts\python.exe"
$VIEWER_SCRIPT = "C:\dev\ros2_ws\src\yolo_detect\yolo_detect\view_debug.py"

Write-Host "Starting Debug Viewer..." -ForegroundColor Cyan
Write-Host "Press 'q' in the image window to quit." -ForegroundColor Yellow

& $PYTHON_EXE $VIEWER_SCRIPT

