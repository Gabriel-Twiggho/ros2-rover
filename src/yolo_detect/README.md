# yolo_detect (ROS 2 + YOLO11 on Windows)

ROS 2 Python package for person detection from `/camera/image_raw/compressed`.

It publishes:
- `/detections` (`vision_msgs/Detection2DArray`)
- `/person_bbox` (`std_msgs/Float32MultiArray`) as `[norm_cx, norm_cy, norm_area, conf]`
- `/yolo_debug` (`sensor_msgs/Image`) with annotated detections

## Quick Start (PowerShell)

### 1) Create Python 3.8 virtual environment

```powershell
py -3.8 -m venv "$env:USERPROFILE\ros2venv"
& "$env:USERPROFILE\ros2venv\Scripts\Activate.ps1"
python --version
```

### 2) Install dependencies

```powershell
python -m pip install --upgrade pip
pip install ultralytics opencv-python numpy
```

Install PyTorch:
- CPU only:
```powershell
pip install torch torchvision torchaudio
```
- NVIDIA GPU (recommended):
```powershell
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu117
```

Then verify:
```powershell
python -c "import ultralytics, torch; print('ultralytics', ultralytics.__version__); print('cuda', torch.cuda.is_available())"
```

### 3) Build ROS 2 workspace

```powershell
. "C:\dev\ros2_iron\local_setup.ps1"
cd C:\dev\ros2_ws
colcon build --packages-select yolo_detect
. "C:\dev\ros2_ws\install\local_setup.ps1"
```

### 4) Run the node

Recommended (uses wrapper script):
```powershell
ros2 launch yolo_detect vision_pc.launch.py
```

Or direct:
```powershell
python "C:\dev\ros2_ws\src\yolo_detect\yolo_detect\yolo_detect_py.py" --ros-args -p model:=yolo11n.pt
```

### 5) Verify outputs

```powershell
ros2 topic list
ros2 topic echo /person_bbox
ros2 topic hz /detections
```

Debug viewer:
```powershell
ros2 run yolo_detect view_debug
```

## Rebuild/Restore After PC Wipe

1. Install ROS 2 Iron in `C:\dev\ros2_iron`.
2. Clone/copy workspace into `C:\dev\ros2_ws`.
3. Recreate venv (`ros2venv`) with Python 3.8.
4. Reinstall Python dependencies (Ultralytics, OpenCV, NumPy, PyTorch).
5. Build workspace:
   ```powershell
   . "C:\dev\ros2_iron\local_setup.ps1"
   cd C:\dev\ros2_ws
   colcon build --packages-select yolo_detect
   . "C:\dev\ros2_ws\install\local_setup.ps1"
   ```
6. Launch:
   ```powershell
   ros2 launch yolo_detect vision_pc.launch.py
   ```

## Testing Without Raspberry Pi

You can validate this node while Pi-side scripts are offline:
- Replay a rosbag containing `/camera/image_raw/compressed`, or
- Publish local camera frames to `/camera/image_raw/compressed`.

As long as this node publishes `/detections`, `/person_bbox`, and `/yolo_debug`, the detector path is validated on PC.

## Troubleshooting

- `ros2: command not found`
  - Source ROS first:
    ```powershell
    . "C:\dev\ros2_iron\local_setup.ps1"
    . "C:\dev\ros2_ws\install\local_setup.ps1"
    ```

- `No executable found` for `ros2 run yolo_detect yolo_detect_node`
  - Rebuild and re-source:
    ```powershell
    cd C:\dev\ros2_ws
    colcon build --packages-select yolo_detect
    . "C:\dev\ros2_ws\install\local_setup.ps1"
    ```

- `No matching distribution found for torch==...+cu117`
  - Use PyTorch index URL:
    ```powershell
    pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu117
    ```

