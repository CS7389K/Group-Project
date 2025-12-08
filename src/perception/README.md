# TurtleBot3 VLM Perception Package

Vision-Language Model (VLM) perception system for TurtleBot3 with Moondream2 on Jetson Xavier NX 8GB.

## 🎯 Overview

This ROS2 Foxy package implements a hybrid perception pipeline combining:
- **YOLO11** object detection (fast, 30+ FPS capable)
- **Moondream2** VLM reasoning (physics-aware, 2-3 Hz target)
- **Decision fusion** for manipulation planning

The system analyzes camera feed in real-time and displays:
- What the robot **sees** (object detection)
- What the robot **thinks** (physical reasoning)
- What the robot **should do** (action decisions)

## 🤖 Hardware Requirements

- **Robot**: TurtleBot3 (Burger/Waffle/Waffle Pi)
- **Computer**: Nvidia Jetson Xavier NX 8GB
- **Camera**: Raspberry Pi Camera Module v2 or HQ
- **Manipulator**: OpenMANIPULATOR-X (optional, for grasping actions)

## 📦 Software Requirements

- **OS**: Ubuntu 20.04
- **ROS**: ROS2 Foxy
- **CUDA**: 10.2+ (included with JetPack)
- **Python**: 3.8+

## 🚀 Installation

### 1. Install ROS2 Foxy

If not already installed:
```bash
# Follow official ROS2 Foxy installation guide
# https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
```

### 2. Clone Repository

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Copy the perception package
cp -r /path/to/moondream2_turtlebot3/src/perception ./turtlebot3_vlm_perception

cd ~/ros2_ws
```

### 3. Install System Dependencies

```bash
# ROS2 dependencies
sudo apt update
sudo apt install -y \
    ros-foxy-cv-bridge \
    ros-foxy-sensor-msgs \
    ros-foxy-std-msgs \
    ros-foxy-rqt-image-view \
    python3-opencv \
    python3-pip

# GStreamer for camera (hardware acceleration)
sudo apt install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev
```

### 4. Install Python Dependencies

```bash
cd ~/ros2_ws/src/turtlebot3_vlm_perception

# Install PyTorch for Jetson (CUDA-enabled)
# Use JP4.6+ compatible wheel from Nvidia
wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -O torch-1.10.0-cp38-cp38-linux_aarch64.whl
pip3 install torch-1.10.0-cp38-cp38-linux_aarch64.whl

# Install other ML dependencies
pip3 install transformers==4.30.0
pip3 install accelerate==0.20.3
pip3 install pillow

# Install YOLO11
pip3 install ultralytics

# Install quantization support (for 8-bit models)
pip3 install bitsandbytes

# System monitoring
pip3 install psutil
```

### 5. Build the Package

**Why is `colcon build` needed?**

`colcon build` is essential for ROS2 packages because it:
- Registers the package with ROS2 (enables `ros2 launch` and `ros2 run` commands)
- Installs launch files, config files, and Python modules to the correct locations
- Creates entry points for your nodes (`camera_publisher`, `vlm_reasoner`)
- Generates the `install/` directory with `setup.bash` for environment configuration

Without building, ROS2 won't recognize the package and launch commands will fail.

```bash
cd ~/ros2_ws

# Source ROS2
source /opt/ros/foxy/setup.bash

# Build the package
colcon build --packages-select turtlebot3_vlm_perception

# Source workspace (required after every build)
source install/setup.bash

# Add to .bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 🎮 Usage

### Before Running: Source the Workspace

**Every time you open a new terminal**, you must source the workspace:

```bash
# Source ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Source your workspace
source ~/ros2_ws/install/setup.bash
```

**Tip**: Add these to `~/.bashrc` to auto-source on terminal startup:
```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Test Camera Only

First verify the camera works:

```bash
# Source workspace
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch camera
ros2 launch turtlebot3_vlm_perception camera_only.launch.py

# In another terminal, check if images are being published
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --no-arr

# To view images (requires rqt_image_view - install if needed)
sudo apt install ros-foxy-rqt-image-view
ros2 run rqt_image_view rqt_image_view
# Then select /camera/image_raw from the dropdown
```

**Troubleshooting Camera:**
- Check connection: `ls /dev/video*` (should show `/dev/video0`)
- Test GStreamer: `gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink`
- If camera is upside down, set `flip_method:=2` in launch file

**If you see a WHITE or GRAY image:**

This typically means the camera is being accessed but not configured properly.

**✅ FIXED: Updated GStreamer Pipeline**

The camera publisher has been updated with the correct GStreamer pipeline based on the working CS7389K/Milestone-4 implementation. Key fixes:
- Added `sensor-id=0` parameter to nvarguscamerasrc
- Added `drop=1` to appsink to prevent buffering
- Proper format specifications

**After updating the code, rebuild:**
```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select turtlebot3_vlm_perception
source install/setup.bash

# Test camera
ros2 launch turtlebot3_vlm_perception camera_only.launch.py
```

**If still showing white/gray, try these steps:**

**Solution 1: Run Camera Diagnostic**
```bash
cd ~/ros2_ws/src/turtlebot3_vlm_perception
chmod +x test_camera.py
python3 test_camera.py

# This will test different camera configurations and save test images to /tmp/
# Check which configuration works: ls -lh /tmp/test_*.jpg
```

**Solution 2: Check Camera Module**
```bash
# Check if camera is detected by the system
ls -l /dev/video*

# Check kernel messages for camera
dmesg | grep -i imx219  # For RPi Camera v2
dmesg | grep -i imx477  # For RPi Camera HQ

# List video devices with details
sudo apt install v4l-utils
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

**Solution 3: Try Different Camera Pipeline**
```bash
# Test camera with simple command
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvoverlaysink

# If that shows white/gray, try:
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! xvimagesink
```

**Solution 4: Use V4L2 Camera Publisher (Most Reliable)**
```bash
# Stop the current node (Ctrl+C), then:
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Run V4L2 version instead
ros2 run turtlebot3_vlm_perception camera_publisher_v4l2

# In another terminal, check the output
ros2 run rqt_image_view rqt_image_view
```

```bash
# 1. Check if camera is detected
ls /dev/video*
# Should show: /dev/video0

# 2. Test camera directly with GStreamer
gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1' ! nvoverlaysink

# 3. Check camera module is enabled
# Look for "IMX219" or your camera model
dmesg | grep -i imx
dmesg | grep -i camera

# 4. Check permissions
sudo usermod -aG video $USER
# Then logout and login again

# 5. Try alternative pipeline (V4L2 instead of nvarguscamerasrc)
# Edit camera_publisher.py and replace gstreamer_pipeline with:
# "v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! appsink"

# 6. Test with simple OpenCV capture
python3 << EOF
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if ret:
    print(f"Frame captured: {frame.shape}")
    print(f"Mean pixel value: {frame.mean()}")
else:
    print("Failed to capture")
cap.release()
EOF
```

**Common causes of white image:**
- Camera cable not fully seated in connector
- Wrong CSI port (try CSI-0 vs CSI-1)
- Camera not enabled in device tree
- Incompatible GStreamer pipeline for your camera model
- Camera module defective or wrong model (RPi Camera v1 vs v2 vs HQ)

**Quick Fix: Use V4L2 fallback camera driver**

If nvarguscamerasrc doesn't work, use the V4L2 version:

```bash
# 1. Edit setup.py to use the V4L2 camera publisher
cd ~/ros2_ws/src/turtlebot3_vlm_perception

# Replace camera_publisher with camera_publisher_v4l2 in entry_points
nano setup.py
# Change line:
#   'camera_publisher = turtlebot3_vlm_perception.camera_publisher:main',
# To:
#   'camera_publisher = turtlebot3_vlm_perception.camera_publisher_v4l2:main',

# 2. Rebuild
cd ~/ros2_ws
colcon build --packages-select turtlebot3_vlm_perception
source install/setup.bash

# 3. Test again
ros2 launch turtlebot3_vlm_perception camera_only.launch.py
```

Alternatively, run the V4L2 version directly:
```bash
ros2 run turtlebot3_vlm_perception camera_publisher_v4l2
```

### Run Complete System

Launch both camera and VLM reasoning:

```bash
# 1. Source workspace (required!)
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. Enable max performance (recommended for Jetson)
sudo nvpmodel -m 0
sudo jetson_clocks

# 3. Launch complete system
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py
```

**Expected Output:**
```
============================================================
🤖 TURTLEBOT3 VISION-LANGUAGE REASONING DASHBOARD
============================================================

📷 FRAME: 1234

────────────────────────────────────────────────────────────
👁️  VISION (YOLO11 Detection)
────────────────────────────────────────────────────────────
  Object: CUP
  Confidence: 94.23%
  Bounding Box: [200, 150, 300, 315]

────────────────────────────────────────────────────────────
🧠 REASONING (Moondream2 VLM)
────────────────────────────────────────────────────────────
  💭 Thought: ✓ Graspable: plastic, light (50g), 45mm
  ⚡ Action: GRASP
  📊 Confidence: 89.50%
  ⏱️  Analysis Time: 387ms

  Physical Properties:
    • Material: plastic
    • Weight: light (~50g)
    • Size: 45mm
    • Fragility: not_fragile
    • Graspable: YES

────────────────────────────────────────────────────────────
💻 SYSTEM STATUS
────────────────────────────────────────────────────────────
  RAM: 5.2GB / 7.6GB (68.4%)
  GPU Memory: 3.1GB (Peak: 3.8GB)
  VLM Rate: 2.5 Hz (target: 2-3 Hz)
  Detection Threshold: 0.50

============================================================
Press Ctrl+C to stop
============================================================
```

### Custom Parameters

```bash
# Always source first!
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

# Adjust analysis rate (1-3 Hz recommended for Jetson)
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    analysis_rate:=2.0

# Change detection threshold
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    detection_threshold:=0.6

# Disable YOLO (VLM only mode)
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    use_yolo:=false

# Camera resolution (lower = faster)
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    camera_width:=320 camera_height:=240
```

## 📊 Performance Metrics

### Jetson Xavier NX 8GB (Expected):
- **Camera**: 30 FPS (hardware-accelerated)
- **YOLO11n**: 30+ FPS inference
- **Moondream2**: 2-3 Hz (8-bit quantized)
- **Combined System**: 2-3 Hz overall reasoning rate
- **Memory**: ~5-6GB RAM, ~3-4GB GPU

### Optimization Tips:
1. **Lower camera resolution** (320x240) for faster processing
2. **Reduce analysis_rate** (1.5-2.0 Hz) if memory constrained
3. **Use YOLO11n** (nano) instead of larger models
4. **Monitor temperature**: `tegrastats`

## 🏗️ Package Structure

```
turtlebot3_vlm_perception/
├── package.xml              # Package metadata
├── setup.py                 # Python package setup
├── setup.cfg                # Install configuration
├── requirements.txt         # Python dependencies
├── README.md               # This file
├── resource/               # Package marker
├── config/
│   └── perception_params.yaml  # Configuration
├── launch/
│   ├── vlm_perception.launch.py    # Full system
│   └── camera_only.launch.py       # Camera test
└── turtlebot3_vlm_perception/
    ├── __init__.py
    ├── camera_publisher.py    # Node A: Camera stream
    └── vlm_reasoner.py        # Node B: VLM reasoning
```

## 🔧 Node Architecture

### Node A: `camera_publisher`
- **Publishes**: `/camera/image_raw` (sensor_msgs/Image)
- **Rate**: 30 Hz
- **Pipeline**: nvarguscamerasrc → nvvidconv → appsink
- **Purpose**: Hardware-accelerated camera streaming

### Node B: `vlm_reasoner`
- **Subscribes**: `/camera/image_raw`
- **Processing**:
  1. YOLO11 object detection (every frame)
  2. Moondream2 VLM reasoning (throttled to 2-3 Hz)
  3. Decision fusion (manipulation actions)
- **Output**: Terminal dashboard (real-time)

## 🧪 Testing

```bash
# Check if nodes are running
ros2 node list

# Expected:
# /camera_publisher
# /vlm_reasoner

# Check topics
ros2 topic list

# Expected:
# /camera/image_raw
# /parameter_events
# /rosout

# Monitor topic rate
ros2 topic hz /camera/image_raw

# Expected: ~30 Hz

# Echo camera info
ros2 topic echo /camera/image_raw --no-arr
```

## 🐛 Troubleshooting

### Camera Issues

**Problem**: "Failed to open camera"
```bash
# Check camera detection
ls /dev/video*

# Test with gst-launch
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink

# Check permissions
sudo usermod -aG video $USER
# Then logout/login
```

### Model Loading Issues

**Problem**: "Out of memory" when loading Moondream2
```bash
# Free memory
sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'

# Monitor memory
watch -n 1 free -h

# Reduce analysis rate
ros2 param set /vlm_reasoner analysis_rate 1.5
```

**Problem**: YOLO download fails
```bash
# Pre-download model
cd ~/
python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"
```

### Performance Issues

**Problem**: VLM slower than 2 Hz
```bash
# Check GPU utilization
tegrastats

# Enable max performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Reduce camera resolution
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    camera_width:=320 camera_height:=240
```

## 📚 References

- [Moondream2 Model](https://huggingface.co/vikhyatk/moondream2)
- [YOLO11 Documentation](https://docs.ultralytics.com/)
- [ROS2 Foxy Docs](https://docs.ros.org/en/foxy/)
- [Jetson Xavier NX Guide](https://developer.nvidia.com/embedded/jetson-xavier-nx)

## 📄 License

MIT License

## 👥 Authors

TurtleBot3 Team - CS7389K Group Project

## 🙏 Acknowledgments

- Moondream2 by Vikhyat Korrapati
- Ultralytics YOLO team
- ROS2 community
- Nvidia Jetson team
