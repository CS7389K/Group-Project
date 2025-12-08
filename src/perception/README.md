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

```bash
cd ~/ros2_ws

# Source ROS2
source /opt/ros/foxy/setup.bash

# Build
colcon build --packages-select turtlebot3_vlm_perception

# Source workspace
source install/setup.bash
```

## 🎮 Usage

### Test Camera Only

First verify the camera works:

```bash
# Terminal 1: Launch camera
ros2 launch turtlebot3_vlm_perception camera_only.launch.py

# Terminal 2: View images
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

**Troubleshooting Camera:**
- Check connection: `ls /dev/video*` (should show `/dev/video0`)
- Test GStreamer: `gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink`
- If camera is upside down, set `flip_method:=2` in launch file

### Run Complete System

Launch both camera and VLM reasoning:

```bash
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
# Adjust analysis rate (1-3 Hz recommended for Jetson)
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    analysis_rate:=2.0

# Change detection threshold
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    detection_threshold:=0.6

# Disable YOLO (VLM only mode)
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    use_yolo:=false

# Camera resolution
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
