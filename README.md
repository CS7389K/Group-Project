# TurtleBot3 VLM Perception System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04%20%7C%2022.04-orange.svg)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

> A hybrid vision-language perception system for TurtleBot3 mobile manipulation, combining YOLO11 object detection with Moondream2 VLM reasoning on Jetson Xavier NX.

## 🚀 Quick Start

### Option 1: Network Bridge (Recommended for Remote Inference)

**TurtleBot3 with ROS2 → Remote Computer without ROS2**

```bash
# On Remote Computer (NO ROS2 needed):
git clone https://github.com/CS7389K/Group-Project.git
cd Group-Project
pip3 install -r standalone_requirements.txt
python3 standalone_vlm_server.py --device cuda --port 5000

# On TurtleBot3 Jetson:
git clone https://github.com/CS7389K/Group-Project.git ~/moondream2_turtlebot3
cd ~/moondream2_turtlebot3
./tools/install_jetson.sh
source ~/ros2_ws/install/setup.bash
ros2 launch vlm_bridge network_bridge.launch.py vlm_server_url:=http://192.168.1.100:5000
```

See [NETWORK_BRIDGE_GUIDE.md](NETWORK_BRIDGE_GUIDE.md) for detailed instructions.

### Option 2: Standalone VLM on Jetson

**All-in-one on Jetson Xavier NX**

```bash
# On Jetson Xavier NX - One command installation
git clone https://github.com/CS7389K/Group-Project.git ~/moondream2_turtlebot3
cd ~/moondream2_turtlebot3
./tools/install_jetson.sh

# Launch the system
source ~/ros2_ws/install/setup.bash
ros2 launch vlm_bridge vlm_bridge.launch.py
```

See [QUICKSTART.md](QUICKSTART.md) for detailed quick start guide.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Packages](#packages)
- [Usage](#usage)
- [System Architecture](#system-architecture)
- [Performance](#performance)
- [Documentation](#documentation)
- [License](#license)

## Overview

This project implements an intelligent perception pipeline for TurtleBot3 mobile manipulation that combines:

- **YOLO11 Object Detection**: Fast real-time object detection (30+ FPS) for 80+ object classes
- **Moondream2 VLM Reasoning**: Physics-aware reasoning about object properties (2-3 Hz)
- **Decision Fusion**: Intelligent manipulation planning (GRASP, PUSH, AVOID, IGNORE actions)

The system enables autonomous manipulation decisions by understanding:
- Material properties (plastic, metal, glass, etc.)
- Physical attributes (weight, size, fragility)
- Graspability and manipulation feasibility
- Risk assessment for collision avoidance

## Features

✅ **Three Deployment Options**
- **Network Bridge**: ROS2 on TurtleBot3 → HTTP → Remote VLM (no ROS2 on inference machine)
- **Standalone VLM Bridge**: ROS2 VLM server on Jetson
- **Complete Perception Stack**: Camera + YOLO + VLM all on Jetson

✅ **Flexible Architecture**
- Run VLM inference on remote computer WITHOUT ROS2
- Or run everything on Jetson Xavier NX
- Network bridge for distributed computing

✅ **Jetson-Optimized**
- 8-bit and 4-bit quantization support
- Memory-efficient CUDA allocation
- Automatic library preloading for TLS issues

✅ **Easy Installation**
- One-command installation script
- Automatic workspace setup
- Pre-download model support

✅ **Flexible Deployment**
- Launch files with configurable parameters
- HTTP REST API for inference
- Real-time performance monitoring

## Documentation

- 🌐 [**Network Bridge Guide**](NETWORK_BRIDGE_GUIDE.md) - **NEW!** Run VLM on remote computer without ROS2
- 📖 [Installation Guide](INSTALL.md) - Complete installation instructions
- 🚀 [Quick Start Guide](QUICKSTART.md) - Get started in minutes
- 📦 [Package Overview](PACKAGE_OVERVIEW.md) - Detailed package documentation
- 🤖 [VLM Bridge README](ros2_bridge/README.md) - VLM server package details
- 📄 [Overleaf Report](https://www.overleaf.com/8494251454nmnssbytfkyk#ee7bf5) (Requires Access)

## Prerequisites

- **Operating System**: Ubuntu 20.04 or 22.04 LTS
- **ROS2**: Humble (Foxy also supported)
- **Python**: 3.8+
- **Hardware**: 
  - TurtleBot3 (Burger/Waffle/Waffle Pi)
  - NVIDIA Jetson Xavier NX 8GB
  - USB Camera or Raspberry Pi Camera Module
  - OpenMANIPULATOR-X (optional)

## Installation

### Automated Installation (Recommended)

```bash
# Clone the repository on Jetson
git clone https://github.com/CS7389K/Group-Project.git ~/moondream2_turtlebot3
cd ~/moondream2_turtlebot3

# Run installation script
chmod +x tools/install_jetson.sh
./tools/install_jetson.sh
```

The script automatically:
1. Installs system dependencies (ROS2 packages, v4l-utils, etc.)
2. Installs PyTorch for Jetson Xavier NX
3. Installs Python dependencies (transformers, YOLO, etc.)
4. Creates and configures ROS2 workspace
5. Builds both packages (`vlm_bridge` and `turtlebot3_vlm_perception`)
6. Sets up environment variables

### Manual Installation

See [INSTALL.md](INSTALL.md) for step-by-step manual installation instructions.

## Packages

This repository contains two installable ROS2 packages:

### 1. `vlm_bridge` - VLM Inference Server

Location: `ros2_bridge/`

A standalone ROS2 package providing Moondream2 VLM inference as a service.

**Nodes:**
- `vlm_server`: Runs VLM model and provides inference
- `vlm_client`: Example client for testing

**Topics:**
- Subscribes: `/camera/image_raw` (sensor_msgs/Image)
- Publishes: `/vlm/response` (std_msgs/String)

**Launch:**
```bash
ros2 launch vlm_bridge vlm_bridge.launch.py
ros2 launch vlm_bridge vlm_bridge.launch.py quantization:=4bit
ros2 launch vlm_bridge vlm_bridge.launch.py with_client:=true
```

See [ros2_bridge/README.md](ros2_bridge/README.md) for details.

### 2. `turtlebot3_vlm_perception` - Complete Perception Stack

Location: `src/perception/`

Full perception system integrating camera, YOLO detection, and VLM reasoning.

**Nodes:**
- `camera_publisher`: V4L2 camera interface
- `vlm_reasoner`: YOLO + Moondream2 hybrid reasoning

**Launch:**
```bash
ros2 launch turtlebot3_vlm_perception camera_only.launch.py
ros2 launch turtlebot3_vlm_perception vlm.launch.py
```

## Usage

### Option 1: VLM Server Only

Launch standalone VLM server:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch vlm_bridge vlm_bridge.launch.py
```

With test client:
```bash
ros2 launch vlm_bridge vlm_bridge.launch.py with_client:=true prompt:="What do you see?"
```

### Option 2: Complete Perception System

Launch full stack with camera + YOLO + VLM:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_vlm_perception vlm.launch.py
```

This will display:
- Live camera feed with YOLO detections (OpenCV window)
- Real-time reasoning dashboard (terminal)
- Object properties and manipulation decisions

### Option 3: Camera Only

Test camera setup:
```bash
ros2 launch turtlebot3_vlm_perception camera_only.launch.py
```

View with rqt:
```bash
ros2 run rqt_image_view rqt_image_view
```

For development with symlink install:

```bash
colcon build --symlink-install --packages-select turtlebot3_vlm_perception
```

## Running

### Terminal 1: Launch TurtleBot3 Base (if using robot)

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

### Terminal 2: Launch VLM Perception System

Full system with camera and VLM reasoning:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py
```

Camera-only test (no VLM processing):

```bash
ros2 launch turtlebot3_vlm_perception camera_only.launch.py
```

### Testing with Standalone Scripts

For development and testing without ROS2:

```bash
cd scripts
python3 complete_hybrid_system.py
```

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    TurtleBot3 + Jetson Xavier NX            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────┐         ┌─────────────────────────┐  │
│  │ Camera          │  Image  │ VLM Reasoner           │  │
│  │ Publisher       │────────>│                        │  │
│  │ (30 FPS)        │         │ ┌────────────────────┐ │  │
│  └─────────────────┘         │ │ YOLO11 Detection  │ │  │
│         │                    │ │ (30+ FPS)         │ │  │
│    RPi Camera v2             │ └─────────┬─────────┘ │  │
│    (GStreamer HW Accel)      │           │           │  │
│                               │           v           │  │
│                               │ ┌────────────────────┐ │  │
│                               │ │ Moondream2 VLM    │ │  │
│                               │ │ (Physics Analysis)│ │  │
│                               │ │ (2-3 Hz)          │ │  │
│                               │ └─────────┬─────────┘ │  │
│                               │           │           │  │
│                               │           v           │  │
│                               │ ┌────────────────────┐ │  │
│                               │ │ Decision Fusion   │ │  │
│                               │ │ GRASP/PUSH/AVOID  │ │  │
│                               │ └───────────────────┘ │  │
│                               └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### ROS2 Topics

- `/camera/image_raw` - Camera feed (sensor_msgs/Image)
- `/vlm/detections` - Object detections with decisions (custom message)

## Performance

Performance metrics on Jetson Xavier NX 8GB:

| Component | Target | Actual |
|-----------|--------|--------|
| Camera Stream | 30 FPS | 30 FPS |
| YOLO11 Detection | 30+ FPS | 30-40 FPS |
| Moondream2 VLM | 2-3 Hz | 2-3 Hz |
| End-to-End Latency | <500ms | ~400ms |
| Memory Usage | <6GB | ~5-6GB |
| GPU Memory | <4GB | ~3-4GB |

### Decision Logic

- **GRASP**: Objects ≤500g, 10-100mm, non-fragile, within reach
- **PUSH**: Heavy/oversized but movable objects
- **AVOID**: Fragile objects or collision risks
- **IGNORE**: Out of reach or non-blocking objects
- **STOP**: Uncertain situations requiring human input

## Known Issues

1. **bitsandbytes on Jetson**: May fail to install - system automatically falls back to FP16
2. **First VLM Query**: Initial model loading takes 10-15 seconds
3. **Camera Permissions**: Ensure user is in `video` group: `sudo usermod -aG video $USER`
4. **Memory Management**: First-time model download requires stable internet connection

### Troubleshooting

If camera fails to open:

```bash
# Release camera resources
./tools/restart_camera.sh

# Verify camera device
ls -la /dev/video*
v4l2-ctl --list-devices
```

## Resources

### ROS2 Documentation

- [ROS2 Foxy Installation](https://docs.ros.org/en/foxy/Installation.html)
- [Building ROS2 Packages](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [ROS2 Launch Files](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html)

### Models & Frameworks

- [Moondream2 VLM](https://huggingface.co/vikhyatk/moondream2)
- [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
- [PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)

### Hardware

- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Jetson Xavier NX](https://developer.nvidia.com/embedded/jetson-xavier-nx)
- [OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

**Built with ❤️ for CS7389K using ROS2 Foxy and Jetson Xavier NX**
