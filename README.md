# TurtleBot3 VLM Perception System

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-blue.svg)](https://docs.ros.org/en/foxy/)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

> A hybrid vision-language perception system for TurtleBot3 mobile manipulation, combining YOLO11 object detection with Moondream2 VLM reasoning on Jetson Xavier NX.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building](#building)
- [Running](#running)
- [System Architecture](#system-architecture)
- [Performance](#performance)
- [Known Issues](#known-issues)
- [Resources](#resources)
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

### Project Documentation

- 📄 [Overleaf Report](https://www.overleaf.com/8494251454nmnssbytfkyk#ee7bf5) (Requires Access)
- 📦 [Package Overview](PACKAGE_OVERVIEW.md)

## Prerequisites

- **Operating System**: Ubuntu 20.04 LTS
- **ROS2**: Foxy
- **Python**: 3.8+
- **Hardware**: 
  - TurtleBot3 (Burger/Waffle/Waffle Pi)
  - Nvidia Jetson Xavier NX 8GB
  - Raspberry Pi Camera Module v2 or HQ
  - OpenMANIPULATOR-X (optional)

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/CS7389K/Group-Project.git
cd Group-Project
```

### 2. Install ROS2 Foxy

Follow the [official ROS2 Foxy installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

### 3. Install Dependencies

Navigate to the perception package and run the setup script:

```bash
cd src/perception
chmod +x install.sh
./install.sh
```

This will:
- Create/setup ROS2 workspace
- Install system dependencies
- Install Python packages (PyTorch, Ultralytics, Moondream2, etc.)
- Build the ROS2 package

### 4. Enable Jetson Performance Mode

For optimal performance on Jetson Xavier NX:

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

## Building

Navigate to your ROS2 workspace and build:

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot3_vlm_perception
source install/setup.bash
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
