# TurtleBot3 VLM Perception System

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04%20%7C%2022.04-orange.svg)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

> A hybrid vision-language perception system for TurtleBot3 mobile manipulation, combining YOLO11 object detection with Moondream2 VLM reasoning on Jetson Xavier NX.

## Overview

An intelligent perception pipeline for TurtleBot3 combining YOLO11 object detection (30+ FPS) with Moondream2 VLM reasoning (2-3 Hz) to enable autonomous manipulation decisions. The system analyzes material properties, physical attributes, and graspability to determine optimal robot actions.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [System Architecture](#system-architecture)
- [Troubleshooting](#troubleshooting)
- [Resources](#resources)
- [License](#license)

## Features

- **Modular ROS2 Architecture**: Separate packages for VLM inference and perception
- **Multiple Deployment Options**: On-device, remote server, or hybrid configurations
- **Jetson-Optimized**: 8-bit/4-bit quantization, memory-efficient CUDA allocation
- **Easy Setup**: Automated installation script with automatic workspace configuration

## Prerequisites

- **Hardware**: 
  - Computer capable of running the VLM server
  - TurtleBot3 (Burger/Waffle/Waffle Pi) running a NVIDIA Jetson Xavier NX 8GB
    - USB Camera or Raspberry Pi Camera Module
    - OpenMANIPULATOR-X (optional)
- **Operating System**: 
  - TurtleBot3: Ubuntu 20.04 
  - VLM Server: Ubuntu 22.04 LTS
- **ROS2**: Foxy
- **Python**: 3.8+

## Usage

```bash
# On Both the Remote Machine and Turtlebot3 Jetson:
git clone https://github.com/CS7389K/Group-Project.git ~/turtlebot3_vlm
cd ~/turtlebot3_vlm

# On the Remote Machine:
# First, start the Flask VLM server (required for vlm_bridge)
python3 scripts/vlm_ros_server.py

# In a separate terminal on the same Remote Machine:
colcon build --symlink-install --packages-select vlm_bridge
source ./install/setup.bash
ros2 launch vlm_bridge vlm_bridge.launch.py

# On Turtlebot3 Jetson:
colcon build --symlink-install --packages-select turtlebot3_vlm_perception
source ./install/setup.bash
ros2 launch vlm_bridge vlm_with_yolo.launch.py
# Or Without YOLO:
# ros2 launch turtlebot3_vlm_perception vlm.launch.py
```

**Important**: The Flask server (`vlm_ros_server.py`) must be running on the same machine as `vlm_bridge` to handle VLM inference requests.

### Download VLM Models

```bash
cd scripts
python3 download_vlm_models.py --output-dir ./vlm_models
```

## Project Structure

```
├── src/
│   ├── turtlebot3_vlm_perception/    # TurtleBot3 ROS2 client
│   │   └── launch/                   # Launch files for VLM perception stack
│   └── vlm_bridge/                   # VLM inference server
│       └── launch/                   # Launch files for VLM server & bridges
├── tools/
│   ├── git_fetch_and_build.sh        # Git update and build helper
│   └── restart_camera.sh             # Camera troubleshooting utility
├── scripts/
│   └── download_vlm_models.py        # Download VLM
├── pyproject.toml                    # Python project configuration
└── docker-compose.yml                # Docker deployment configuration
```

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    TurtleBot3 + Jetson Xavier NX            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────┐         ┌──────────────────────────┐   │
│  │ TurtleBot3      │         │ VLM Reasoner             │   │
|  | Camera          | Image   |                          |   |
│  │ Publisher       │────────>│                          │   │
│  │ (30 FPS)        │         │ ┌────────────────────┐   │   │
│  └─────────────────┘         │ │ YOLO11 Detection   │   │   │
│         │                    │ │ (30+ FPS)          │   │   │
│    RPi Camera v2             │ └─────────┬──────────┘   │   │
│    (GStreamer HW Accel)      │           │              │   │
│                              │           v              │   │
│                              │ ┌────────────────────┐   │   │
│                              │ │ Moondream2 VLM     │   │   │
│                              │ │ (Physics Analysis) │   │   │
│                              │ │ (2-3 Hz)           │   │   │
│                              │ └─────────┬──────────┘   │   │
│                              │           │              │   │
│                              │           v              │   │
│                              │ ┌────────────────────┐   │   │
│                              │ │ Decision Fusion    │   │   │
│                              │ │ GRASP/PUSH/AVOID   │   │   │
│                              │ └────────────────────┘   │   │
│                              └──────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### ROS2 Topics

- `/camera/image_raw` (sensor_msgs/Image) - Camera feed from TurtleBot3
- `/yolo/detections` (std_msgs/String) - YOLO object detection results with bounding boxes
- `/vlm/response` (std_msgs/String) - VLM inference responses
- `/vlm/inference_result` (std_msgs/String) - Network bridge VLM results
- `/vlm/analysis_result` (std_msgs/String) - Integrated YOLO+VLM analysis
- `/vlm/response` (std_msgs/String) - VLM responses (by client nodes)

## Troubleshooting

If camera fails to open:

```bash
# Release camera resources
./tools/restart_camera.sh
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

**Built with ❤️ using ROS2 Foxy and Jetson Xavier NX**
