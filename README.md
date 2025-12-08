# TurtleBot3 Vision-Language Model Perception

Vision-Language Model (VLM) perception system for TurtleBot3 with Moondream2 on Jetson Xavier NX 8GB.

## Quick Links

- [Overleaf (Requires Access)](https://www.overleaf.com/8494251454nmnssbytfkyk#ee7bf5)

## 🎯 Project Overview

This project implements a hybrid perception pipeline for TurtleBot3 mobile manipulation using:
- **YOLO11** for fast object detection (30+ FPS)
- **Moondream2** VLM for physics-aware reasoning (2-3 Hz)
- **Decision fusion** for manipulation planning

The system enables the robot to:
1. **See**: Detect objects in camera feed
2. **Think**: Reason about physical properties (material, weight, fragility, graspability)
3. **Act**: Make informed manipulation decisions (GRASP, PUSH, AVOID, IGNORE)

## 📦 Repository Structure

```
moondream2_turtlebot3/
├── README.md                          # This file
├── docker-compose.yml                 # Docker configuration
├── Dockerfile                         # Container setup
├── configs/
│   └── system_config.yaml            # System configuration
├── scripts/                          # Standalone test scripts
│   ├── complete_hybrid_system.py     # Complete hybrid demo
│   ├── ros_vision_agent.py           # Legacy ROS2 prototype
│   ├── rpi_camera_node.py            # Legacy camera node
│   └── test_moondream*.py            # Model testing scripts
└── src/
    ├── perception/                    # 🆕 ROS2 Foxy Package
    │   ├── README.md                  # Package documentation
    │   ├── package.xml                # ROS2 package manifest
    │   ├── setup.py                   # Python package setup
    │   ├── install.sh                 # Quick setup script
    │   ├── requirements.txt           # Python dependencies
    │   ├── config/
    │   │   └── perception_params.yaml # Configuration
    │   ├── launch/
    │   │   ├── vlm_perception.launch.py    # Full system
    │   │   └── camera_only.launch.py       # Camera test
    │   └── turtlebot3_vlm_perception/
    │       ├── camera_publisher.py    # Camera streaming node
    │       └── vlm_reasoner.py        # VLM reasoning node
    ├── control/                       # (Future) Motion control
    └── planning/                      # (Future) Task planning
```

## 🚀 Quick Start

### For TurtleBot3 Deployment (ROS2 Package)

The main ROS2 package is located in `src/perception/`. This is production-ready code for deployment on TurtleBot3.

**See detailed instructions**: [`src/perception/README.md`](src/perception/README.md)

**Quick install**:
```bash
cd src/perception
chmod +x install.sh
./install.sh
```

**Quick run**:
```bash
# Enable max performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Launch full system
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py
```

### For Development/Testing (Standalone Scripts)

The `scripts/` folder contains standalone test scripts for development:

- `complete_hybrid_system.py` - Complete hybrid demo (no ROS2)
- `test_moondream*.py` - Various model testing scripts
- Legacy prototypes (not for production use)

**Run standalone demo**:
```bash
cd scripts
python3 complete_hybrid_system.py
```

## 🤖 Hardware Requirements

- **Robot**: TurtleBot3 (Burger/Waffle/Waffle Pi)
- **Computer**: Nvidia Jetson Xavier NX 8GB
- **Camera**: Raspberry Pi Camera Module v2 or HQ
- **Manipulator**: OpenMANIPULATOR-X (optional)

## 💻 Software Stack

- **OS**: Ubuntu 20.04
- **ROS**: ROS2 Foxy
- **ML Framework**: PyTorch 1.10+ (Jetson-optimized)
- **VLM**: Moondream2 (8-bit quantized)
- **Detection**: YOLO11n (Nano model)

## 📊 Performance Targets

| Component | Target | Actual (Jetson Xavier NX) |
|-----------|--------|---------------------------|
| Camera Stream | 30 FPS | 30 FPS |
| YOLO11 Detection | 30+ FPS | 30-40 FPS |
| Moondream2 VLM | 2-3 Hz | 2-3 Hz |
| End-to-End System | 2-3 Hz | 2-3 Hz |
| Memory Usage | <6GB RAM | ~5-6GB RAM |
| GPU Usage | <4GB | ~3-4GB |

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    TurtleBot3 + Jetson Xavier NX            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────┐         ┌─────────────────────────┐  │
│  │ Node A:         │  Image  │ Node B:                 │  │
│  │ Camera          │────────>│ VLM Reasoner            │  │
│  │ Publisher       │ 30 FPS  │                         │  │
│  └─────────────────┘         │ ┌─────────────────────┐ │  │
│         │                    │ │ YOLO11 Detection   │ │  │
│         │                    │ │ (30+ FPS)          │ │  │
│    RPi Camera v2             │ └──────────┬──────────┘ │  │
│    (GStreamer HW Accel)      │            │            │  │
│                               │            v            │  │
│                               │ ┌─────────────────────┐ │  │
│                               │ │ Moondream2 VLM     │ │  │
│                               │ │ Physics Reasoning  │ │  │
│                               │ │ (2-3 Hz)           │ │  │
│                               │ └──────────┬──────────┘ │  │
│                               │            │            │  │
│                               │            v            │  │
│                               │ ┌─────────────────────┐ │  │
│                               │ │ Decision Fusion    │ │  │
│                               │ │ GRASP/PUSH/AVOID   │ │  │
│                               │ └──────────┬──────────┘ │  │
│                               └────────────┼────────────┘  │
│                                            │               │
│                                            v               │
│                               ┌────────────────────────┐   │
│                               │ Terminal Dashboard     │   │
│                               │ (Real-time Display)    │   │
│                               └────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## 🎯 Features

### Vision (YOLO11)
- ✅ Real-time object detection
- ✅ 80+ object classes (COCO dataset)
- ✅ Bounding box localization
- ✅ Confidence scoring

### Reasoning (Moondream2)
- ✅ Material identification (plastic, metal, glass, etc.)
- ✅ Weight estimation (light/medium/heavy)
- ✅ Fragility assessment
- ✅ Size estimation
- ✅ Reachability analysis
- ✅ Graspability evaluation

### Decision Making
- ✅ GRASP: Objects within specs (≤500g, 10-100mm, not fragile)
- ✅ PUSH: Heavy or oversized but movable objects
- ✅ AVOID: Fragile or high-risk objects
- ✅ IGNORE: Out of reach or non-blocking objects
- ✅ STOP: Uncertain situations requiring clarification

## 📈 Future Work

- [ ] Integration with motion control (`src/control/`)
- [ ] Task planning module (`src/planning/`)
- [ ] Multi-object scene understanding
- [ ] Grasp pose estimation
- [ ] Real-world manipulation experiments
- [ ] Performance optimization for TurtleBot3 Burger (Raspberry Pi 4)

## 🐛 Known Issues

1. **bitsandbytes**: May not install on Jetson (fallback to FP16)
2. **Memory**: First VLM query may take 10-15s (model loading)
3. **Camera**: Requires proper cable seating and permissions

## 📚 Documentation

- **Package README**: `src/perception/README.md` (detailed setup & usage)
- **Configuration**: `src/perception/config/perception_params.yaml`
- **API Reference**: See docstrings in Python files

## 🙏 Acknowledgments

- [Moondream2](https://huggingface.co/vikhyatk/moondream2) by Vikhyat Korrapati
- [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
- [ROS2](https://docs.ros.org/en/foxy/)
- [Nvidia Jetson](https://developer.nvidia.com/embedded/jetson-xavier-nx)

## 📄 License

MIT License

## 👥 Team

CS7389K Group Project
