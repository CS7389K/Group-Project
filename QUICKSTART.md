# Quick Start Guide - TurtleBot3 VLM on Jetson

## One-Line Installation

```bash
curl -sSL https://raw.githubusercontent.com/CS7389K/Group-Project/main/tools/install_jetson.sh | bash
```

Or clone and run:

```bash
git clone https://github.com/CS7389K/Group-Project.git ~/moondream2_turtlebot3
cd ~/moondream2_turtlebot3
./tools/install_jetson.sh
```

## After Installation

### Launch Options

**Option 1: Standalone VLM Server**
```bash
ros2 launch vlm_bridge vlm_bridge.launch.py
```

**Option 2: Complete Perception System (YOLO + VLM)**
```bash
ros2 launch turtlebot3_vlm_perception vlm.launch.py
```

**Option 3: Camera Only**
```bash
ros2 launch turtlebot3_vlm_perception camera_only.launch.py
```

### Common Commands

```bash
# List available packages
ros2 pkg list | grep -E "vlm|turtlebot3"

# Check active topics
ros2 topic list

# View camera feed
ros2 run rqt_image_view rqt_image_view

# Monitor VLM responses
ros2 topic echo /vlm/response

# Change quantization
ros2 launch vlm_bridge vlm_bridge.launch.py quantization:=4bit
```

### Performance Tips

```bash
# Max performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Monitor resources
tegrastats  # GPU/CPU/Memory
htop        # Processes
```

## Directory Structure

```
~/moondream2_turtlebot3/
├── ros2_bridge/              # VLM Bridge package
│   ├── vlm_bridge/           # Python nodes
│   ├── launch/               # Launch files
│   ├── config/               # Configuration
│   └── README.md
├── src/perception/           # Full perception stack
│   └── turtlebot3_vlm_perception/
└── tools/
    └── install_jetson.sh     # Installation script

~/ros2_ws/                    # ROS2 workspace
└── src/
    ├── vlm_bridge           -> symlink to ros2_bridge
    └── turtlebot3_vlm_perception -> symlink to src/perception
```

## Troubleshooting Quick Reference

| Issue | Solution |
|-------|----------|
| Out of memory | Use `quantization:=4bit` |
| Camera not found | Check `v4l2-ctl --list-devices` |
| Model download slow | Pre-download with Python script |
| TLS allocation error | Already fixed in code with preload |
| Type error | Install `transformers<4.46.0` |

## Full Documentation

- Installation: `INSTALL.md`
- VLM Bridge: `ros2_bridge/README.md`
- Package Overview: `PACKAGE_OVERVIEW.md`
