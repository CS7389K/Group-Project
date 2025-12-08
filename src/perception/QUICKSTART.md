# TurtleBot3 VLM Perception - Quick Reference

## 🚀 Quick Commands

### Installation
```bash
cd src/perception
chmod +x install.sh check_system.sh
./install.sh                    # Full installation (includes colcon build)
./check_system.sh               # Verify installation
```

### Why colcon build?
`colcon build` registers the package with ROS2. Without it:
- ❌ `ros2 launch` commands won't work
- ❌ `ros2 run` commands won't find nodes
- ❌ Launch files won't be accessible

**After any code changes, rebuild:**
```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select turtlebot3_vlm_perception
source install/setup.bash
```

### Running the System
```bash
# IMPORTANT: Source workspace first (every new terminal!)
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

# Enable max performance (Jetson)
sudo nvpmodel -m 0
sudo jetson_clocks

# Test camera only
ros2 launch turtlebot3_vlm_perception camera_only.launch.py

# Run full system
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py

# With custom parameters
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    analysis_rate:=2.0 \
    detection_threshold:=0.6 \
    camera_width:=640 \
    camera_height:=480
```

### Auto-source on Terminal Startup
```bash
# Add to .bashrc (one-time setup)
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Reload
source ~/.bashrc
```

### Viewing Camera Feed (Optional)
```bash
# Install image viewer (if not already installed)
sudo apt install ros-foxy-rqt-image-view

# Run viewer (no need to specify topic in command)
ros2 run rqt_image_view rqt_image_view
# Then select /camera/image_raw from the dropdown menu
```

### Debugging
```bash
# Check nodes
ros2 node list

# Check topics
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --no-arr

# Check camera hardware
ls /dev/video*
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink

# Monitor resources
tegrastats                      # Jetson stats
htop                           # CPU/RAM usage
nvidia-smi                     # GPU usage (if available)

# Check logs
ros2 run rqt_console rqt_console
```

## 📊 Parameters

### Camera Publisher (`camera_publisher`)
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_width` | int | 640 | Frame width |
| `camera_height` | int | 480 | Frame height |
| `camera_fps` | int | 30 | Frame rate |
| `flip_method` | int | 0 | 0=none, 2=180° |

### VLM Reasoner (`vlm_reasoner`)
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `analysis_rate` | float | 2.5 | VLM analysis Hz |
| `detection_threshold` | float | 0.5 | YOLO confidence |
| `use_yolo` | bool | true | Enable detection |
| `robot_payload_g` | int | 500 | Max payload (g) |
| `robot_gripper_min_mm` | int | 10 | Min gripper (mm) |
| `robot_gripper_max_mm` | int | 100 | Max gripper (mm) |

### Runtime Parameter Changes
```bash
# List parameters
ros2 param list /vlm_reasoner

# Get parameter
ros2 param get /vlm_reasoner analysis_rate

# Set parameter
ros2 param set /vlm_reasoner analysis_rate 2.0
ros2 param set /vlm_reasoner detection_threshold 0.6
```

## 🎯 Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | 30 Hz | Camera stream |

## 🤖 Action Decisions

| Action | Condition | Example |
|--------|-----------|---------|
| **GRASP** | Graspable object within specs | Plastic cup (50g, 45mm) |
| **PUSH** | Too large/heavy but movable | Box (800g, 150mm) |
| **AVOID** | Fragile or high-risk | Glass bottle |
| **IGNORE** | Not blocking or unreachable | Background objects |
| **STOP** | Uncertain conditions | Unknown object |

## 🔧 Optimization

### Performance Modes (Jetson)
```bash
# Max performance (15W)
sudo nvpmodel -m 0
sudo jetson_clocks

# Balanced (10W)
sudo nvpmodel -m 1

# Power save (5W)
sudo nvpmodel -m 2

# Check current mode
sudo nvpmodel -q
```

### Memory Management
```bash
# Clear memory
sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'

# Monitor memory
watch -n 1 free -h

# If out of memory, reduce:
# - analysis_rate (2.5 → 1.5 Hz)
# - camera resolution (640x480 → 320x240)
# - disable YOLO (use_yolo:=false)
```

### Temperature Monitoring
```bash
# Continuous monitoring
tegrastats

# Single reading
cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

## 🐛 Common Issues

### Issue: Camera not found
```bash
# Check connection
ls /dev/video*

# Test camera
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink

# Fix permissions
sudo usermod -aG video $USER
# Then logout/login
```

### Issue: Out of memory
```bash
# Free memory
sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'

# Reduce load
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    analysis_rate:=1.5 \
    camera_width:=320 \
    camera_height:=240
```

### Issue: VLM too slow (<2 Hz)
```bash
# Enable max performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Reduce camera resolution
ros2 param set /camera_publisher camera_width 320
ros2 param set /camera_publisher camera_height 240

# Check temperature (thermal throttling?)
tegrastats
```

### Issue: YOLO download fails
```bash
# Pre-download model
python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"

# Or disable YOLO
ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py \
    use_yolo:=false
```

## 📁 File Locations

```
~/ros2_ws/
├── src/turtlebot3_vlm_perception/    # Source code
├── build/                             # Build artifacts
├── install/                           # Installed package
└── log/                              # Build logs

~/.ros/                                # ROS logs
~/.cache/huggingface/                  # Model cache
~/.cache/ultralytics/                  # YOLO models
```

## 🔍 Useful Tools

```bash
# View images
ros2 run rqt_image_view rqt_image_view

# Plot topics
ros2 run rqt_plot rqt_plot

# Node graph
ros2 run rqt_graph rqt_graph

# Monitor system
rqt
```

## 📞 Support

- **Package README**: `src/perception/README.md`
- **System check**: `./check_system.sh`
- **Logs**: `~/.ros/log/`
- **GitHub Issues**: (your repo URL)

## 🎓 Learning Resources

- [ROS2 Foxy Docs](https://docs.ros.org/en/foxy/)
- [Jetson Xavier NX Guide](https://developer.nvidia.com/embedded/jetson-xavier-nx)
- [Moondream2 Model Card](https://huggingface.co/vikhyatk/moondream2)
- [YOLO11 Docs](https://docs.ultralytics.com/)
