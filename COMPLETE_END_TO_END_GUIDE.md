# Complete End-to-End Network Bridge System

## Overview

This is a **COMPLETE, SELF-CONTAINED** solution that includes EVERYTHING needed:

```
┌─────────────────────────────────────────────────────────────────┐
│              TurtleBot3 (Jetson Xavier NX) - ONE PACKAGE        │
│                     vlm_bridge (Complete!)                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐                                           │
│  │ Camera Publisher │  Captures frames from /dev/video0         │
│  │                  │  Publishes to /camera/image_raw           │
│  └────────┬─────────┘                                           │
│           │                                                      │
│           │ ROS2 Topic: /camera/image_raw                       │
│           │                                                      │
│           ▼                                                      │
│  ┌──────────────────────────────────────────────────┐          │
│  │  ROS2-Network Bridge                             │          │
│  │  • Subscribes to /camera/image_raw              │          │
│  │  • Converts to Base64 JPEG                      │          │
│  │  • Sends HTTP POST to remote server             │          │
│  │  • Receives JSON response                       │          │
│  │  • Publishes to /vlm/inference_result           │          │
│  └──────────────────────┬───────────────────────────┘          │
│                         │                                        │
└─────────────────────────┼────────────────────────────────────────┘
                          │
                          │ HTTP POST (over network)
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│         Remote Computer - Standalone VLM Server                  │
│                    (NO ROS2 NEEDED!)                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Flask HTTP Server                                              │
│  • Receives Base64 images                                       │
│  • Runs Moondream2 VLM inference                                │
│  • Returns JSON results                                         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
                          │
                          │ HTTP Response
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│              TurtleBot3 (Jetson Xavier NX)                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ROS2 Topic: /vlm/inference_result                              │
│  • Your navigation/manipulation code subscribes here            │
│  • Contains VLM analysis: "A red cup on the table..."          │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## What's Included - Complete Package

### On TurtleBot3 (ONE Package: `vlm_bridge`)

**Nodes:**
1. ✅ **`camera_publisher`** - Captures camera frames
2. ✅ **`ros2_network_bridge`** - Sends to VLM, receives results
3. ✅ **`vlm_client`** - Test/demo client (optional)
4. ✅ **`vlm_server`** - Alternative: run VLM on Jetson (optional)

**Launch Files:**
1. ✅ **`complete_network_bridge.launch.py`** - **ONE command to launch everything!**
2. ✅ **`network_bridge.launch.py`** - Bridge only (if camera already running)
3. ✅ **`vlm_bridge.launch.py`** - Alternative: VLM on Jetson

**Configuration:**
- ✅ `config/network_bridge_params.yaml` - All settings in one place

### On Remote Computer (NO ROS2!)

1. ✅ **`standalone_vlm_server.py`** - HTTP VLM server
2. ✅ **`standalone_requirements.txt`** - Dependencies

## Complete Installation

### Step 1: Install on TurtleBot3 (Jetson)

```bash
# Clone repository
git clone https://github.com/CS7389K/Group-Project.git ~/moondream2_turtlebot3
cd ~/moondream2_turtlebot3

# Run installation script (installs everything)
./tools/install_jetson.sh

# Source workspace
source ~/ros2_ws/install/setup.bash
```

### Step 2: Install on Remote Computer (Inference Machine)

```bash
# Clone repository
git clone https://github.com/CS7389K/Group-Project.git
cd Group-Project

# Install dependencies (NO ROS2!)
pip3 install -r standalone_requirements.txt

# For CUDA (if you have GPU):
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

## Complete Usage - Just TWO Commands!

### Command 1: Start VLM Server (Remote Computer)

```bash
# On your remote computer (desktop/laptop)
cd ~/Group-Project
python3 standalone_vlm_server.py --host 0.0.0.0 --port 5000 --device cuda

# Note the IP address (e.g., 192.168.1.100)
```

Output:
```
Starting HTTP server on 0.0.0.0:5000
Inference endpoint: http://0.0.0.0:5000/inference
Health check: http://0.0.0.0:5000/health
```

### Command 2: Start Complete System (TurtleBot3)

```bash
# On TurtleBot3 Jetson
source ~/ros2_ws/install/setup.bash

# ONE COMMAND LAUNCHES EVERYTHING!
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000
```

**That's it!** Camera + Bridge + VLM all running!

## What This ONE Command Does

```
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000
```

Automatically launches:
1. ✅ Camera node - captures from /dev/video0
2. ✅ Bridge node - sends to VLM server
3. ✅ Both nodes communicate via ROS2
4. ✅ Results published to /vlm/inference_result

## Configuration Options

### Camera Settings

```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    camera_device:=1 \
    camera_width:=1280 \
    camera_height:=720 \
    camera_fps:=15
```

### Show Camera Preview

```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    show_camera_preview:=true
```

### Adjust Inference Rate

```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    inference_rate:=1.0
```

### Custom Prompt

```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    prompt:="What objects can the robot grasp?"
```

## Monitoring & Testing

### Check Topics

```bash
# List all topics
ros2 topic list

# Should see:
#   /camera/image_raw
#   /vlm/inference_result

# Check camera feed rate
ros2 topic hz /camera/image_raw

# Check inference results
ros2 topic echo /vlm/inference_result
```

### View Camera Feed

```bash
# Install rqt if not already
sudo apt install ros-humble-rqt-image-view

# View camera
ros2 run rqt_image_view rqt_image_view
# Select /camera/image_raw from dropdown
```

### Test VLM Server Connection

```bash
# From TurtleBot3, test connection
curl http://192.168.1.100:5000/health

# Should return:
# {"status":"healthy","model_loaded":true,"inference_count":0}
```

## Using Results in Your Code

Subscribe to `/vlm/inference_result` in your navigation/manipulation code:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyRobotController(Node):
    def __init__(self):
        super().__init__('my_controller')
        
        # Subscribe to VLM results
        self.subscription = self.create_subscription(
            String,
            '/vlm/inference_result',
            self.vlm_callback,
            10
        )
    
    def vlm_callback(self, msg):
        """Handle VLM inference results"""
        result = msg.data
        self.get_logger().info(f'VLM sees: {result}')
        
        # Your logic here
        if 'cup' in result.lower():
            self.get_logger().info('Cup detected! Planning grasp...')
        elif 'obstacle' in result.lower():
            self.get_logger().info('Obstacle detected! Avoiding...')

def main():
    rclpy.init()
    node = MyRobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

### Camera Issues

```bash
# List available cameras
ls -la /dev/video*

# Test camera with v4l2
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0

# Fix permissions
sudo usermod -aG video $USER
# Log out and back in

# Try different camera device
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    camera_device:=1
```

### Network Issues

```bash
# Test server reachability
ping 192.168.1.100

# Test VLM server
curl http://192.168.1.100:5000/health

# Check firewall (on VLM server)
sudo ufw allow 5000
```

### No Inference Results

```bash
# Check if bridge is running
ros2 node list | grep bridge

# Check if camera is publishing
ros2 topic hz /camera/image_raw

# Check bridge logs
ros2 node info ros2_network_bridge

# Test with verbose output
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    --ros-args --log-level debug
```

## Performance

**Expected Performance:**
- Camera: 30 FPS
- VLM Inference: 2 Hz (configurable)
- Network latency: ~50-100ms (local WiFi)
- Total pipeline: ~400-650ms per inference

**Tuning:**
```bash
# For faster inference (requires good network)
inference_rate:=3.0

# For slower/unreliable network
inference_rate:=1.0
timeout:=15.0

# For lower bandwidth
camera_width:=320
camera_height:=240
```

## System Architecture Summary

### All Files in One Package (`vlm_bridge`)

**Python Nodes:**
- `camera_publisher.py` - Camera capture
- `ros2_network_bridge.py` - ROS2↔HTTP bridge
- `vlm_server.py` - Alternative: local VLM
- `vlm_client.py` - Test client

**Launch Files:**
- `complete_network_bridge.launch.py` - **Complete system** ⭐
- `network_bridge.launch.py` - Bridge only
- `vlm_bridge.launch.py` - Alternative architecture

**Config:**
- `config/network_bridge_params.yaml` - Settings

**Package Files:**
- `setup.py` - All 4 nodes registered
- `package.xml` - Dependencies
- `setup.cfg` - Installation config

## Comparison with Other Approaches

| Approach | Camera | VLM | Network | ROS2 on VLM Machine |
|----------|--------|-----|---------|---------------------|
| **Complete Network Bridge** (NEW) | ✅ Included | ✅ Remote | HTTP | ❌ No |
| vlm_reasoner (existing) | ✅ Included | ✅ On Jetson | None | N/A |
| vlm_bridge (existing) | ❌ External | ✅ Local/Remote | ROS2 DDS | ✅ Yes |

**Complete Network Bridge is the most complete solution:**
- ✅ Everything in one package
- ✅ ONE launch command
- ✅ No ROS2 on inference machine
- ✅ Self-contained and portable

## Next Steps

1. **Deploy and test:**
   ```bash
   ros2 launch vlm_bridge complete_network_bridge.launch.py \
       vlm_server_url:=http://YOUR_IP:5000
   ```

2. **Verify operation:**
   ```bash
   ros2 topic echo /vlm/inference_result
   ```

3. **Integrate with your robot code:**
   - Subscribe to `/vlm/inference_result`
   - Use results for navigation/manipulation

4. **Tune performance:**
   - Adjust `inference_rate` for your needs
   - Configure camera resolution if needed

## Documentation

- **This file** - Complete end-to-end guide
- `NETWORK_BRIDGE_GUIDE.md` - Detailed network bridge documentation
- `QUICK_REFERENCE_NETWORK_BRIDGE.md` - Command reference
- `README.md` - Main project documentation

## Summary

✅ **Complete end-to-end solution**
✅ **Camera included** - no external dependencies
✅ **ONE launch command** - everything starts together
✅ **Self-contained package** - all files in `vlm_bridge`
✅ **Remote VLM** - no ROS2 needed on inference machine
✅ **Ready to use** - just configure IP address and launch!
