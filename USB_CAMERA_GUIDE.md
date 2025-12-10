# USB Camera Setup Guide

The camera publisher now supports both CSI (GStreamer) and USB (V4L2) cameras with automatic fallback.

## Quick Start with USB Camera

### For /dev/video1 (default):
```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://YOUR_PC_IP:5000 \
    camera_device:=/dev/video1
```

### For /dev/video0:
```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://YOUR_PC_IP:5000 \
    camera_device:=/dev/video0
```

### Force USB mode (skip CSI attempt):
```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://YOUR_PC_IP:5000 \
    camera_device:=/dev/video1 \
    force_v4l2:=true
```

## How It Works

The camera publisher tries cameras in this order:
1. **CSI Camera (GStreamer)** - Tries nvarguscamerasrc first
2. **USB Camera (V4L2)** - Falls back to specified device

This means:
- If you have a CSI camera, it uses that automatically
- If CSI fails or doesn't exist, it uses USB camera
- You can force USB mode with `force_v4l2:=true`

## Check Available Cameras

```bash
# List all video devices
ls -l /dev/video*

# Get device info
v4l2-ctl --list-devices

# Test a USB camera directly
v4l2-ctl -d /dev/video1 --list-formats-ext
```

## Test USB Camera

```bash
# Quick test with v4l2
v4l2-ctl -d /dev/video1 --set-fmt-video=width=640,height=480
v4l2-ctl -d /dev/video1 --stream-mmap --stream-count=10

# Test with gstreamer
gst-launch-1.0 v4l2src device=/dev/video1 ! \
    videoconvert ! autovideosink
```

## Complete Launch Examples

### Basic (auto-detect camera):
```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000
```

### USB camera on /dev/video1 (recommended):
```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    camera_device:=/dev/video1 \
    camera_width:=640 \
    camera_height:=480
```

### Force USB, show preview windows:
```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    camera_device:=/dev/video1 \
    force_v4l2:=true \
    show_camera_preview:=true \
    show_preview:=true
```

### Different resolutions:
```bash
# 1280x720
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    camera_device:=/dev/video1 \
    camera_width:=1280 \
    camera_height:=720

# 320x240 (low bandwidth)
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    camera_device:=/dev/video1 \
    camera_width:=320 \
    camera_height:=240
```

## Verify It's Working

```bash
# Check if camera topic exists
ros2 topic list | grep camera

# Check camera publishing rate
ros2 topic hz /camera/image_raw

# See camera data
ros2 topic echo /camera/image_raw --once

# See VLM results
ros2 topic echo /vlm/inference_result
```

## Troubleshooting

### "Device does not exist"
```bash
# Find your camera
ls -l /dev/video*

# Use the correct device
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    camera_device:=/dev/videoX
```

### "Permission denied"
```bash
# Add user to video group
sudo usermod -a -G video $USER
# Log out and back in

# Or use sudo (not recommended)
sudo ros2 launch vlm_bridge complete_network_bridge.launch.py ...
```

### "Camera opened but no frames"
```bash
# Check if another process is using it
sudo lsof /dev/video1

# Kill conflicting process
sudo kill <PID>
```

### "CSI attempt delays startup"
Skip CSI check:
```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    force_v4l2:=true \
    camera_device:=/dev/video1
```

## Camera Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_device` | `/dev/video1` | USB camera device path |
| `force_v4l2` | `false` | Skip CSI, use USB directly |
| `camera_width` | `640` | Frame width |
| `camera_height` | `480` | Frame height |
| `camera_fps` | `30` | Target frame rate |
| `show_preview` | `false` | Show camera window |

## Examples by Hardware

### Logitech C920:
```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    camera_device:=/dev/video0 \
    force_v4l2:=true \
    camera_width:=1280 \
    camera_height:=720
```

### Generic USB Webcam:
```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    camera_device:=/dev/video1 \
    force_v4l2:=true \
    camera_width:=640 \
    camera_height:=480
```

### Jetson with RPi Camera (CSI):
```bash
# No camera_device needed - auto-detects CSI
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000
```

The system will automatically choose the best camera!
