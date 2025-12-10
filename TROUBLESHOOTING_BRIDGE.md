# Troubleshooting: No Images or Window Showing

## Quick Diagnostics

Run this diagnostic script to check everything:
```bash
cd ~/moondream2_turtlebot3/ros2_bridge
python3 diagnose_bridge.py
```

## Common Issues & Fixes

### 1. **No Window Appearing**

**Possible causes:**
- Preview parameter disabled
- Display not available (SSH without X11)
- OpenCV can't create window

**Fix:**
```bash
# Check if preview is enabled
ros2 param get /ros2_network_bridge show_preview

# Enable it if false
ros2 param set /ros2_network_bridge show_preview true

# Or relaunch with preview enabled
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://YOUR_IP:5000 \
    show_preview:=true
```

**For SSH sessions:**
```bash
# Enable X11 forwarding when connecting
ssh -X user@jetson-ip

# Or use VNC/GUI directly on Jetson
```

### 2. **No Images Being Sent to Server**

**Check if camera is publishing:**
```bash
# List all topics
ros2 topic list

# Should see:
#   /camera/image_raw
#   /vlm/inference_result

# Check camera publishing rate
ros2 topic hz /camera/image_raw

# Should show ~30 Hz
```

**If no camera topic:**
```bash
# Check if camera node is running
ros2 node list

# Should see:
#   /camera_publisher
#   /ros2_network_bridge

# Check camera hardware
ls /dev/video*
# Should show /dev/video0 or similar

# Test camera directly
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvoverlaysink
```

**If camera node crashed:**
```bash
# Check logs
ros2 node info /camera_publisher

# Restart the launch
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://YOUR_IP:5000
```

### 3. **Bridge Node Not Processing Images**

**Check if bridge is subscribed:**
```bash
# See bridge node details
ros2 node info /ros2_network_bridge

# Should show:
#   Subscribers:
#     /camera/image_raw: sensor_msgs/msg/Image
#   Publishers:
#     /vlm/inference_result: std_msgs/msg/String
```

**Check bridge logs:**
```bash
# Look for these messages in terminal where you launched:
#   "Initializing ROS2-Network Bridge..."
#   "Waiting for images on /camera/image_raw..."
#   "Sending image to VLM server (frame X)..."

# If you see errors about VLM server, check server connection
```

### 4. **VLM Server Connection Issues**

**Test server health:**
```bash
# From Jetson, ping the server
curl http://YOUR_SERVER_IP:5000/health

# Should return: {"status": "healthy", "model": "..."}
```

**If connection refused:**
```bash
# On the PC running VLM server:

# 1. Check if server is running
ps aux | grep standalone_vlm_server

# 2. Start it if not running
python3 standalone_vlm_server.py --host 0.0.0.0 --port 5000

# 3. Check firewall
sudo ufw status
sudo ufw allow 5000/tcp

# 4. Verify it's listening on all interfaces
netstat -tuln | grep 5000
# Should show 0.0.0.0:5000 or :::5000
```

**Wrong IP address:**
```bash
# Find PC IP address
ip addr show | grep "inet "

# Use that IP in launch command
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://CORRECT_IP:5000
```

### 5. **Inference Rate Too Low**

**Check inference parameter:**
```bash
ros2 param get /ros2_network_bridge inference_rate

# Default is 2.0 Hz (every 0.5 seconds)
```

**Increase rate:**
```bash
# Set to 5 Hz (every 0.2 seconds)
ros2 param set /ros2_network_bridge inference_rate 5.0

# Or relaunch
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://YOUR_IP:5000 \
    inference_rate:=5.0
```

### 6. **Silent Failures (No Error Messages)**

**Enable more verbose logging:**
```bash
# Launch with debug output
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://YOUR_IP:5000 \
    --ros-args --log-level DEBUG
```

**Monitor all messages:**
```bash
# Terminal 1: Watch camera
ros2 topic echo /camera/image_raw

# Terminal 2: Watch VLM results
ros2 topic echo /vlm/inference_result

# Terminal 3: Monitor bridge logs
ros2 node info /ros2_network_bridge
```

## Complete Test Procedure

### Step 1: Verify Camera Works
```bash
# Test camera directly with GStreamer
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
    'video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1' ! \
    nvoverlaysink
```
Press Ctrl+C to stop. You should see camera feed.

### Step 2: Start VLM Server (on PC)
```bash
# On your PC (not Jetson)
python3 standalone_vlm_server.py --host 0.0.0.0 --port 5000 --device cuda

# Should see:
#   "Initializing Moondream2 VLM..."
#   "Model loaded successfully!"
#   "Starting HTTP server on 0.0.0.0:5000"
```

### Step 3: Test Server Connection (from Jetson)
```bash
# Replace 192.168.1.100 with your PC's IP
curl http://192.168.1.100:5000/health

# Should return JSON with status "healthy"
```

### Step 4: Launch Complete System (on Jetson)
```bash
cd ~/moondream2_turtlebot3
source install/setup.bash

ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    show_preview:=true
```

### Step 5: Verify Everything is Working
```bash
# In separate terminals:

# Check camera is publishing
ros2 topic hz /camera/image_raw
# Should show ~30 Hz

# Check VLM results are coming
ros2 topic echo /vlm/inference_result
# Should see inference results every ~0.5 seconds

# You should also see a window titled "VLM Bridge - Camera Feed"
```

## Expected Output

**When working correctly, you should see:**

1. **Camera Publisher logs:**
   ```
   [camera_publisher]: Initializing camera with GStreamer pipeline
   [camera_publisher]: Camera opened successfully!
   [camera_publisher]: Ready to publish to /camera/image_raw at 30 Hz
   [camera_publisher]: Published 300 frames
   ```

2. **Bridge logs:**
   ```
   [ros2_network_bridge]: Initializing ROS2-Network Bridge...
   [ros2_network_bridge]: ROS2-Network Bridge ready!
   [ros2_network_bridge]: Waiting for images on /camera/image_raw...
   [ros2_network_bridge]: Sending image to VLM server (frame 1)...
   [ros2_network_bridge]: Inference completed in 1234ms
   [ros2_network_bridge]: Published inference result: The image shows...
   ```

3. **Window:** A CV window showing camera feed with overlays showing frame count, inference count, and last VLM result

4. **VLM Server logs (on PC):**
   ```
   POST /inference - Processing image...
   Inference completed in 0.85s
   POST /inference - 200 OK
   ```

## Still Not Working?

Run the diagnostic script and share the output:
```bash
python3 ~/moondream2_turtlebot3/ros2_bridge/diagnose_bridge.py
```

Check system resources:
```bash
# CPU/Memory usage
htop

# GPU usage (on Jetson)
tegrastats

# Disk space
df -h
```

Check ROS2 environment:
```bash
# Verify workspace is sourced
echo $ROS_DISTRO
# Should show: humble

echo $AMENT_PREFIX_PATH
# Should include your workspace paths
```
