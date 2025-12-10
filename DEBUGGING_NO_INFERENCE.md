# Debugging: Camera Publishing but No VLM Inference

## What You're Seeing

- ✓ Camera publisher says it's sending frames
- ✓ VLM server shows one GET request (health check)
- ✗ No POST requests to /inference endpoint
- ✗ No inference results

## What Should Be Happening

### Complete Data Flow:
```
Camera → /camera/image_raw → Bridge Node → HTTP POST → VLM Server → Response → /vlm/inference_result
  30Hz        ROS2 Topic        (2 Hz)      /inference    Python     JSON      ROS2 Topic
```

### Expected Logs:

**Camera Publisher:**
```
[camera_publisher]: Published 300 frames
[camera_publisher]: Published 600 frames
```

**Bridge Node:**
```
[ros2_network_bridge]: ✓ First camera frame received!
[ros2_network_bridge]: Received 100 frames total (inference count: 0)
[ros2_network_bridge]: Processing frame 15 for inference...
[ros2_network_bridge]: Encoded image: 45.2 KB
[ros2_network_bridge]: Sending POST to http://192.168.1.100:5000/inference
[ros2_network_bridge]: Got response: status=200
[ros2_network_bridge]: Inference completed in 1234ms
[ros2_network_bridge]: Published inference result: The image shows...
```

**VLM Server:**
```
GET /health - 200 OK
POST /inference - Processing image...
POST /inference - 200 OK (1.23s)
```

## Diagnostic Steps

### 1. Check if bridge is receiving frames:

```bash
# In the terminal where the bridge is running, look for:
# - "✓ First camera frame received!"
# - "Received 100 frames total"
# - "Processing frame X for inference..."

# If you DON'T see these messages, the bridge isn't receiving frames
```

### 2. Check ROS2 topic connection:

```bash
# See if bridge is subscribed to camera
ros2 topic info /camera/image_raw

# Should show:
# Subscription count: 1
# 
# If it shows "Subscription count: 0", the bridge isn't subscribed!
```

### 3. Check camera is actually publishing:

```bash
# Monitor camera rate
ros2 topic hz /camera/image_raw

# Should show ~30 Hz
# If it shows nothing, camera isn't publishing
```

### 4. Run diagnostic script:

```bash
cd ~/moondream2_turtlebot3/ros2_bridge
./check_bridge.sh
```

## Common Issues & Fixes

### Issue 1: Bridge Not Subscribed to Camera

**Symptom:** Camera publishes, but bridge never logs "First camera frame received"

**Cause:** QoS mismatch or bridge not running

**Fix:**
```bash
# Check if bridge is actually running
ros2 node list

# Should show both:
#   /camera_publisher
#   /ros2_network_bridge

# If bridge is missing, the launch failed
# Check the launch terminal for errors
```

### Issue 2: Frames Received but Not Processed

**Symptom:** Bridge logs "Received 100 frames" but never "Processing frame X for inference"

**Cause:** Throttling is working, but inference never triggers

**This means the time check is failing. Check inference_rate:**

```bash
ros2 param get /ros2_network_bridge inference_rate
# Should be 2.0 (meaning every 0.5 seconds)

ros2 param get /ros2_network_bridge timeout
# Should be 10.0 seconds
```

### Issue 3: Processing Starts but Connection Fails

**Symptom:** Bridge logs "Processing frame..." and "Sending POST..." but then "Cannot connect"

**Cause:** VLM server URL is wrong or server isn't running

**Fix:**
```bash
# Check server URL
ros2 param get /ros2_network_bridge vlm_server_url

# Test it
curl http://YOUR_IP:5000/health

# If that fails, check:
# 1. Is server running?
ps aux | grep standalone_vlm_server

# 2. Is it listening on correct port?
netstat -tuln | grep 5000

# 3. Firewall?
sudo ufw allow 5000/tcp
```

### Issue 4: POST Request Times Out

**Symptom:** Bridge logs "Sending POST..." but then "Request timeout (10s)"

**Cause:** VLM inference is too slow or server crashed

**Fix:**
```bash
# Check VLM server terminal - is it actually processing?
# You should see:
#   POST /inference - Processing image...
#   Inference completed in X.XXs
#   POST /inference - 200 OK

# If server is frozen/crashed, restart it:
python3 standalone_vlm_server.py --host 0.0.0.0 --port 5000 --device cuda

# Increase timeout if inference is just slow:
ros2 param set /ros2_network_bridge timeout 30.0
```

## Enable Verbose Logging

Add these logs to see exactly what's happening:

**In your launch terminal, you should see:**
```
[ros2_network_bridge]: ✓ First camera frame received!
[ros2_network_bridge]: Image size: 640x480
[ros2_network_bridge]: Encoding: bgr8
[ros2_network_bridge]: Received 100 frames total (inference count: 0)
[ros2_network_bridge]: Processing frame 15 for inference...
[ros2_network_bridge]: Encoded image: 45.2 KB
[ros2_network_bridge]: Sending POST to http://192.168.1.100:5000/inference
```

**If you see the first 3 lines:**
- ✓ Bridge IS receiving camera frames
- ✓ Camera IS publishing
- ✓ ROS2 connection works

**If you DON'T see "Processing frame X":**
- The throttling is preventing inference (expected behavior)
- Should process every 0.5 seconds (2 Hz)

**If you see "Processing..." but no "Encoded image":**
- Image conversion is failing
- Check the error message right after

**If you see "Encoded image" but no "Sending POST":**
- Something crashed between encoding and sending
- Check for error messages

**If you see "Sending POST" but no response:**
- Network issue or server not responding
- Check VLM server logs

## Manual Test

Test the bridge manually:

```bash
# Terminal 1: Monitor VLM results
ros2 topic echo /vlm/inference_result

# Terminal 2: Monitor camera
ros2 topic hz /camera/image_raw

# Terminal 3: Watch the launch output
# Look for the log messages described above
```

You should see:
1. Camera: ~30 Hz
2. Bridge logs: Processing every ~0.5 seconds  
3. VLM results: New results every ~0.5 seconds

## Check Server Side

On the PC running the VLM server:

```bash
# The server should log:
# - GET /health requests (from health checks)
# - POST /inference requests (from actual inference)

# If you ONLY see GET requests:
# - Bridge is not sending inference requests
# - Connection issue between Jetson and PC
```

## Summary

The most likely issues:

1. **Bridge not receiving frames** → Check ROS2 topic subscription
2. **Bridge receiving but not processing** → Check inference_rate and throttling logic
3. **Processing but connection fails** → Check VLM server URL and network
4. **Connection works but times out** → VLM server too slow, increase timeout

Run `./check_bridge.sh` to automatically check most of these!
