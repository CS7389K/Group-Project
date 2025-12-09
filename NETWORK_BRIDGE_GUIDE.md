# Network Bridge Architecture - ROS2 to Non-ROS VLM Server

## Overview

This architecture allows you to:
1. **Run ROS2 on TurtleBot3 (Jetson Xavier NX)** - publishes camera images
2. **Run VLM inference on a remote computer WITHOUT ROS2** - just Python + PyTorch
3. **Get results back to TurtleBot3 via ROS2** - for navigation/manipulation

```
┌────────────────────────────────────────────────────────────────────┐
│                    TurtleBot3 (Jetson Xavier NX)                    │
│                          WITH ROS2                                  │
├────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────────┐         ┌────────────────────────────┐      │
│  │  Camera Node     │ ──────> │  ROS2-Network Bridge       │      │
│  │  /camera/image   │         │                            │      │
│  │  _raw            │         │  • Subscribes: images      │      │
│  └──────────────────┘         │  • Converts: ROS→HTTP      │      │
│                                │  • Sends: to remote        │      │
│                                │  • Receives: results       │      │
│                                │  • Publishes: to ROS2      │      │
│                                └────────────┬───────────────┘      │
│                                             │                       │
└─────────────────────────────────────────────┼───────────────────────┘
                                              │
                                              │ HTTP POST
                                              │ (Base64 JPEG)
                                              │
                                              ▼
┌────────────────────────────────────────────────────────────────────┐
│              Remote Computer (Desktop/Server)                       │
│                      NO ROS2 REQUIRED!                              │
├────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────────────────────────────────────────────────┐     │
│  │         Standalone VLM HTTP Server                        │     │
│  │         (Flask + Moondream2)                             │     │
│  │                                                          │     │
│  │  • Receives: HTTP POST with image                       │     │
│  │  • Decodes: Base64 → PIL Image                          │     │
│  │  • Inference: Moondream2 VLM                            │     │
│  │  • Returns: JSON with result                            │     │
│  └──────────────────────────────────────────────────────────┘     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                                              │
                                              │ HTTP Response
                                              │ (JSON)
                                              │
                                              ▼
┌────────────────────────────────────────────────────────────────────┐
│                    TurtleBot3 (Jetson Xavier NX)                    │
├────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌────────────────────────────┐    ┌──────────────────────────┐   │
│  │  ROS2-Network Bridge       │──> │  Your Navigation Node    │   │
│  │  Publishes:                │    │  Subscribes:             │   │
│  │  /vlm/inference_result     │    │  /vlm/inference_result   │   │
│  └────────────────────────────┘    └──────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Components

### 1. On TurtleBot3 (Jetson) - WITH ROS2

**ROS2-Network Bridge** (`ros2_network_bridge.py`)
- Subscribes to `/camera/image_raw` (ROS2)
- Converts images to base64 JPEG
- Sends HTTP POST to remote VLM server
- Receives JSON response
- Publishes to `/vlm/inference_result` (ROS2)

### 2. On Remote Computer - NO ROS2 NEEDED

**Standalone VLM Server** (`standalone_vlm_server.py`)
- Pure Python HTTP server (Flask)
- Moondream2 VLM inference
- NO ROS2 installation required
- Can run on any machine with Python + CUDA

## Installation & Setup

### On TurtleBot3 (Jetson Xavier NX)

```bash
# 1. Install ROS2 bridge package (already done from previous setup)
cd ~/moondream2_turtlebot3
./tools/install_jetson.sh

# 2. Install additional dependency for HTTP client
pip3 install requests

# 3. Rebuild package
cd ~/ros2_ws
colcon build --packages-select vlm_bridge
source install/setup.bash
```

### On Remote Computer (Inference Machine)

**Requirements:** Python 3.8+, PyTorch, NO ROS2 needed

```bash
# 1. Clone repository (or just copy standalone_vlm_server.py)
git clone https://github.com/CS7389K/Group-Project.git
cd Group-Project

# 2. Install Python dependencies
pip3 install torch transformers pillow flask bitsandbytes accelerate

# For CUDA (if available):
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# 3. That's it! No ROS2 needed.
```

## Usage

### Step 1: Start VLM Server on Remote Computer

```bash
cd ~/Group-Project

# Basic usage (CPU)
python3 standalone_vlm_server.py --device cpu

# With CUDA GPU
python3 standalone_vlm_server.py --device cuda --quantization 8bit

# Custom port and host
python3 standalone_vlm_server.py --host 0.0.0.0 --port 5000 --quantization 4bit

# Full options
python3 standalone_vlm_server.py \
    --host 0.0.0.0 \
    --port 5000 \
    --model vikhyatk/moondream2 \
    --quantization 8bit \
    --device cuda
```

The server will print:
```
Starting HTTP server on 0.0.0.0:5000
Inference endpoint: http://0.0.0.0:5000/inference
Health check: http://0.0.0.0:5000/health
```

### Step 2: Find Remote Computer's IP Address

On the remote computer:
```bash
# Linux/Mac
ifconfig  # or: ip addr show

# Look for IP like 192.168.1.100
```

### Step 3: Start Camera on TurtleBot3

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_vlm_perception camera_only.launch.py
```

### Step 4: Start Bridge on TurtleBot3

In another terminal on TurtleBot3:

```bash
source ~/ros2_ws/install/setup.bash

# Use the IP address from Step 2
ros2 launch vlm_bridge network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000

# Or run the node directly
ros2 run vlm_bridge ros2_network_bridge \
    --ros-args \
    -p vlm_server_url:=http://192.168.1.100:5000 \
    -p inference_rate:=2.0
```

### Step 5: Monitor Results on TurtleBot3

In another terminal:

```bash
# Watch inference results
ros2 topic echo /vlm/inference_result

# Check data rate
ros2 topic hz /vlm/inference_result

# View all topics
ros2 topic list
```

## Configuration

### Bridge Configuration

Edit `ros2_bridge/config/network_bridge_params.yaml`:

```yaml
/**:
  ros__parameters:
    # Change this to your remote computer's IP
    vlm_server_url: "http://192.168.1.100:5000"
    
    # How often to send images (Hz)
    inference_rate: 2.0
    
    # Request timeout
    timeout: 10.0
    
    # VLM prompt
    prompt: "Describe objects in this image."
```

### Server Configuration

Command-line options for `standalone_vlm_server.py`:

```bash
--host 0.0.0.0          # Listen on all interfaces
--port 5000             # Port number
--model vikhyatk/moondream2  # HuggingFace model
--quantization 8bit     # 8bit, 4bit, or none
--device cuda           # cuda or cpu
```

## API Reference

### HTTP Inference Endpoint

**POST** `/inference`

Request:
```json
{
  "image": "<base64-encoded JPEG>",
  "prompt": "Describe this image",
  "timestamp": 1234567890.123
}
```

Response:
```json
{
  "result": "A robot gripper approaching a red cup on a table...",
  "inference_time_ms": 456.78,
  "timestamp": 1234567890.123,
  "server_timestamp": 1234567890.579
}
```

### Health Check Endpoint

**GET** `/health`

Response:
```json
{
  "status": "healthy",
  "model_loaded": true,
  "inference_count": 42
}
```

## ROS2 Topics

### Subscribed (on TurtleBot3)
- `/camera/image_raw` (sensor_msgs/Image) - Camera feed

### Published (on TurtleBot3)
- `/vlm/inference_result` (std_msgs/String) - VLM results

## Performance

### Network Latency
- Local network (WiFi): ~50-100ms
- Image transmission: ~20-50ms (JPEG compressed)
- VLM inference: ~300-500ms
- **Total round-trip: ~400-650ms**

### Throughput
- Recommended rate: **2 Hz** (safe for most networks)
- Maximum rate: **3-4 Hz** (good network, fast VLM)
- Minimum rate: **0.5 Hz** (slow network or complex scenes)

### Bandwidth
- Image size: ~50-100 KB (JPEG, 640x480)
- At 2 Hz: ~1-2 Mbps bandwidth
- Result size: <1 KB (text)

## Troubleshooting

### Bridge Cannot Connect to Server

```bash
# On TurtleBot3, test connection:
curl http://192.168.1.100:5000/health

# Should return: {"status":"healthy",...}
```

If fails:
- Check server is running
- Check IP address is correct
- Check firewall allows port 5000
- Verify both machines on same network

### Server Errors

```bash
# Check server logs
# Look for error messages in terminal where server is running

# Test with curl:
curl -X POST http://localhost:5000/inference \
  -H "Content-Type: application/json" \
  -d '{"image":"<base64>","prompt":"test"}'
```

### Slow Inference

- Reduce `inference_rate` in bridge config
- Use 4-bit quantization on server
- Check network speed: `ping <server-ip>`

### Memory Issues on Server

```bash
# Use 4-bit quantization
python3 standalone_vlm_server.py --quantization 4bit

# Or use CPU (slower but less memory)
python3 standalone_vlm_server.py --device cpu
```

## Advantages of This Architecture

✅ **No ROS2 on Inference Machine**
- Easier to setup on Windows/Mac
- Can use any Python environment
- No ROS2 installation headaches

✅ **Network Flexibility**
- VLM server can be anywhere on network
- Can use cloud servers
- Can switch between multiple TurtleBots

✅ **Resource Distribution**
- Heavy inference on powerful desktop
- Lightweight bridge on Jetson
- Better resource utilization

✅ **Easy Scaling**
- One VLM server can serve multiple robots
- Add load balancer for multiple servers
- Deploy different models easily

## Comparison with Previous Architectures

| Architecture | ROS2 on Jetson | ROS2 on Inference Machine | Network |
|-------------|----------------|---------------------------|---------|
| **Original vlm_bridge** | ✅ Yes | ✅ Yes (required) | ROS2 DDS |
| **Network Bridge** (new) | ✅ Yes | ❌ No | HTTP |
| **vlm_reasoner** (existing) | ✅ Yes | ❌ N/A (runs on Jetson) | None |

Choose based on your needs:
- **Network Bridge**: Remote VLM, no ROS2 on inference machine
- **vlm_bridge**: Both machines have ROS2, direct DDS communication  
- **vlm_reasoner**: All-in-one solution, everything on Jetson

## Security Considerations

⚠️ **Important:** This setup uses unencrypted HTTP. For production:

1. **Use HTTPS** with SSL certificates
2. **Add authentication** (API keys, tokens)
3. **Firewall rules** to restrict access
4. **VPN** for remote access

Example with authentication:
```python
# In standalone_vlm_server.py, add:
API_KEY = "your-secret-key"

@app.before_request
def check_auth():
    if request.headers.get('X-API-Key') != API_KEY:
        return jsonify({'error': 'Unauthorized'}), 401
```

## Next Steps

1. **Test the connection** between TurtleBot3 and server
2. **Tune inference_rate** for your network
3. **Customize prompts** for your application
4. **Integrate with navigation** using `/vlm/inference_result`
5. **Add decision logic** based on VLM responses
