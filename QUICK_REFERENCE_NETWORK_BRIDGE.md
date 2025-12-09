# Quick Reference: Network Bridge Commands

## On Remote Computer (NO ROS2)

```bash
# Install
pip3 install -r standalone_requirements.txt

# Start VLM Server
python3 standalone_vlm_server.py --host 0.0.0.0 --port 5000 --device cuda

# Test Server
curl http://localhost:5000/health
python3 tools/test_network_bridge.py --url http://localhost:5000
```

## On TurtleBot3 (WITH ROS2)

```bash
# Install (one time)
git clone https://github.com/CS7389K/Group-Project.git ~/moondream2_turtlebot3
cd ~/moondream2_turtlebot3
./tools/install_jetson.sh

# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch camera
ros2 launch turtlebot3_vlm_perception camera_only.launch.py

# Launch bridge (replace IP with your server)
ros2 launch vlm_bridge network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000

# Monitor results
ros2 topic echo /vlm/inference_result
ros2 topic hz /vlm/inference_result
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Subscribe | Camera feed input |
| `/vlm/inference_result` | std_msgs/String | Publish | VLM results output |

## Configuration

**On TurtleBot3:** Edit `~/ros2_ws/src/vlm_bridge/config/network_bridge_params.yaml`

```yaml
vlm_server_url: "http://YOUR_SERVER_IP:5000"
inference_rate: 2.0  # Hz
timeout: 10.0        # seconds
```

## Troubleshooting

```bash
# Test network connection
ping YOUR_SERVER_IP
curl http://YOUR_SERVER_IP:5000/health

# Check topics
ros2 topic list
ros2 topic info /camera/image_raw
ros2 topic info /vlm/inference_result

# View logs
ros2 node info ros2_network_bridge

# Check firewall (on server)
sudo ufw allow 5000
```

## Performance Tuning

**Slower network:**
```bash
ros2 launch vlm_bridge network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    inference_rate:=1.0 \
    timeout:=15.0
```

**Faster network:**
```bash
ros2 launch vlm_bridge network_bridge.launch.py \
    vlm_server_url:=http://192.168.1.100:5000 \
    inference_rate:=3.0 \
    timeout:=5.0
```

## System Check

```bash
# On remote computer
python3 standalone_vlm_server.py &
curl http://localhost:5000/health
# Should return: {"status":"healthy","model_loaded":true,...}

# On TurtleBot3
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep vlm_bridge
# Should show: vlm_bridge

ros2 topic list | grep camera
# Should show: /camera/image_raw

ros2 run vlm_bridge ros2_network_bridge \
    --ros-args -p vlm_server_url:=http://YOUR_IP:5000
# Should connect without errors
```

## File Locations

**On Remote Computer:**
- VLM Server: `./standalone_vlm_server.py`
- Requirements: `./standalone_requirements.txt`

**On TurtleBot3:**
- Bridge Node: `~/ros2_ws/src/vlm_bridge/vlm_bridge/ros2_network_bridge.py`
- Launch File: `~/ros2_ws/src/vlm_bridge/launch/network_bridge.launch.py`
- Config: `~/ros2_ws/src/vlm_bridge/config/network_bridge_params.yaml`
