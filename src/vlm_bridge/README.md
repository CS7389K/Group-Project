# VLM Bridge - ROS2 Package

ROS2 bridge for Moondream2 Vision-Language Model inference on TurtleBot3.

## Overview

This package provides a ROS2 interface for running Moondream2 VLM inference on a TurtleBot3 with Jetson Xavier NX. It includes:

- **VLM Server Node**: Runs Moondream2 model and provides inference service
- **VLM Client Node**: Example client for testing queries
- **Launch Files**: Easy startup with configurable parameters

## Installation on Jetson

### 1. Clone Repository

```bash
cd ~/
git clone https://github.com/CS7389K/Group-Project.git moondream2_turtlebot3
cd moondream2_turtlebot3
```

### 2. Install System Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs

# Initialize rosdep if not already done
sudo rosdep init
rosdep update
```

### 3. Install Python Dependencies

```bash
# Install PyTorch for Jetson (pre-built wheel)
wget https://nvidia.box.com/shared/static/...<jetson-specific-wheel>.whl
pip3 install torch-*.whl

# Install other dependencies
pip3 install -r requirements.txt
```

### 4. Build ROS2 Workspace

```bash
# Create workspace structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Link the package
ln -s ~/moondream2_turtlebot3/ros2_bridge vlm_bridge

# Build
cd ~/ros2_ws
colcon build --packages-select vlm_bridge

# Source the workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage

### Launch VLM Server

```bash
# Basic launch (8-bit quantization, default settings)
ros2 launch vlm_bridge vlm_bridge.launch.py

# With 4-bit quantization (lower memory)
ros2 launch vlm_bridge vlm_bridge.launch.py quantization:=4bit

# With test client
ros2 launch vlm_bridge vlm_bridge.launch.py with_client:=true

# Custom prompt
ros2 launch vlm_bridge vlm_bridge.launch.py with_client:=true prompt:="What objects do you see?"
```

### Run Nodes Individually

```bash
# Start VLM server
ros2 run vlm_bridge vlm_server

# Start test client (in another terminal)
ros2 run vlm_bridge vlm_client --ros-args -p prompt:="Describe this scene"
```

## Topics

- **Subscribed:**
  - `/camera/image_raw` (sensor_msgs/Image) - Camera feed input

- **Published:**
  - `/vlm/response` (std_msgs/String) - VLM inference results

## Parameters

### VLM Server

- `model_id` (string, default: "vikhyatk/moondream2"): HuggingFace model ID
- `quantization` (string, default: "8bit"): Quantization mode (8bit/4bit/none)
- `device` (string, default: "cuda"): Compute device (cuda/cpu)

### VLM Client

- `prompt` (string): Query prompt for VLM
- `query_rate` (float, default: 0.5): Query frequency in Hz

## Configuration

Edit `config/vlm_params.yaml` to change default parameters:

```yaml
/**:
  ros__parameters:
    model_id: "vikhyatk/moondream2"
    quantization: "8bit"
    device: "cuda"
    query_rate: 2.0
```

## Memory Optimization

For Jetson Xavier NX (8GB RAM):

1. **Use 8-bit quantization** (default) - ~4GB GPU memory
2. **Use 4-bit quantization** - ~2.5GB GPU memory
3. **Close other applications** before running
4. **Set CUDA memory config**:
   ```bash
   export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128,expandable_segments:True
   ```

## Integration with TurtleBot3 Perception

To use with the existing perception stack:

```bash
# Terminal 1: Launch camera
ros2 launch turtlebot3_vlm_perception camera_only.launch.py

# Terminal 2: Launch VLM bridge
ros2 launch vlm_bridge vlm_bridge.launch.py

# Terminal 3: View responses
ros2 topic echo /vlm/response
```

## Troubleshooting

### "cannot allocate memory in static TLS block"

Add library preload to your script or environment:

```python
import ctypes
ctypes.CDLL('/path/to/libgomp.so', mode=ctypes.RTLD_GLOBAL)
```

### Out of Memory on GPU

- Reduce quantization: `quantization:=4bit`
- Lower query rate: `query_rate:=1.0`
- Close other GPU applications

### Model Download Fails

Set HuggingFace cache:

```bash
export HF_HOME=~/hf_cache
export TRANSFORMERS_CACHE=~/hf_cache
```

## License

MIT License - See LICENSE file for details
