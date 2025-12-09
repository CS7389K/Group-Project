# TurtleBot3 VLM Perception System - Installation Guide

Complete installation guide for deploying the VLM perception system on TurtleBot3 with Jetson Xavier NX.

## System Requirements

### Hardware
- TurtleBot3 (Burger or Waffle)
- NVIDIA Jetson Xavier NX (8GB RAM)
- OpenMANIPULATOR-X (optional)
- USB Camera or Raspberry Pi Camera Module

### Software
- Ubuntu 20.04 or 22.04 (JetPack 5.x)
- ROS2 Humble
- Python 3.8+
- CUDA 11.4+ (included with JetPack)

## Quick Installation (Recommended)

```bash
# Clone the repository on Jetson
cd ~/
git clone https://github.com/CS7389K/Group-Project.git moondream2_turtlebot3
cd moondream2_turtlebot3

# Run the installation script
chmod +x tools/install_jetson.sh
./tools/install_jetson.sh
```

The script will:
1. Install system dependencies
2. Install PyTorch for Jetson
3. Install Python packages
4. Setup ROS2 workspace
5. Build packages
6. Configure environment

## Manual Installation

If you prefer to install manually or the script fails:

### 1. Install ROS2 Humble

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install System Dependencies

```bash
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    v4l-utils \
    git \
    wget
```

### 3. Install PyTorch for Jetson

```bash
# Download PyTorch wheel for Jetson
wget https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl

# Install
pip3 install torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
```

For other JetPack versions, see: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

### 4. Clone Repository and Install Dependencies

```bash
cd ~/
git clone https://github.com/CS7389K/Group-Project.git moondream2_turtlebot3
cd moondream2_turtlebot3

# Install Python dependencies
pip3 install -r requirements.txt
```

### 5. Setup ROS2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Link packages
ln -s ~/moondream2_turtlebot3/ros2_bridge vlm_bridge
ln -s ~/moondream2_turtlebot3/src/perception turtlebot3_vlm_perception

# Initialize rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build packages
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. Configure Environment

```bash
# Add CUDA memory optimization
echo 'export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128,expandable_segments:True' >> ~/.bashrc

# Add HuggingFace cache directory (optional)
echo 'export HF_HOME=~/hf_cache' >> ~/.bashrc

source ~/.bashrc
```

## Package Structure

After installation, you'll have two ROS2 packages:

### 1. `vlm_bridge` - Standalone VLM Server

Location: `~/ros2_ws/src/vlm_bridge`

Provides a ROS2 interface for Moondream2 inference:
- `vlm_server` node: Runs VLM model and serves queries
- `vlm_client` node: Example client for testing

Launch:
```bash
ros2 launch vlm_bridge vlm_bridge.launch.py
```

### 2. `turtlebot3_vlm_perception` - Complete Perception Stack

Location: `~/ros2_ws/src/turtlebot3_vlm_perception`

Full perception system with YOLO + VLM reasoning:
- `camera_publisher` node: V4L2 camera interface
- `vlm_reasoner` node: YOLO detection + Moondream2 reasoning

Launch:
```bash
ros2 launch turtlebot3_vlm_perception vlm.launch.py
```

## Testing Installation

### Test 1: Check Package Installation

```bash
# List installed packages
ros2 pkg list | grep -E "vlm|turtlebot3"

# Expected output:
# turtlebot3_vlm_perception
# vlm_bridge
```

### Test 2: Camera Test

```bash
# List cameras
v4l2-ctl --list-devices

# Test camera capture
ros2 launch turtlebot3_vlm_perception camera_only.launch.py

# In another terminal, view images
ros2 run rqt_image_view rqt_image_view
```

### Test 3: VLM Bridge Test

```bash
# Start VLM server (may take 1-2 minutes to load model)
ros2 launch vlm_bridge vlm_bridge.launch.py with_client:=true

# Check if model loaded
ros2 node list  # Should show /vlm_server and /vlm_client

# Monitor responses
ros2 topic echo /vlm/response
```

### Test 4: Full System Test

```bash
# Launch complete system
ros2 launch turtlebot3_vlm_perception vlm.launch.py

# You should see:
# - Camera feed window (OpenCV)
# - YOLO detections (bounding boxes)
# - VLM reasoning dashboard in terminal
```

## Troubleshooting

### Issue: "Cannot allocate memory in static TLS block"

**Solution:** The `vlm_reasoner.py` includes a preload fix. If you still see this:

```bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
```

### Issue: Out of GPU Memory

**Solutions:**
1. Use 4-bit quantization:
   ```bash
   ros2 launch vlm_bridge vlm_bridge.launch.py quantization:=4bit
   ```

2. Lower VLM analysis rate in `config/vlm_params.yaml`:
   ```yaml
   query_rate: 1.0  # Reduce from 2.0 to 1.0 Hz
   ```

3. Close other applications and maximize CUDA memory:
   ```bash
   sudo systemctl stop nvargus-daemon
   sudo jetson_clocks
   ```

### Issue: Model Download Fails

**Solution:** Pre-download model:

```bash
python3 << EOF
from transformers import AutoModelForCausalLM, AutoTokenizer
model = AutoModelForCausalLM.from_pretrained("vikhyatk/moondream2", trust_remote_code=True)
tokenizer = AutoTokenizer.from_pretrained("vikhyatk/moondream2")
EOF
```

### Issue: Camera Not Detected

**Solutions:**
1. Check camera device:
   ```bash
   ls -la /dev/video*
   v4l2-ctl --list-devices
   ```

2. Update device path in launch file:
   ```bash
   ros2 launch turtlebot3_vlm_perception camera_only.launch.py device:=/dev/video1
   ```

3. Fix permissions:
   ```bash
   sudo usermod -aG video $USER
   # Log out and back in
   ```

### Issue: "transformers" Type Error

**Solution:** The system uses transformers <4.46.0 for Python 3.8 compatibility. If you see type errors, ensure correct version:

```bash
pip3 install 'transformers>=4.30.0,<4.46.0'
```

## Performance Tuning

### Jetson Power Mode

Set maximum performance:
```bash
sudo nvpmodel -m 0  # Max power mode
sudo jetson_clocks   # Max clocks
```

### Memory Monitoring

```bash
# Watch GPU memory
watch -n 1 'tegrastats'

# Watch system memory
watch -n 1 'free -h'
```

### Optimize for Your Use Case

Edit `ros2_bridge/config/vlm_params.yaml`:

```yaml
/**:
  ros__parameters:
    # Balance quality vs speed
    quantization: "8bit"  # Use "4bit" for lower memory
    query_rate: 2.0       # Lower to 1.0 for more headroom
    
    # Detection threshold
    detection_threshold: 0.5  # Increase to reduce false positives
```

## Updating the System

```bash
cd ~/moondream2_turtlebot3
git pull

# Rebuild packages
cd ~/ros2_ws
colcon build --symlink-install --packages-select vlm_bridge turtlebot3_vlm_perception
```

## Uninstallation

```bash
# Remove packages from workspace
rm ~/ros2_ws/src/vlm_bridge
rm ~/ros2_ws/src/turtlebot3_vlm_perception

# Remove workspace (optional)
rm -rf ~/ros2_ws

# Remove repository
rm -rf ~/moondream2_turtlebot3
```

## Additional Resources

- **Package Documentation:**
  - VLM Bridge: `ros2_bridge/README.md`
  - Perception Stack: See package README

- **ROS2 Documentation:**
  - ROS2 Humble: https://docs.ros.org/en/humble/
  - TurtleBot3: https://emanual.robotis.com/docs/en/platform/turtlebot3/

- **Jetson Resources:**
  - JetPack: https://developer.nvidia.com/embedded/jetpack
  - PyTorch for Jetson: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

## Support

For issues and questions:
- GitHub Issues: https://github.com/CS7389K/Group-Project/issues
- Email: vsj23@txstate.edu
