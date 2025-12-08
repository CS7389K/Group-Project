#!/bin/bash
# Quick setup script for TurtleBot3 VLM Perception on Jetson Xavier NX

set -e

echo "=========================================="
echo "TurtleBot3 VLM Perception Setup"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo -e "${YELLOW}Warning: Not running on Jetson. Some features may not work.${NC}"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check ROS2 installation
if [ ! -d /opt/ros/foxy ]; then
    echo -e "${RED}Error: ROS2 Foxy not found. Please install ROS2 first.${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 Foxy found${NC}"

# Source ROS2
source /opt/ros/foxy/setup.bash

# Install system dependencies
echo ""
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y \
    ros-foxy-cv-bridge \
    ros-foxy-sensor-msgs \
    ros-foxy-std-msgs \
    python3-opencv \
    python3-pip \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    libgstreamer1.0-dev

echo -e "${GREEN}✓ System dependencies installed${NC}"

# Create workspace if needed
if [ ! -d ~/ros2_ws/src ]; then
    echo ""
    echo "Creating ROS2 workspace..."
    mkdir -p ~/ros2_ws/src
fi

# Copy package
echo ""
echo "Copying package to workspace..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cp -r "$SCRIPT_DIR" ~/ros2_ws/src/turtlebot3_vlm_perception

echo -e "${GREEN}✓ Package copied${NC}"

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
echo -e "${YELLOW}Note: This will take several minutes...${NC}"

cd ~/ros2_ws/src/turtlebot3_vlm_perception

# Check if PyTorch is installed
if ! python3 -c "import torch" 2>/dev/null; then
    echo "Installing PyTorch for Jetson..."
    echo -e "${YELLOW}This may take 10-15 minutes...${NC}"
    
    # Download PyTorch wheel for Jetson (adjust version as needed)
    if [ ! -f torch-1.10.0-cp38-cp38-linux_aarch64.whl ]; then
        wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl \
            -O torch-1.10.0-cp38-cp38-linux_aarch64.whl
    fi
    
    pip3 install torch-1.10.0-cp38-cp38-linux_aarch64.whl
else
    echo -e "${GREEN}✓ PyTorch already installed${NC}"
fi

# Install other dependencies
pip3 install --upgrade pip
pip3 install transformers==4.30.0 accelerate==0.20.3 pillow psutil

# Install YOLO (may need to compile)
echo "Installing Ultralytics YOLO11..."
pip3 install ultralytics

# Try to install bitsandbytes (may fail on Jetson)
echo "Installing bitsandbytes (8-bit quantization)..."
pip3 install bitsandbytes || echo -e "${YELLOW}Warning: bitsandbytes install failed. VLM may use more memory.${NC}"

echo -e "${GREEN}✓ Python dependencies installed${NC}"

# Build package
echo ""
echo "Building ROS2 package..."
cd ~/ros2_ws
colcon build --packages-select turtlebot3_vlm_perception

echo -e "${GREEN}✓ Package built${NC}"

# Setup environment
echo ""
echo "Setting up environment..."
if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    echo -e "${GREEN}✓ Added workspace to .bashrc${NC}"
fi

# Test camera
echo ""
echo "Testing camera..."
if ls /dev/video* >/dev/null 2>&1; then
    echo -e "${GREEN}✓ Camera device found: $(ls /dev/video*)${NC}"
else
    echo -e "${YELLOW}⚠ Warning: No camera device found. Check connection.${NC}"
fi

# Final instructions
echo ""
echo "=========================================="
echo -e "${GREEN}✓ Setup Complete!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Enable performance mode (recommended):"
echo "   sudo nvpmodel -m 0"
echo "   sudo jetson_clocks"
echo ""
echo "2. Source workspace:"
echo "   source ~/ros2_ws/install/setup.bash"
echo ""
echo "3. Test camera:"
echo "   ros2 launch turtlebot3_vlm_perception camera_only.launch.py"
echo ""
echo "4. Run full system:"
echo "   ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py"
echo ""
echo "For troubleshooting, see: ~/ros2_ws/src/turtlebot3_vlm_perception/README.md"
echo ""
