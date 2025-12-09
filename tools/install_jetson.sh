#!/bin/bash
# =============================================================================
# Jetson Setup Script for TurtleBot3 VLM Perception System
# =============================================================================
# This script installs all dependencies and sets up the ROS2 workspace
# for the Moondream2 VLM perception system on Jetson Xavier NX.
#
# Usage:
#   ./install_jetson.sh
#
# Requirements:
#   - Jetson Xavier NX with JetPack 5.x
#   - Ubuntu 20.04 or 22.04
#   - ROS2 Humble installed
# =============================================================================

set -e  # Exit on error

echo "========================================================================"
echo "TurtleBot3 VLM Perception - Jetson Installation"
echo "========================================================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo -e "${YELLOW}WARNING: This does not appear to be a Jetson device${NC}"
    echo "This script is optimized for Jetson Xavier NX"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check for ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ERROR: ROS2 not sourced${NC}"
    echo "Please install and source ROS2 Humble first:"
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 $ROS_DISTRO detected${NC}"
echo ""

# =============================================================================
# Step 1: Install System Dependencies
# =============================================================================
echo "Step 1: Installing system dependencies..."
echo "------------------------------------------------------------------------"

sudo apt update
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-image-transport \
    v4l-utils \
    git \
    wget \
    curl

echo -e "${GREEN}✓ System dependencies installed${NC}"
echo ""

# =============================================================================
# Step 2: Install PyTorch for Jetson
# =============================================================================
echo "Step 2: Installing PyTorch for Jetson..."
echo "------------------------------------------------------------------------"

# Check if PyTorch is already installed
if python3 -c "import torch" 2>/dev/null; then
    echo -e "${GREEN}✓ PyTorch already installed${NC}"
    python3 -c "import torch; print(f'  Version: {torch.__version__}')"
else
    echo "Installing PyTorch for Jetson Xavier NX..."
    
    # PyTorch 2.0 for JetPack 5.x
    TORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"
    
    wget -O /tmp/torch-2.0.0-cp38-linux_aarch64.whl "$TORCH_URL" || {
        echo -e "${YELLOW}WARNING: Could not download PyTorch wheel${NC}"
        echo "Please install PyTorch manually from:"
        echo "  https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
    }
    
    if [ -f /tmp/torch-2.0.0-cp38-linux_aarch64.whl ]; then
        pip3 install /tmp/torch-2.0.0-cp38-linux_aarch64.whl
        echo -e "${GREEN}✓ PyTorch installed${NC}"
    fi
fi
echo ""

# =============================================================================
# Step 3: Install Python Dependencies
# =============================================================================
echo "Step 3: Installing Python dependencies..."
echo "------------------------------------------------------------------------"

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

if [ -f "$PROJECT_ROOT/requirements.txt" ]; then
    pip3 install -r "$PROJECT_ROOT/requirements.txt"
    echo -e "${GREEN}✓ Python dependencies installed${NC}"
else
    echo -e "${YELLOW}WARNING: requirements.txt not found${NC}"
fi
echo ""

# =============================================================================
# Step 4: Setup ROS2 Workspace
# =============================================================================
echo "Step 4: Setting up ROS2 workspace..."
echo "------------------------------------------------------------------------"

# Create workspace if it doesn't exist
ROS_WS="$HOME/ros2_ws"
if [ ! -d "$ROS_WS" ]; then
    mkdir -p "$ROS_WS/src"
    echo -e "${GREEN}✓ Created workspace at $ROS_WS${NC}"
else
    echo -e "${GREEN}✓ Workspace already exists at $ROS_WS${NC}"
fi

# Link packages
echo "Linking packages to workspace..."

# Link ros2_bridge (vlm_bridge)
if [ -d "$PROJECT_ROOT/ros2_bridge" ]; then
    ln -sf "$PROJECT_ROOT/ros2_bridge" "$ROS_WS/src/vlm_bridge"
    echo -e "${GREEN}✓ Linked vlm_bridge package${NC}"
fi

# Link perception package
if [ -d "$PROJECT_ROOT/src/perception" ]; then
    ln -sf "$PROJECT_ROOT/src/perception" "$ROS_WS/src/turtlebot3_vlm_perception"
    echo -e "${GREEN}✓ Linked turtlebot3_vlm_perception package${NC}"
fi

echo ""

# =============================================================================
# Step 5: Initialize rosdep
# =============================================================================
echo "Step 5: Initializing rosdep..."
echo "------------------------------------------------------------------------"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Install dependencies
cd "$ROS_WS"
rosdep install --from-paths src --ignore-src -r -y || {
    echo -e "${YELLOW}WARNING: Some rosdep dependencies could not be installed${NC}"
}

echo -e "${GREEN}✓ rosdep initialized${NC}"
echo ""

# =============================================================================
# Step 6: Build ROS2 Packages
# =============================================================================
echo "Step 6: Building ROS2 packages..."
echo "------------------------------------------------------------------------"

cd "$ROS_WS"
colcon build --symlink-install --packages-select vlm_bridge turtlebot3_vlm_perception

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Packages built successfully${NC}"
else
    echo -e "${RED}ERROR: Build failed${NC}"
    exit 1
fi
echo ""

# =============================================================================
# Step 7: Setup Environment
# =============================================================================
echo "Step 7: Setting up environment..."
echo "------------------------------------------------------------------------"

# Add to .bashrc if not already there
if ! grep -q "source $ROS_WS/install/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# TurtleBot3 VLM Perception Workspace" >> ~/.bashrc
    echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc
    echo -e "${GREEN}✓ Added workspace to .bashrc${NC}"
else
    echo -e "${GREEN}✓ Workspace already in .bashrc${NC}"
fi

# Set CUDA environment variables
if ! grep -q "PYTORCH_CUDA_ALLOC_CONF" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# CUDA Memory Optimization for VLM" >> ~/.bashrc
    echo "export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128,expandable_segments:True" >> ~/.bashrc
    echo -e "${GREEN}✓ Added CUDA optimization to .bashrc${NC}"
fi

echo ""

# =============================================================================
# Step 8: Download Models (Optional)
# =============================================================================
echo "Step 8: Pre-downloading models (optional)..."
echo "------------------------------------------------------------------------"

read -p "Download Moondream2 model now? (recommended, ~2GB) (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    python3 << EOF
print("Downloading Moondream2 model...")
from transformers import AutoModelForCausalLM, AutoTokenizer
import os

# Set cache directory
cache_dir = os.path.expanduser("~/hf_cache")
os.makedirs(cache_dir, exist_ok=True)

try:
    print("Downloading model files...")
    model = AutoModelForCausalLM.from_pretrained(
        "vikhyatk/moondream2",
        trust_remote_code=True,
        cache_dir=cache_dir
    )
    tokenizer = AutoTokenizer.from_pretrained(
        "vikhyatk/moondream2",
        cache_dir=cache_dir
    )
    print("✓ Model downloaded successfully")
except Exception as e:
    print(f"WARNING: Could not download model: {e}")
    print("Model will be downloaded on first run")
EOF
fi

echo ""

# =============================================================================
# Installation Complete
# =============================================================================
echo "========================================================================"
echo -e "${GREEN}Installation Complete!${NC}"
echo "========================================================================"
echo ""
echo "Packages installed:"
echo "  - vlm_bridge (ROS2 bridge for VLM inference)"
echo "  - turtlebot3_vlm_perception (Camera + VLM reasoning)"
echo ""
echo "To activate the workspace in this terminal:"
echo "  source $ROS_WS/install/setup.bash"
echo ""
echo "To test the installation:"
echo "  ros2 launch vlm_bridge vlm_bridge.launch.py"
echo ""
echo "For more information, see:"
echo "  $PROJECT_ROOT/ros2_bridge/README.md"
echo "  $PROJECT_ROOT/README.md"
echo ""
echo "========================================================================"
