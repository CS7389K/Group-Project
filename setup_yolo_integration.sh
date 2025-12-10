#!/bin/bash
# YOLO11 + VLM Integration Setup Script
# Run on Jetson Xavier NX

set -e

echo "=========================================="
echo "YOLO11 + VLM Integration Setup"
echo "=========================================="

# Check if on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo "⚠️  Warning: This doesn't appear to be a Jetson device"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Install YOLO11
echo ""
echo "📦 Installing YOLO11..."
pip3 install ultralytics

# Verify installation
echo ""
echo "✓ Verifying YOLO11..."
python3 -c "from ultralytics import YOLO; model = YOLO('yolo11n.pt'); print('✓ YOLO11 ready')" || {
    echo "❌ YOLO11 installation failed"
    exit 1
}

# Build ROS2 package
echo ""
echo "🔨 Building ROS2 package..."
cd ~/moondream2_turtlebot3
colcon build --packages-select vlm_bridge

# Source the workspace
echo ""
echo "📝 Sourcing workspace..."
source install/setup.bash

echo ""
echo "=========================================="
echo "✅ Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Make sure VLM server is running on desktop"
echo "2. Launch the integrated system:"
echo ""
echo "   ros2 launch vlm_bridge yolo_vlm_complete.launch.py \\"
echo "       vlm_server_url:=http://YOUR_DESKTOP_IP:5000 \\"
echo "       camera_device:=/dev/video1"
echo ""
echo "See YOLO_VLM_INTEGRATION.md for complete documentation"
echo ""
