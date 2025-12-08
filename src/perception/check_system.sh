#!/bin/bash
# Quick test script to verify TurtleBot3 VLM Perception installation

echo "=========================================="
echo "TurtleBot3 VLM Perception - System Check"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check counter
CHECKS_PASSED=0
TOTAL_CHECKS=0

check_command() {
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} $2"
        CHECKS_PASSED=$((CHECKS_PASSED + 1))
        return 0
    else
        echo -e "${RED}✗${NC} $2"
        return 1
    fi
}

check_file() {
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} $2"
        CHECKS_PASSED=$((CHECKS_PASSED + 1))
        return 0
    else
        echo -e "${RED}✗${NC} $2"
        return 1
    fi
}

check_python_module() {
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    if python3 -c "import $1" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} Python module: $1"
        CHECKS_PASSED=$((CHECKS_PASSED + 1))
        return 0
    else
        echo -e "${RED}✗${NC} Python module: $1"
        return 1
    fi
}

echo "System Checks:"
echo "──────────────"

# Check if Jetson
if [ -f /etc/nv_tegra_release ]; then
    JETSON_VERSION=$(cat /etc/nv_tegra_release | grep REVISION | cut -d' ' -f2)
    echo -e "${GREEN}✓${NC} Running on Jetson (Release: $JETSON_VERSION)"
    CHECKS_PASSED=$((CHECKS_PASSED + 1))
else
    echo -e "${YELLOW}⚠${NC} Not running on Jetson (some features may not work)"
fi
TOTAL_CHECKS=$((TOTAL_CHECKS + 1))

# Check camera
if ls /dev/video* &> /dev/null; then
    echo -e "${GREEN}✓${NC} Camera device found: $(ls /dev/video*)"
    CHECKS_PASSED=$((CHECKS_PASSED + 1))
else
    echo -e "${YELLOW}⚠${NC} No camera device found"
fi
TOTAL_CHECKS=$((TOTAL_CHECKS + 1))

echo ""
echo "ROS2 Checks:"
echo "────────────"

check_file "/opt/ros/foxy/setup.bash" "ROS2 Foxy installed"
check_command "ros2" "ros2 command available"

# Source ROS2 if available
if [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
fi

# Check if package is built
TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
if [ -f ~/ros2_ws/install/turtlebot3_vlm_perception/share/turtlebot3_vlm_perception/package.xml ]; then
    echo -e "${GREEN}✓${NC} Package built in workspace"
    CHECKS_PASSED=$((CHECKS_PASSED + 1))
else
    echo -e "${RED}✗${NC} Package not built (run: cd ~/ros2_ws && colcon build)"
fi

echo ""
echo "Python Dependencies:"
echo "────────────────────"

check_python_module "torch"
check_python_module "transformers"
check_python_module "cv2"
check_python_module "PIL"
check_python_module "ultralytics"
check_python_module "psutil"

echo ""
echo "GStreamer Checks:"
echo "─────────────────"

check_command "gst-launch-1.0" "GStreamer installed"
check_command "gst-inspect-1.0" "GStreamer tools available"

TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
if gst-inspect-1.0 nvarguscamerasrc &> /dev/null; then
    echo -e "${GREEN}✓${NC} nvarguscamerasrc plugin available"
    CHECKS_PASSED=$((CHECKS_PASSED + 1))
else
    echo -e "${YELLOW}⚠${NC} nvarguscamerasrc not found (camera may not work)"
fi

echo ""
echo "=========================================="
echo "Results: $CHECKS_PASSED/$TOTAL_CHECKS checks passed"
echo "=========================================="
echo ""

if [ $CHECKS_PASSED -eq $TOTAL_CHECKS ]; then
    echo -e "${GREEN}✓ All checks passed! System is ready.${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Enable performance mode:"
    echo "     sudo nvpmodel -m 0 && sudo jetson_clocks"
    echo ""
    echo "  2. Launch system:"
    echo "     ros2 launch turtlebot3_vlm_perception vlm_perception.launch.py"
    exit 0
elif [ $CHECKS_PASSED -ge $((TOTAL_CHECKS * 3 / 4)) ]; then
    echo -e "${YELLOW}⚠ Most checks passed. Review warnings above.${NC}"
    echo ""
    echo "You may be able to run with limited functionality."
    exit 0
else
    echo -e "${RED}✗ Too many checks failed. Please run install script.${NC}"
    echo ""
    echo "Run: cd src/perception && ./install.sh"
    exit 1
fi
