#!/bin/bash
# Camera diagnostic script for Jetson Xavier NX with Raspberry Pi Camera

echo "======================================================================"
echo "JETSON CAMERA DIAGNOSTIC"
echo "======================================================================"
echo ""

# Check camera device
echo "1. Checking camera device files..."
if ls /dev/video* 1> /dev/null 2>&1; then
    ls -l /dev/video*
    echo "   ✓ Camera devices found"
else
    echo "   ❌ No /dev/video* devices found"
    echo "   Camera is NOT connected or detected"
fi
echo ""

# Check if nvargus-daemon is running
echo "2. Checking NVIDIA Argus camera daemon..."
if pgrep -x nvargus-daemon > /dev/null; then
    echo "   ✓ nvargus-daemon is running"
else
    echo "   ⚠️  nvargus-daemon is NOT running"
    echo "   Try: sudo systemctl restart nvargus-daemon"
fi
echo ""

# Check for camera processes holding resources
echo "3. Checking for processes using camera..."
camera_procs=$(lsof /dev/video0 2>/dev/null | grep -v COMMAND)
if [ -z "$camera_procs" ]; then
    echo "   ✓ No processes holding camera"
else
    echo "   ⚠️  Camera is in use:"
    lsof /dev/video0 2>/dev/null
    echo ""
    echo "   Run release_gstream.sh to free it"
fi
echo ""

# Check GStreamer plugins
echo "4. Checking GStreamer NVIDIA plugins..."
if gst-inspect-1.0 nvarguscamerasrc > /dev/null 2>&1; then
    echo "   ✓ nvarguscamerasrc plugin found"
else
    echo "   ❌ nvarguscamerasrc plugin NOT found"
    echo "   GStreamer NVIDIA plugins may not be installed"
fi
echo ""

# Try quick GStreamer test
echo "5. Testing camera with GStreamer (5 second test)..."
echo "   This will show if camera hardware is working..."
timeout 5 gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=50 ! \
    'video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1' ! \
    fakesink 2>&1 | grep -E "ERROR|Setting|Running"

if [ $? -eq 0 ] || [ $? -eq 124 ]; then
    echo "   ✓ GStreamer can access camera"
else
    echo "   ❌ GStreamer cannot access camera"
fi
echo ""

# Check v4l2
echo "6. Checking V4L2 devices..."
if command -v v4l2-ctl &> /dev/null; then
    v4l2-ctl --list-devices 2>/dev/null | head -20
else
    echo "   ⚠️  v4l2-ctl not installed"
    echo "   Install: sudo apt install v4l-utils"
fi
echo ""

echo "======================================================================"
echo "RECOMMENDED NEXT STEPS"
echo "======================================================================"
echo ""
echo "If camera devices found but tests failing:"
echo "  1. Run: ./release_gstream.sh"
echo "  2. Run: sudo systemctl restart nvargus-daemon"
echo "  3. Test: python3 direct_camera_test.py"
echo ""
echo "If camera devices NOT found:"
echo "  1. Power off Jetson"
echo "  2. Check camera ribbon cable on both ends"
echo "  3. Ensure cable inserted in correct CSI port"
echo "  4. Power on and check: ls /dev/video*"
echo ""
echo "======================================================================"
