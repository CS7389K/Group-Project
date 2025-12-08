#!/bin/bash
# Quick fix for white/gray camera images

echo "=========================================="
echo "Camera White/Gray Image Fix"
echo "=========================================="
echo ""

# Check camera detection
echo "1. Checking camera detection..."
if ls /dev/video* &> /dev/null; then
    echo "✓ Camera devices found:"
    ls -l /dev/video*
else
    echo "✗ No camera devices found!"
    echo "  Check cable connection and run: dmesg | grep -i camera"
    exit 1
fi
echo ""

# Check kernel messages
echo "2. Checking kernel messages for camera..."
CAMERA_DETECTED=false
if dmesg | grep -iq "imx219"; then
    echo "✓ RPi Camera v2 (IMX219) detected"
    CAMERA_DETECTED=true
elif dmesg | grep -iq "imx477"; then
    echo "✓ RPi Camera HQ (IMX477) detected"
    CAMERA_DETECTED=true
elif dmesg | grep -iq "camera"; then
    echo "⚠ Camera detected but model uncertain"
    CAMERA_DETECTED=true
else
    echo "✗ No camera detected in kernel messages"
fi
echo ""

# Test with V4L2
echo "3. Testing camera with V4L2..."
python3 << 'EOF'
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
if cap.isOpened():
    ret, frame = cap.read()
    if ret and frame is not None:
        mean_val = np.mean(frame)
        print(f"✓ Camera capture successful")
        print(f"  Resolution: {frame.shape}")
        print(f"  Mean pixel value: {mean_val:.1f}")
        
        if mean_val > 250:
            print("  ⚠ Image is all WHITE - likely a hardware/cable issue")
            print("    Try:")
            print("    - Reseat camera cable on both ends")
            print("    - Check CSI port (try different port if available)")
            print("    - Verify camera module is not defective")
        elif mean_val < 5:
            print("  ⚠ Image is all BLACK - camera covered or wrong settings")
        else:
            print("  ✓ Image data looks good!")
            print("")
            print("Camera is working! Use V4L2 camera publisher:")
            print("  ros2 run turtlebot3_vlm_perception camera_publisher_v4l2")
            
        # Save test image
        cv2.imwrite('/tmp/camera_test.jpg', frame)
        print(f"  Test image saved: /tmp/camera_test.jpg")
    else:
        print("✗ Failed to capture frame")
    cap.release()
else:
    print("✗ Cannot open camera with V4L2")
EOF

echo ""
echo "=========================================="
echo "Recommended Actions:"
echo "=========================================="
echo ""
echo "If image is WHITE/GRAY:"
echo "1. Physical check:"
echo "   - Reseat camera cable on BOTH ends"
echo "   - Make sure ribbon cable is inserted correctly (contacts facing inward)"
echo "   - Try different CSI port if you have multiple"
echo ""
echo "2. Use V4L2 camera publisher (more reliable):"
echo "   ros2 run turtlebot3_vlm_perception camera_publisher_v4l2"
echo ""
echo "3. If still not working, check camera with:"
echo "   nvgstcapture-1.0  # Nvidia camera capture tool"
echo ""
