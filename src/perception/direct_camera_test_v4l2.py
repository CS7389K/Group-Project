#!/usr/bin/env python3
"""
Direct camera test using V4L2 (fallback method)
Use this if GStreamer version fails
"""

import cv2
import numpy as np
import sys

print("=" * 70)
print("DIRECT CAMERA TEST - V4L2 MODE")
print("=" * 70)
print()

# Test with different video device numbers
devices_to_try = [0, 1, 2]

cap = None
working_device = None

for device_id in devices_to_try:
    print(f"Trying /dev/video{device_id}...")
    cap = cv2.VideoCapture(device_id)
    
    if cap.isOpened():
        # Try to read a frame to verify it actually works
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"✓ Successfully opened /dev/video{device_id}")
            working_device = device_id
            break
        else:
            print(f"  Device opens but can't read frames")
            cap.release()
            cap = None
    else:
        print(f"  Cannot open /dev/video{device_id}")

if cap is None or not cap.isOpened():
    print("\n❌ FAILED: No working camera found")
    print("\nTroubleshooting:")
    print("1. Check available devices: ls -l /dev/video*")
    print("2. Check v4l2 info: v4l2-ctl --list-devices")
    print("3. If using RPi Camera on Jetson, use GStreamer version instead:")
    print("   python3 direct_camera_test.py")
    sys.exit(1)

print(f"\nUsing /dev/video{working_device}")

# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Test capture
print("\nCapturing test frames...")
success_count = 0
for i in range(10):
    ret, frame = cap.read()
    if ret and frame is not None:
        success_count += 1

print(f"✓ Captured {success_count}/10 frames")

if success_count == 0:
    print("❌ FAILED: Cannot read frames")
    cap.release()
    sys.exit(1)

# Analyze image
print("\nAnalyzing image content...")
ret, frame = cap.read()
if ret and frame is not None:
    print(f"  Shape: {frame.shape}")
    print(f"  Mean pixel value: {np.mean(frame):.1f}")
    print(f"  Min: {np.min(frame)}, Max: {np.max(frame)}")
    
    mean_val = np.mean(frame)
    if mean_val > 250:
        print("\n⚠️  WARNING: Image is mostly WHITE")
        print("   Possible causes:")
        print("   - Camera not properly connected")
        print("   - Need to use GStreamer instead of V4L2")
    elif mean_val < 5:
        print("\n⚠️  WARNING: Image is mostly BLACK")
    else:
        print("\n✓ Image data looks GOOD!")
    
    cv2.imwrite('/tmp/direct_camera_test_v4l2.jpg', frame)
    print("\n✓ Test image saved: /tmp/direct_camera_test_v4l2.jpg")

# Live view
print("\nOpening live view window...")
print("Press 'q' to quit, 's' to save frame")
print("-" * 70)

frame_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame")
        break
    
    frame_count += 1
    mean_val = np.mean(frame)
    
    cv2.putText(frame, f"V4L2 - Frame: {frame_count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(frame, f"Device: /dev/video{working_device}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"Mean: {mean_val:.1f}", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    if mean_val > 250:
        cv2.putText(frame, "WARNING: Mostly WHITE!", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    elif mean_val < 5:
        cv2.putText(frame, "WARNING: Mostly BLACK!", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    cv2.imshow('V4L2 Camera Test', frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        filename = f'/tmp/v4l2_frame_{frame_count}.jpg'
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")

cap.release()
cv2.destroyAllWindows()

print("\n" + "=" * 70)
print("V4L2 TEST COMPLETE")
print("=" * 70)
print(f"Total frames: {frame_count}")
print("=" * 70)
