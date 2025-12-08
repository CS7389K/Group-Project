#!/usr/bin/env python3
"""
Direct camera test - bypasses ROS to check if camera hardware works
Run this FIRST to verify camera before testing ROS nodes
"""

import cv2
import numpy as np
import sys

print("=" * 70)
print("DIRECT CAMERA TEST (No ROS) - JETSON OPTIMIZED")
print("=" * 70)
print()

# Test 1: GStreamer camera access (proper for Jetson + RPi Camera)
print("Test 1: Opening camera with GStreamer (nvarguscamerasrc)...")
gst_pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=1"
)

print(f"Pipeline: {gst_pipeline}")
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("❌ FAILED: Cannot open camera with GStreamer")
    print("\nTroubleshooting:")
    print("1. Check camera connection: ls /dev/video*")
    print("2. Try manual GStreamer test:")
    print("   gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink")
    print("3. Release camera resources:")
    print("   sudo systemctl restart nvargus-daemon")
    print("4. If still failing, camera may not be properly connected")
    sys.exit(1)

print("✓ Camera opened with GStreamer")

# Test 2: Capture frames
print("\nTest 2: Capturing frames...")
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

# Test 3: Analyze image
print("\nTest 3: Analyzing image content...")
ret, frame = cap.read()
if ret and frame is not None:
    print(f"  Shape: {frame.shape}")
    print(f"  Mean pixel value: {np.mean(frame):.1f}")
    print(f"  Min: {np.min(frame)}, Max: {np.max(frame)}")
    print(f"  Dtype: {frame.dtype}")
    
    mean_val = np.mean(frame)
    if mean_val > 250:
        print("\n⚠️  WARNING: Image is mostly WHITE")
        print("   Possible causes:")
        print("   - Camera cable not seated properly")
        print("   - Wrong CSI port")
        print("   - Camera module defective")
        print("   - Need different GStreamer pipeline")
    elif mean_val < 5:
        print("\n⚠️  WARNING: Image is mostly BLACK")
        print("   Possible causes:")
        print("   - Camera lens covered")
        print("   - Wrong exposure settings")
    else:
        print("\n✓ Image data looks GOOD!")
    
    # Save test image
    cv2.imwrite('/tmp/direct_camera_test.jpg', frame)
    print(f"\n✓ Test image saved: /tmp/direct_camera_test.jpg")
    print("  View with: eog /tmp/direct_camera_test.jpg")

# Test 4: Live view
print("\nTest 4: Opening live view window...")
print("Press 'q' to quit, 's' to save frame")
print("-" * 70)

frame_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame")
        break
    
    frame_count += 1
    
    # Add overlay
    mean_val = np.mean(frame)
    cv2.putText(frame, f"Frame: {frame_count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, f"Mean: {mean_val:.1f}", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    if mean_val > 250:
        cv2.putText(frame, "WARNING: Mostly WHITE!", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    elif mean_val < 5:
        cv2.putText(frame, "WARNING: Mostly BLACK!", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    cv2.imshow('Direct Camera Test', frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("\nUser pressed 'q' - exiting")
        break
    elif key == ord('s'):
        filename = f'/tmp/camera_frame_{frame_count}.jpg'
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")

cap.release()
cv2.destroyAllWindows()

print("\n" + "=" * 70)
print("TEST COMPLETE")
print("=" * 70)
print("\nSummary:")
print(f"  Total frames: {frame_count}")
print(f"  Test images saved to /tmp/")
print("\nIf you saw proper images, the camera hardware works!")
print("If images were white/gray, try:")
print("  1. Reseat camera cable on both ends")
print("  2. Check correct CSI port")
print("  3. Try: gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink")
print("=" * 70)
