#!/usr/bin/env python3
"""
Camera Diagnostic Tool
======================
Run this to test different camera configurations and find what works.
"""

import cv2
import sys

print("=" * 60)
print("CAMERA DIAGNOSTIC TOOL")
print("=" * 60)
print()

# Test 1: Simple device access
print("Test 1: Simple OpenCV VideoCapture")
print("-" * 60)
for device_id in [0, 1]:
    print(f"Trying /dev/video{device_id}...")
    cap = cv2.VideoCapture(device_id)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            import numpy as np
            print(f"  ✅ SUCCESS on device {device_id}")
            print(f"     Shape: {frame.shape}")
            print(f"     Mean: {np.mean(frame):.1f}")
            print(f"     Min: {np.min(frame)}, Max: {np.max(frame)}")
            if np.mean(frame) > 250:
                print(f"  ⚠️  WARNING: Image appears all white!")
            elif np.mean(frame) < 5:
                print(f"  ⚠️  WARNING: Image appears all black!")
            else:
                print(f"  ✓ Image looks good!")
                
            # Save test image
            cv2.imwrite(f'/tmp/test_device{device_id}.jpg', frame)
            print(f"     Saved to: /tmp/test_device{device_id}.jpg")
        else:
            print(f"  ❌ Failed to read frame")
        cap.release()
    else:
        print(f"  ❌ Cannot open device {device_id}")
print()

# Test 2: GStreamer with nvarguscamerasrc
print("Test 2: GStreamer nvarguscamerasrc")
print("-" * 60)
gst_pipelines = [
    # Default pipeline
    "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink",
    
    # Alternative with different resolution
    "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink",
    
    # Simplified pipeline
    "nvarguscamerasrc ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink",
]

for i, pipeline in enumerate(gst_pipelines):
    print(f"Pipeline {i+1}:")
    print(f"  {pipeline[:80]}...")
    try:
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                import numpy as np
                print(f"  ✅ SUCCESS")
                print(f"     Shape: {frame.shape}")
                print(f"     Mean: {np.mean(frame):.1f}")
                if np.mean(frame) > 250:
                    print(f"  ⚠️  WARNING: Image appears all white!")
                elif np.mean(frame) < 5:
                    print(f"  ⚠️  WARNING: Image appears all black!")
                else:
                    print(f"  ✓ Image looks good!")
                cv2.imwrite(f'/tmp/test_gst{i}.jpg', frame)
                print(f"     Saved to: /tmp/test_gst{i}.jpg")
            else:
                print(f"  ❌ Cannot read frame")
            cap.release()
        else:
            print(f"  ❌ Cannot open pipeline")
    except Exception as e:
        print(f"  ❌ Error: {e}")
    print()

# Test 3: Check camera settings
print("Test 3: Camera Properties")
print("-" * 60)
cap = cv2.VideoCapture(0)
if cap.isOpened():
    props = {
        'CAP_PROP_FRAME_WIDTH': cv2.CAP_PROP_FRAME_WIDTH,
        'CAP_PROP_FRAME_HEIGHT': cv2.CAP_PROP_FRAME_HEIGHT,
        'CAP_PROP_FPS': cv2.CAP_PROP_FPS,
        'CAP_PROP_BRIGHTNESS': cv2.CAP_PROP_BRIGHTNESS,
        'CAP_PROP_CONTRAST': cv2.CAP_PROP_CONTRAST,
        'CAP_PROP_SATURATION': cv2.CAP_PROP_SATURATION,
        'CAP_PROP_GAIN': cv2.CAP_PROP_GAIN,
        'CAP_PROP_EXPOSURE': cv2.CAP_PROP_EXPOSURE,
    }
    
    for name, prop in props.items():
        value = cap.get(prop)
        print(f"  {name}: {value}")
    cap.release()
else:
    print("  ❌ Cannot open camera for property check")
print()

# Summary
print("=" * 60)
print("DIAGNOSTIC COMPLETE")
print("=" * 60)
print()
print("Next steps:")
print("1. Check saved images in /tmp/test_*.jpg")
print("2. If all images are white/gray:")
print("   - Check camera cable connection")
print("   - Try: sudo apt install v4l-utils && v4l2-ctl --list-devices")
print("   - Check: dmesg | grep -i camera")
print("3. Use the working pipeline/device in your ROS node")
print()
