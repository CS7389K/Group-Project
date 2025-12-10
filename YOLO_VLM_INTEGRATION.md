# YOLO11 + VLM Integration Guide

## Overview

This system integrates YOLO11 object detection with Moondream2 VLM for enhanced robot manipulation decision-making.

### Architecture

```
┌─────────────┐     ┌──────────────┐     ┌─────────────────┐     ┌──────────────┐
│   Camera    │────>│ YOLO11 Node  │────>│ YOLO+VLM Bridge │────>│  VLM Server  │
│  Publisher  │     │  (Jetson)    │     │    (Jetson)     │     │   (Remote)   │
└─────────────┘     └──────────────┘     └─────────────────┘     └──────────────┘
      │                    │                       │                      │
      │                    │                       │                      │
      v                    v                       v                      v
 /camera/image_raw   /yolo/detections    /vlm/analysis_result    Enhanced
                                                                   Reasoning
```

### Key Features

1. **Object Detection**: YOLO11 identifies objects with bounding boxes
2. **Dimension Estimation**: Converts pixel bbox to real-world mm estimates
3. **Spatial Analysis**: Position (left/center/right), relative size, multi-object detection
4. **Cross-Validation**: YOLO class ↔ VLM material/type consistency checking
5. **Enhanced VLM Prompts**: Uses YOLO data to guide VLM queries
6. **Multi-Object Awareness**: Handles scenes with multiple objects
7. **Jetson TLS Fix**: cv2 imported before torch to avoid memory issues

---

## Installation

### 1. Install YOLO11 (Jetson)

```bash
# On Jetson Xavier NX
pip3 install ultralytics

# Test YOLO11
python3 -c "from ultralytics import YOLO; model = YOLO('yolo11n.pt'); print('✓ YOLO11 ready')"
```

**Note**: The first run will download the model (~6MB for yolo11n.pt)

### 2. Build ROS2 Package

```bash
cd ~/moondream2_turtlebot3
colcon build --packages-select vlm_bridge
source install/setup.bash
```

---

## Usage

### Quick Start

**On Jetson (TurtleBot3):**
```bash
ros2 launch vlm_bridge yolo_vlm_complete.launch.py \
    vlm_server_url:=http://10.43.174.30:5000 \
    camera_device:=/dev/video1 \
    yolo_model:=n \
    inference_rate:=2.0
```

**On Desktop (VLM Server):**
```bash
cd /workspace/scripts  # or wherever vlm_ros_server.py is located
python3 vlm_ros_server.py
```

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vlm_server_url` | *required* | VLM server URL (e.g., http://10.43.174.30:5000) |
| `camera_device` | `/dev/video1` | USB camera device path |
| `yolo_model` | `n` | Model size: n(fastest)/s/m/l/x(best accuracy) |
| `yolo_confidence` | `0.5` | Detection confidence threshold (0.0-1.0) |
| `inference_rate` | `2.0` | VLM inference frequency (Hz) |
| `show_preview` | `true` | Show visualization windows |
| `yolo_required` | `false` | Wait for YOLO before inference (set true for YOLO-only mode) |

### YOLO Model Sizes

| Model | Speed | Accuracy | Size | Use Case |
|-------|-------|----------|------|----------|
| `n` | Fastest | Good | 6 MB | Real-time on Jetson (recommended) |
| `s` | Fast | Better | 22 MB | Balanced performance |
| `m` | Medium | Very Good | 50 MB | Higher accuracy needed |
| `l` | Slow | Excellent | 88 MB | Offline processing |
| `x` | Slowest | Best | 136 MB | Maximum accuracy |

---

## System Output

### Example Terminal Output (Jetson)

```
[YOLO11 Detector]
Frame 100: 2 objects: ['bottle', 'cup']

[YOLO+VLM Bridge]
[Inference #5]
YOLO: bottle @ 0.87 (~85mm)
VLM: GRASP - light plastic bottle (50g, 85mm) - clear path
Total time: 845ms
✓ Cross-validation: YOLO 'bottle' ↔ VLM 'plastic' MATCH
```

### Example Terminal Output (Desktop VLM Server)

```
[VLM Analysis #5] YOLO-Enhanced
  YOLO: bottle @ 0.87, ~85mm
  Q1 (Material & Cross-check): yes, this is a plastic bottle, lightweight, smooth glossy surface
  Q2 (Fragility & Size Check): not fragile, plastic is durable, 85mm seems accurate, cylindrical shape
  Q3 (Spatial Context): yes within reach, not blocking path, cup visible to the right
  → Decision: GRASP
  → Reasoning: GRASP: light plastic bottle (50g, 85mm) - navigate around nearby objects
  → Cross-validation: OK
  → Time: 825ms
```

---

## Integration Details

### 1. YOLO Detection Data Format

Published on `/yolo/detections`:

```json
{
  "timestamp": 1702387845.123,
  "frame_id": 1234,
  "detections": [
    {
      "class": "bottle",
      "confidence": 0.87,
      "bbox": {
        "x1": 120, "y1": 80, "x2": 200, "y2": 280,
        "width_px": 80, "height_px": 200,
        "center_x": 160, "center_y": 180,
        "normalized_x": 0.25, "normalized_y": 0.375
      },
      "estimated_size_mm": {
        "width": 67,
        "height": 167,
        "max_dimension": 167
      },
      "spatial": {
        "position": "left",
        "vertical": "middle",
        "relative_size": 0.104
      }
    }
  ]
}
```

### 2. VLM Request Format

Sent to VLM server `/analyze` endpoint:

```json
{
  "image": "<base64 JPEG>",
  "timestamp": 1702387845.123,
  "frame_id": 1234,
  "yolo_enabled": true,
  "primary_object": {
    "class": "bottle",
    "confidence": 0.87,
    "bbox": {...},
    "estimated_size_mm": {...},
    "spatial": {...}
  },
  "yolo_detections": [...]
}
```

### 3. VLM Response Format

```json
{
  "material": "plastic",
  "weight_category": "light",
  "weight_grams": 50,
  "fragility": "not_fragile",
  "size_mm": 85,
  "reachable": true,
  "path_blocking": false,
  "graspable": true,
  "action": "GRASP",
  "reasoning": "GRASP: light plastic bottle (50g, 85mm) - clear path",
  "confidence": 0.89,
  "analysis_time_ms": 825,
  "yolo_guided": true,
  "yolo_confidence": 0.87
}
```

---

## Cross-Validation Logic

The system cross-validates YOLO object class with VLM's material/type analysis:

### Material Mapping

| YOLO Class | Expected Materials |
|------------|-------------------|
| bottle | plastic, glass |
| cup | plastic, glass, ceramic, paper |
| book | paper, wood |
| cell phone | plastic, metal, glass |
| laptop | metal, plastic |
| bowl | ceramic, glass, plastic, metal |
| fork/knife/spoon | metal |

### Validation Examples

✅ **MATCH**: YOLO detects "bottle" → VLM says "plastic" → Consistent

⚠️ **MISMATCH**: YOLO detects "cup" → VLM says "metal" → Unexpected (metal cup unusual)

❌ **CONFLICT**: YOLO @ 0.60 detects "bottle" → VLM says "not a bottle, it's a can" → Low confidence, defer to VLM

---

## Decision Logic

### Priority Order

1. **Safety**: Fragile objects → AVOID
2. **Conflict Resolution**: YOLO/VLM disagree + low YOLO confidence → STOP
3. **Multi-object Caution**: Multiple objects + blocking → STOP
4. **Optimal Grasp**: Weight OK + size OK + not fragile → GRASP
5. **Alternative**: Too large/heavy but manageable → PUSH
6. **Uncertain Fragility**: → AVOID
7. **Out of Range**: → IGNORE
8. **Fallback**: Unclear conditions → STOP

### Actions

- **GRASP**: Pick up the object
- **PUSH**: Move it by pushing
- **AVOID**: Do not touch (fragile/dangerous)
- **IGNORE**: No action needed (not blocking, out of reach)
- **STOP**: Requires human decision

---

## Troubleshooting

### YOLO Issues

**Problem**: `RuntimeError: Unable to allocate TLS thread-local storage`

**Solution**: Already fixed! The `yolo11_detector.py` imports cv2 **before** torch:
```python
# JETSON TLS FIX: Import cv2 BEFORE torch/ultralytics
import cv2
import numpy as np

# Now safe to import torch-based libraries
import rclpy
from ultralytics import YOLO
```

**Problem**: YOLO detections not appearing

**Check**:
```bash
# Verify YOLO node is running
ros2 node list | grep yolo

# Check YOLO topic
ros2 topic echo /yolo/detections --once

# Check logs
ros2 node info /yolo11_detector
```

### VLM Issues

**Problem**: VLM server returns 404 on `/analyze`

**Solution**: The server might be running an old version. Restart:
```bash
pkill -f vlm_ros_server
cd /workspace/scripts
python3 vlm_ros_server.py
```

**Problem**: Cross-validation always shows mismatch

**Solution**: VLM prompts might be too generic. Check `vlm_ros_server.py` has `analyze_with_yolo()` method.

### Performance Issues

**YOLO too slow on Jetson**:
- Use `yolo_model:=n` (smallest/fastest)
- Lower camera resolution: `camera_width:=320 camera_height:=240`
- Reduce YOLO confidence: `yolo_confidence:=0.6`

**VLM inference too slow**:
- Lower `inference_rate:=1.0` (1 Hz)
- Use quantized model on server (8-bit or 4-bit)

---

## Advanced Configuration

### Run Without YOLO (VLM Only)

```bash
ros2 launch vlm_bridge yolo_vlm_complete.launch.py \
    vlm_server_url:=http://10.43.174.30:5000 \
    yolo_required:=false
```

The system will work with VLM-only analysis if YOLO isn't running.

### Run Individual Nodes

**Just YOLO**:
```bash
ros2 run vlm_bridge yolo11_detector --ros-args \
    -p model_size:=n \
    -p confidence_threshold:=0.5 \
    -p show_preview:=true
```

**Just VLM Bridge**:
```bash
ros2 run vlm_bridge yolo_vlm_bridge --ros-args \
    -p vlm_server_url:=http://10.43.174.30:5000 \
    -p inference_rate:=2.0 \
    -p yolo_required:=false
```

---

## Testing

### Verify YOLO Detection

```bash
# Terminal 1: Start camera
ros2 run vlm_bridge camera_publisher

# Terminal 2: Start YOLO
ros2 run vlm_bridge yolo11_detector

# Terminal 3: Check detections
ros2 topic echo /yolo/detections
```

### Verify Cross-Validation

Place a **bottle** in view:
- YOLO should detect: `"class": "bottle"`
- VLM should report: `"material": "plastic"` or `"glass"`
- Cross-validation: `✓ MATCH`

Place a **metal cup** in view:
- YOLO: `"class": "cup"`
- VLM: `"material": "metal"`
- Cross-validation: `⚠ MISMATCH` (cups are usually not metal)

---

## Performance Metrics

### Typical Timing (Jetson Xavier NX + Desktop GTX 1080 Ti)

- Camera capture: ~33ms (30 FPS)
- YOLO11n inference: ~45ms on Jetson
- Network transmission: ~10ms (local network)
- VLM inference: ~800ms (3 queries × ~270ms each)
- **Total**: ~890ms per analysis

### Optimization Tips

1. **Use YOLO11n**: 2x faster than YOLO11s, only 5% accuracy drop
2. **Lower resolution**: 320x240 vs 640x480 = 4x fewer pixels
3. **Increase inference_rate delay**: 2 Hz means process every 500ms
4. **Reduce YOLO confidence**: 0.5 vs 0.7 = more detections but less filtering

---

## Next Steps

1. **Test with different objects**: bottles, cups, books, phones
2. **Verify cross-validation**: Check YOLO ↔ VLM consistency
3. **Tune confidence thresholds**: Balance false positives vs missed detections
4. **Monitor performance**: Check FPS and inference times
5. **Add more material mappings**: Extend cross-validation for more YOLO classes

---

## File Locations

- **YOLO Detector**: `ros2_bridge/vlm_bridge/yolo11_detector.py`
- **Integrated Bridge**: `ros2_bridge/vlm_bridge/yolo_vlm_bridge.py`
- **VLM Server**: `scripts/vlm_ros_server.py`
- **Launch File**: `ros2_bridge/launch/yolo_vlm_complete.launch.py`
- **Camera Publisher**: `ros2_bridge/vlm_bridge/camera_publisher.py`

---

## Support

For issues:
1. Check this guide's Troubleshooting section
2. Verify all nodes are running: `ros2 node list`
3. Check topics: `ros2 topic list`
4. Review logs for errors

Good luck! 🚀
