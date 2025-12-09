# System Architecture

## Package Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    TurtleBot3 VLM System                         │
│                    Jetson Xavier NX                              │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                         ROS2 Workspace                           │
│                      ~/ros2_ws/                                  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │
                ┌─────────────┴─────────────┐
                │                           │
                ▼                           ▼
┌───────────────────────────┐  ┌──────────────────────────────┐
│   Package 1: vlm_bridge   │  │ Package 2: turtlebot3_vlm_   │
│                           │  │          perception          │
│  Standalone VLM Server    │  │  Complete Perception Stack   │
└───────────────────────────┘  └──────────────────────────────┘
                │                            │
                │                            │
                ▼                            ▼
┌───────────────────────────┐  ┌──────────────────────────────┐
│ Nodes:                    │  │ Nodes:                       │
│  • vlm_server             │  │  • camera_publisher          │
│  • vlm_client (test)      │  │  • vlm_reasoner              │
│                           │  │                              │
│ Topics:                   │  │ Topics:                      │
│  • /camera/image_raw (sub)│  │  • /camera/image_raw (pub)   │
│  • /vlm/response (pub)    │  │                              │
│                           │  │ Features:                    │
│ Features:                 │  │  • V4L2 Camera               │
│  • Moondream2 VLM         │  │  • YOLO11 Detection          │
│  • 8-bit/4-bit Quant      │  │  • Moondream2 VLM            │
│  • Service Architecture   │  │  • Decision Fusion           │
│  • Configurable           │  │  • Dashboard Display         │
└───────────────────────────┘  └──────────────────────────────┘
```

## Data Flow

### Option 1: VLM Bridge Only

```
┌─────────────┐    /camera/image_raw     ┌─────────────┐
│   Camera    │ ────────────────────────> │ VLM Server  │
│  (External) │    sensor_msgs/Image      │             │
└─────────────┘                           │ Moondream2  │
                                          │ Inference   │
                                          └─────────────┘
                                                 │
                                                 │ /vlm/response
                                                 │ std_msgs/String
                                                 ▼
                                          ┌─────────────┐
                                          │   Client    │
                                          │  (Optional) │
                                          └─────────────┘
```

### Option 2: Complete Perception Stack

```
┌─────────────┐    V4L2         ┌─────────────────┐
│ USB Camera  │ ──────────────> │ Camera Publisher│
│  /dev/video │                 │                 │
└─────────────┘                 └─────────────────┘
                                        │
                                        │ /camera/image_raw
                                        │ sensor_msgs/Image
                                        ▼
                                ┌─────────────────┐
                                │  VLM Reasoner   │
                                │                 │
                                │  ┌───────────┐  │
                                │  │  YOLO11   │  │ ──> Fast Detection
                                │  │ Detection │  │     (30+ FPS)
                                │  └───────────┘  │
                                │        │        │
                                │        ▼        │
                                │  ┌───────────┐  │
                                │  │Moondream2 │  │ ──> Deep Reasoning
                                │  │    VLM    │  │     (2-3 Hz)
                                │  └───────────┘  │
                                │        │        │
                                │        ▼        │
                                │  ┌───────────┐  │
                                │  │ Decision  │  │ ──> Action Planning
                                │  │  Fusion   │  │     GRASP/PUSH/AVOID
                                │  └───────────┘  │
                                └─────────────────┘
                                        │
                                        ▼
                                ┌─────────────────┐
                                │    Dashboard    │
                                │  (Terminal UI)  │
                                └─────────────────┘
```

## Component Architecture

### VLM Server Node

```
┌───────────────────────────────────────────────────────────┐
│                    VLM Server Node                         │
├───────────────────────────────────────────────────────────┤
│                                                            │
│  ┌──────────────────────────────────────────────────┐    │
│  │           Image Subscription                      │    │
│  │  /camera/image_raw (sensor_msgs/Image)           │    │
│  └──────────────────────┬───────────────────────────┘    │
│                         │                                 │
│                         ▼                                 │
│  ┌──────────────────────────────────────────────────┐    │
│  │           CV Bridge Conversion                    │    │
│  │  ROS Image -> OpenCV -> PIL Image                │    │
│  └──────────────────────┬───────────────────────────┘    │
│                         │                                 │
│                         ▼                                 │
│  ┌──────────────────────────────────────────────────┐    │
│  │        Moondream2 Model Inference                 │    │
│  │  - AutoModelForCausalLM                          │    │
│  │  - 8-bit/4-bit Quantization                      │    │
│  │  - CUDA Optimization                             │    │
│  │  - FP32 Vision Encoder Patch                     │    │
│  └──────────────────────┬───────────────────────────┘    │
│                         │                                 │
│                         ▼                                 │
│  ┌──────────────────────────────────────────────────┐    │
│  │          Response Publication                     │    │
│  │  /vlm/response (std_msgs/String)                 │    │
│  └──────────────────────────────────────────────────┘    │
│                                                            │
└───────────────────────────────────────────────────────────┘
```

### VLM Reasoner Node

```
┌───────────────────────────────────────────────────────────┐
│                  VLM Reasoner Node                         │
├───────────────────────────────────────────────────────────┤
│                                                            │
│  ┌──────────────────────────────────────────────────┐    │
│  │           Image Subscription                      │    │
│  │  /camera/image_raw (sensor_msgs/Image)           │    │
│  └──────────────────────┬───────────────────────────┘    │
│                         │                                 │
│           ┌─────────────┴─────────────┐                   │
│           │                           │                   │
│           ▼                           ▼                   │
│  ┌──────────────────┐      ┌──────────────────┐          │
│  │  YOLO11 Object   │      │   Frame Buffer   │          │
│  │    Detection     │      │   (Throttling)   │          │
│  │   (30+ FPS)      │      └──────────────────┘          │
│  └────────┬─────────┘                                     │
│           │                                               │
│           │ Detections                                    │
│           ▼                                               │
│  ┌──────────────────────────────────────────────────┐    │
│  │     Detection-Guided Prompting                    │    │
│  │  - Consolidated VLM queries                      │    │
│  │  - Material & weight estimation                  │    │
│  │  - Fragility & size analysis                     │    │
│  │  - Reachability assessment                       │    │
│  └──────────────────────┬───────────────────────────┘    │
│                         │                                 │
│                         ▼                                 │
│  ┌──────────────────────────────────────────────────┐    │
│  │       Moondream2 VLM Inference                    │    │
│  │  - Physics-aware reasoning                       │    │
│  │  - 2-3 Hz processing rate                        │    │
│  └──────────────────────┬───────────────────────────┘    │
│                         │                                 │
│                         ▼                                 │
│  ┌──────────────────────────────────────────────────┐    │
│  │          Decision Fusion Logic                    │    │
│  │  - Grasp feasibility check                       │    │
│  │  - Weight vs payload check                       │    │
│  │  - Fragility risk assessment                     │    │
│  │  - Action selection: GRASP/PUSH/AVOID/STOP       │    │
│  └──────────────────────┬───────────────────────────┘    │
│                         │                                 │
│                         ▼                                 │
│  ┌──────────────────────────────────────────────────┐    │
│  │           Dashboard Display                       │    │
│  │  - Detection visualization (OpenCV)              │    │
│  │  - Reasoning results (Terminal)                  │    │
│  │  - System stats (RAM/GPU)                        │    │
│  └──────────────────────────────────────────────────┘    │
│                                                            │
└───────────────────────────────────────────────────────────┘
```

## Deployment Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   Development Machine                        │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │  GitHub Repository                                  │    │
│  │  CS7389K/Group-Project                             │    │
│  │                                                     │    │
│  │  ├── ros2_bridge/          (Package 1)            │    │
│  │  ├── src/perception/       (Package 2)            │    │
│  │  ├── tools/                                        │    │
│  │  │   └── install_jetson.sh                        │    │
│  │  ├── INSTALL.md                                    │    │
│  │  ├── QUICKSTART.md                                 │    │
│  │  └── requirements.txt                              │    │
│  └────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                         │
                         │ git clone
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                  Jetson Xavier NX                            │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │  ~/moondream2_turtlebot3/                          │    │
│  │  (Cloned repository)                               │    │
│  └────────────────────────────────────────────────────┘    │
│                         │                                   │
│                         │ ./tools/install_jetson.sh         │
│                         │                                   │
│                         ▼                                   │
│  ┌────────────────────────────────────────────────────┐    │
│  │  ~/ros2_ws/                                        │    │
│  │  (ROS2 Workspace)                                  │    │
│  │                                                     │    │
│  │  src/                                              │    │
│  │  ├── vlm_bridge/        -> (symlink)              │    │
│  │  └── turtlebot3_vlm_perception/ -> (symlink)      │    │
│  │                                                     │    │
│  │  build/                                            │    │
│  │  install/                                          │    │
│  └────────────────────────────────────────────────────┘    │
│                         │                                   │
│                         │ source install/setup.bash         │
│                         │                                   │
│                         ▼                                   │
│  ┌────────────────────────────────────────────────────┐    │
│  │  Running System                                    │    │
│  │                                                     │    │
│  │  ros2 launch vlm_bridge vlm_bridge.launch.py     │    │
│  │     OR                                             │    │
│  │  ros2 launch turtlebot3_vlm_perception vlm.launch.py│  │
│  └────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

## Memory Layout (Jetson Xavier NX)

```
┌─────────────────────────────────────────────────────┐
│          Jetson Xavier NX (8GB RAM)                  │
├─────────────────────────────────────────────────────┤
│                                                      │
│  System RAM (8GB)                                   │
│  ┌────────────────────────────────────────────┐    │
│  │ OS + ROS2          ~1.5 GB                 │    │
│  ├────────────────────────────────────────────┤    │
│  │ Camera/YOLO        ~1.0 GB                 │    │
│  ├────────────────────────────────────────────┤    │
│  │ Available          ~5.5 GB                 │    │
│  └────────────────────────────────────────────┘    │
│                                                      │
│  GPU Memory (Shared from System RAM)                │
│  ┌────────────────────────────────────────────┐    │
│  │ 8-bit VLM          ~4.0 GB                 │    │
│  │ 4-bit VLM          ~2.5 GB  (alternative)  │    │
│  ├────────────────────────────────────────────┤    │
│  │ CUDA Runtime       ~0.5 GB                 │    │
│  ├────────────────────────────────────────────┤    │
│  │ Available          ~1.5 GB                 │    │
│  └────────────────────────────────────────────┘    │
│                                                      │
└─────────────────────────────────────────────────────┘

Recommendation: Use 8-bit quantization for best balance
Alternative: Use 4-bit if running multiple processes
```

## Performance Profile

```
Component              | Frequency | Latency    | Memory
-----------------------|-----------|------------|--------
Camera Capture         | 30 FPS    | ~33 ms     | ~50 MB
YOLO11 Detection       | 30 FPS    | ~30 ms     | ~1 GB
VLM Inference          | 2-3 Hz    | ~400 ms    | ~4 GB
Decision Fusion        | 2-3 Hz    | ~1 ms      | ~10 MB
Dashboard Update       | 30 FPS    | ~10 ms     | ~20 MB
-----------------------|-----------|------------|--------
Total System           | 30 FPS    | ~500 ms    | ~5.1 GB
                       |  (camera) | (VLM)      | (peak)
```
