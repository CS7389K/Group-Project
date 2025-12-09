#!/usr/bin/env python3
"""
VLM Reasoning Node for TurtleBot3 + Moondream2
===============================================
Subscribes to camera feed and performs vision-language reasoning.

Input: /camera/image_raw (sensor_msgs/Image)
Output: Terminal dashboard showing detection, reasoning, and action decisions

Features:
- YOLO11 object detection (fast, 30+ FPS capable)
- Moondream2 VLM reasoning (physics-aware, 2-3 Hz)
- Detection-guided prompting strategy
- Decision fusion for manipulation actions
- Jetson Xavier NX optimized (8-bit quantization)
"""

# CRITICAL: Enable Python 3.8 compatibility for type annotations
from __future__ import annotations

# CRITICAL: Preload libraries to fix "cannot allocate memory in static TLS block" on Jetson
import ctypes
import os

def preload_libraries():
    """Preload problematic libraries for Jetson TLS fix"""
    lib_paths = [
        '/home/nvidia/.local/lib/python3.8/site-packages/torch.libs/libgomp-804f19d4.so.1.0.0',
        '/home/nvidia/.local/lib/python3.8/site-packages/tensorflow_cpu_aws.libs/libgomp-cc9055c7.so.1.0.0',
    ]
    
    for lib_path in lib_paths:
        if os.path.exists(lib_path):
            try:
                ctypes.CDLL(lib_path, mode=ctypes.RTLD_GLOBAL)
                print(f"[Preload] SUCCESS: {lib_path}")
            except Exception as e:
                print(f"[Preload] FAILED: {lib_path}: {e}")

# CRITICAL: Run preload before any other imports
preload_libraries()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import gc
import time
import psutil
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple
from enum import Enum
from PIL import Image as PILImage

# Memory optimization for Jetson
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128,expandable_segments:True'


def cleanup_memory():
    """Force garbage collection and CUDA cache cleanup"""
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


# ============================================================================
# Data Structures (from complete_hybrid_system.py)
# ============================================================================

class ManipulationAction(Enum):
    """Robot manipulation actions"""
    GRASP = "GRASP"
    PUSH = "PUSH"
    AVOID = "AVOID"
    IGNORE = "IGNORE"
    STOP = "STOP"
    SCANNING = "SCANNING"


@dataclass
class RobotSpec:
    """TurtleBot3 + OpenMANIPULATOR specifications"""
    reach_mm: int = 400
    payload_g: int = 500
    gripper_min_mm: int = 10
    gripper_max_mm: int = 100
    footprint_mm: int = 300


@dataclass
class Detection:
    """Object detection result"""
    class_name: str
    confidence: float
    bbox: List[int]  # [x1, y1, x2, y2]
    timestamp: float


@dataclass
class PhysicalProperties:
    """VLM-estimated physical properties"""
    material: str
    weight_category: str  # light/medium/heavy
    weight_grams: int
    fragility: str  # fragile/not_fragile
    graspable: bool
    size_mm: int


@dataclass
class ReasoningResult:
    """Complete reasoning output"""
    action: ManipulationAction
    confidence: float
    reasoning: str
    detection_confidence: float
    analysis_time_ms: float
    properties: Optional[PhysicalProperties] = None


# ============================================================================
# Moondream2 VLM Component
# ============================================================================

class Moondream2VLM:
    """
    Moondream2 Vision-Language Model for Jetson Xavier NX.
    Implements physics-aware reasoning with detection-guided prompting.
    """
    
    def __init__(self, robot_spec: RobotSpec, logger):
        self.robot = robot_spec
        self.logger = logger
        self.model = None
        self.tokenizer = None
    
    def load(self):
        """Load Moondream2 with 8-bit quantization for Jetson"""
        from transformers import AutoModelForCausalLM, AutoTokenizer
        
        self.logger.info('Loading Moondream2 VLM (8-bit optimized)...')
        cleanup_memory()
        
        model_id = "vikhyatk/moondream2"
        
        # 8-bit quantization for Jetson Xavier NX
        # Python 3.8 compatible: Pass parameters directly without BitsAndBytesConfig
        self.logger.info('Applying 8-bit quantization for Jetson...')
        
        self.logger.info('Loading model weights...')
        self.model = AutoModelForCausalLM.from_pretrained(
            model_id,
            trust_remote_code=True,
            load_in_8bit=True,
            device_map="auto",
            low_cpu_mem_usage=True,
            torch_dtype=torch.float16,
            llm_int8_enable_fp32_cpu_offload=True,
            # llm_int8_skip_modules=["vision_encoder", "vision", "input_layernorm"]
        )
        
        self.logger.info('Loading tokenizer...')
        self.tokenizer = AutoTokenizer.from_pretrained(model_id)
        
        # Jetson stability patch: Keep vision encoder in FP32
        if hasattr(self.model, 'vision_encoder'):
            self.logger.info('Applying Jetson FP32 patch to vision encoder...')
            self.model.vision_encoder.to(dtype=torch.float32)
        elif hasattr(self.model, 'vision'):
            self.logger.info('Applying Jetson FP32 patch to vision module...')
            self.model.vision.to(dtype=torch.float32)
        
        self.logger.info('Moondream2 loaded successfully')
    
    def query(self, image: PILImage.Image, prompt: str) -> str:
        """Single VLM query"""
        cleanup_memory()
        result = self.model.query(image, prompt)
        return result['answer'].strip()
    
    def analyze_object(self, image: PILImage.Image, detection: Detection) -> Tuple[PhysicalProperties, float]:
        """
        Analyze physical properties using detection-guided prompting.
        Optimized: 3 consolidated queries instead of 5+ separate queries.
        """
        start_time = time.time()
        obj_name = detection.class_name
        
        # Query 1: Material and weight (consolidated)
        q1 = (f"Describe this {obj_name}: What material (plastic/metal/glass/wood/paper)? "
              f"Weight estimate (light <100g, medium 100-500g, heavy >500g)? "
              f"Answer format: 'Material: X, Weight: Y'")
        answer1 = self.query(image, q1).lower()
        
        # Parse material
        material = 'unknown'
        for mat in ['plastic', 'metal', 'glass', 'wood', 'paper', 'ceramic']:
            if mat in answer1:
                material = mat
                break
        
        # Parse weight
        weight_cat = 'medium'
        weight_g = 250
        if 'light' in answer1:
            weight_cat = 'light'
            weight_g = 50
        elif 'heavy' in answer1:
            weight_cat = 'heavy'
            weight_g = 700
        
        # Query 2: Fragility and size
        q2 = f"Is this {material} {obj_name} fragile/breakable? Width in mm (20/50/80/150)? Format: 'Fragile: yes/no, Size: X mm'"
        answer2 = self.query(image, q2).lower()
        
        fragility = 'fragile' if 'yes' in answer2 or 'fragile' in answer2 else 'not_fragile'
        
        # Parse size
        try:
            size_mm = int(''.join(filter(str.isdigit, answer2.split('size')[-1])))
            size_mm = max(10, min(200, size_mm))
        except:
            bbox_width = detection.bbox[2] - detection.bbox[0]
            size_mm = int(bbox_width * 0.5)
        
        # Query 3: Reachability and action recommendation
        q3 = (f"Robot specs: {self.robot.payload_g}g payload, {self.robot.gripper_min_mm}-{self.robot.gripper_max_mm}mm gripper. "
              f"This {obj_name}: Reachable? Can grasp/push/avoid? Format: 'Reachable: yes/no, Action: X'")
        answer3 = self.query(image, q3).lower()
        
        reachable = 'yes' in answer3 or 'close' in answer3 or 'near' in answer3
        
        # Determine graspability
        graspable = (
            weight_g <= self.robot.payload_g and
            self.robot.gripper_min_mm <= size_mm <= self.robot.gripper_max_mm and
            fragility == 'not_fragile' and
            reachable
        )
        
        analysis_time_ms = (time.time() - start_time) * 1000
        
        properties = PhysicalProperties(
            material=material,
            weight_category=weight_cat,
            weight_grams=weight_g,
            fragility=fragility,
            graspable=graspable,
            size_mm=size_mm
        )
        
        return properties, analysis_time_ms


# ============================================================================
# Decision Fusion Logic
# ============================================================================

class DecisionFusion:
    """Decision logic for manipulation actions"""
    
    def __init__(self, robot_spec: RobotSpec):
        self.robot = robot_spec
    
    def decide(self, detection: Detection, properties: PhysicalProperties, 
               analysis_time_ms: float) -> ReasoningResult:
        """Make manipulation decision based on physical properties"""
        
        # Decision logic from proposal
        if properties.fragility == 'fragile':
            action = ManipulationAction.AVOID
            reason = f"WARNING: {properties.fragility.upper()} - risk of breaking"
            confidence = 0.90
            
        elif properties.graspable:
            action = ManipulationAction.GRASP
            reason = (f"Graspable: {properties.material}, "
                     f"{properties.weight_category} ({properties.weight_grams}g), "
                     f"{properties.size_mm}mm")
            confidence = 0.85
            
        elif properties.weight_grams <= self.robot.payload_g * 2 and properties.fragility == 'not_fragile':
            action = ManipulationAction.PUSH
            reason = "Can push (too large for gripper but movable)"
            confidence = 0.75
            
        elif properties.weight_grams > self.robot.payload_g * 2:
            action = ManipulationAction.AVOID
            reason = f"Too heavy ({properties.weight_grams}g > {self.robot.payload_g * 2}g threshold)"
            confidence = 0.80
            
        else:
            action = ManipulationAction.STOP
            reason = "Uncertain - need more information"
            confidence = 0.50
        
        return ReasoningResult(
            action=action,
            confidence=confidence * detection.confidence,
            reasoning=reason,
            detection_confidence=detection.confidence,
            analysis_time_ms=analysis_time_ms,
            properties=properties
        )


# ============================================================================
# Main ROS2 Node
# ============================================================================

class VLMReasonerNode(Node):
    """
    Vision-Language Model Reasoning Node for TurtleBot3.
    Integrates YOLO detection + Moondream2 VLM reasoning.
    """
    
    def __init__(self):
        super().__init__('vlm_reasoner')
        
        # Declare parameters
        self.declare_parameter('analysis_rate', 2.0)  # Hz for VLM analysis
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('use_yolo', True)
        self.declare_parameter('robot_payload_g', 500)
        self.declare_parameter('robot_gripper_min_mm', 10)
        self.declare_parameter('robot_gripper_max_mm', 100)
        
        # Get parameters
        self.analysis_rate = self.get_parameter('analysis_rate').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.use_yolo = self.get_parameter('use_yolo').value
        
        # Robot specs
        robot_spec = RobotSpec(
            payload_g=self.get_parameter('robot_payload_g').value,
            gripper_min_mm=self.get_parameter('robot_gripper_min_mm').value,
            gripper_max_mm=self.get_parameter('robot_gripper_max_mm').value
        )
        
        self.get_logger().info('Initializing VLM Reasoning Node...')
        self.get_logger().info(f'  Analysis rate: {self.analysis_rate} Hz')
        self.get_logger().info(f'  Detection threshold: {self.detection_threshold}')
        self.get_logger().info(f'  Robot payload: {robot_spec.payload_g}g')
        self.get_logger().info(f'  Gripper range: {robot_spec.gripper_min_mm}-{robot_spec.gripper_max_mm}mm')
        
        # Initialize components
        self.bridge = CvBridge()
        self.vlm = Moondream2VLM(robot_spec, self.get_logger())
        self.fusion = DecisionFusion(robot_spec)
        
        # Load models
        self.vlm.load()
        
        # YOLO detector (optional)
        self.yolo = None
        if self.use_yolo:
            try:
                from ultralytics import YOLO
                self.yolo = YOLO('yolo11n.pt')
                self.get_logger().info('YOLO11n loaded successfully')
            except Exception as e:
                self.get_logger().warn(f'YOLO not available: {e}')
                self.use_yolo = False
        
        # State
        self.last_analysis_time = 0.0
        self.analysis_interval = 1.0 / self.analysis_rate
        self.current_detection = None
        self.current_reasoning = None
        self.frame_count = 0
        
        # QoS Profile: Best Effort for real-time
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        self.get_logger().info('Subscribed to /camera/image_raw')
        self.get_logger().info('=' * 70)
        self.get_logger().info('VLM REASONING NODE READY')
        self.get_logger().info('=' * 70)
    
    def image_callback(self, msg: Image):
        """Process incoming camera frames"""
        self.frame_count += 1

        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

            # Create a copy for visualization
            display_image = cv_image.copy()

            # 1. Object Detection (YOLO - Fast)
            detections = []
            if self.use_yolo and self.yolo:
                with torch.no_grad():
                    results = self.yolo(cv_image, verbose=False)
                
                # Debug: Print detection results
                print(f"\n[DEBUG] Frame {self.frame_count} - YOLO Detections:")
                
                for r in results:
                    for box in r.boxes:
                        conf = float(box.conf[0])
                        cls_id = int(box.cls[0])
                        class_name = self.yolo.names[cls_id]
                        bbox = box.xyxy[0].cpu().numpy().astype(int).tolist()
                        
                        # Debug print all detections
                        print(f"  - {class_name}: {conf:.2%} @ {bbox}")
                        
                        if conf > self.detection_threshold:
                            detections.append(Detection(
                                class_name=class_name,
                                confidence=conf,
                                bbox=bbox,
                                timestamp=time.time()
                            ))
                            
                            # Draw bounding box on display image
                            x1, y1, x2, y2 = bbox
                            cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            label = f"{class_name}: {conf:.2f}"
                            cv2.putText(display_image, label, (x1, y1 - 10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                print(f"  Total detections above threshold: {len(detections)}")

            # Update detection state
            self.current_detection = detections[0] if detections else None


            # 2. VLM Reasoning (Throttled to target Hz)
            current_time = time.time()
            if (self.current_detection and 
                current_time - self.last_analysis_time >= self.analysis_interval):

                self.last_analysis_time = current_time

                # Analyze with VLM
                properties, analysis_time = self.vlm.analyze_object(
                    pil_image, self.current_detection
                )

                # Make decision
                self.current_reasoning = self.fusion.decide(
                    self.current_detection, properties, analysis_time
                )

            # Display image with detections
            cv2.imshow('YOLO Detection', display_image)
            cv2.waitKey(1)

            # 3. Display Dashboard (every frame)
            self.display_dashboard()

        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')
    
    def display_dashboard(self):
        """Display real-time reasoning dashboard in terminal"""
        # Clear terminal
        print("\033[2J\033[H", end="")
        
        # Header
        print("=" * 70)
        print("TURTLEBOT3 VISION-LANGUAGE REASONING DASHBOARD")
        print("=" * 70)
        
        # Frame info
        print(f"\nFRAME: {self.frame_count}")
        
        # Detection section
        print("\n" + "─" * 70)
        print("VISION (YOLO11 Detection)")
        print("─" * 70)
        
        if self.current_detection:
            det = self.current_detection
            print(f"  Object: {det.class_name.upper()}")
            print(f"  Confidence: {det.confidence:.2%}")
            print(f"  Bounding Box: {det.bbox}")
        else:
            print("  Status: SCANNING... (no objects detected)")
        
        # Reasoning section
        print("\n" + "─" * 70)
        print("REASONING (Moondream2 VLM)")
        print("─" * 70)
        
        if self.current_reasoning:
            res = self.current_reasoning
            print(f"  Thought: {res.reasoning}")
            print(f"  Action: {res.action.value}")
            print(f"  Confidence: {res.confidence:.2%}")
            print(f"  Analysis Time: {res.analysis_time_ms:.0f}ms")
            
            if res.properties:
                print("\n  Physical Properties:")
                print(f"    - Material: {res.properties.material}")
                print(f"    - Weight: {res.properties.weight_category} (~{res.properties.weight_grams}g)")
                print(f"    - Size: {res.properties.size_mm}mm")
                print(f"    - Fragility: {res.properties.fragility}")
                print(f"    - Graspable: {'YES' if res.properties.graspable else 'NO'}")
        else:
            print("  Status: Waiting for detection...")
        
        # System info
        print("\n" + "─" * 70)
        print("SYSTEM STATUS")
        print("─" * 70)
        
        mem = psutil.virtual_memory()
        print(f"  RAM: {mem.used / (1024**3):.1f}GB / {mem.total / (1024**3):.1f}GB ({mem.percent:.1f}%)")
        
        if torch.cuda.is_available():
            gpu_mem = torch.cuda.memory_allocated() / (1024**3)
            gpu_max = torch.cuda.max_memory_allocated() / (1024**3)
            print(f"  GPU Memory: {gpu_mem:.1f}GB (Peak: {gpu_max:.1f}GB)")
        
        print(f"  VLM Rate: {self.analysis_rate} Hz (target: 2-3 Hz)")
        print(f"  Detection Threshold: {self.detection_threshold:.2f}")
        
        print("=" * 70)
        print("Press Ctrl+C to stop")
        print("=" * 70)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = VLMReasonerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
