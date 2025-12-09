#!/usr/bin/env python3
"""
VLM Inference Server for ROS2 Bridge
Uses existing Moondream2 system from moondream2_turtlebot3 project
"""
from flask import Flask, request, jsonify
import torch
import gc
import os
import base64
import io
import time
import json
from PIL import Image
from dataclasses import dataclass, asdict
from enum import Enum

# Import your existing system
import sys
sys.path.append('/workspace/scripts')

os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128'

def cleanup():
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

# Use your existing classes
class ManipulationAction(Enum):
    GRASP = "GRASP"
    PUSH = "PUSH"
    AVOID = "AVOID"
    IGNORE = "IGNORE"
    STOP = "STOP"

@dataclass
class RobotSpec:
    reach_mm: int = 400
    payload_g: int = 500
    gripper_min_mm: int = 10
    gripper_max_mm: int = 100

@dataclass
class VLMAnalysis:
    material: str
    weight_category: str
    weight_grams: int
    fragility: str
    size_mm: int
    reachable: bool
    path_blocking: bool
    graspable: bool
    action: str
    reasoning: str
    confidence: float
    analysis_time_ms: float
    yolo_guided: bool = False
    yolo_confidence: float = 0.0

class Moondream2VLM:
    """VLM inference - integrated with existing system"""
    
    def __init__(self, robot_spec: RobotSpec):
        self.robot = robot_spec
        self.model = None
        self.query_count = 0
        
    def load(self):
        from transformers import AutoModelForCausalLM
        
        print("Loading Moondream2 VLM...")
        cleanup()
        
        model_id = "vikhyatk/moondream2"
        self.model = AutoModelForCausalLM.from_pretrained(
            model_id,
            trust_remote_code=True,
            device_map="auto",
            torch_dtype=torch.float16,
            low_cpu_mem_usage=True,
            max_memory={0: "10GB"}
        )
        print("✅ VLM loaded")
    
    def query(self, image: Image.Image, question: str) -> str:
        cleanup()
        self.query_count += 1
        result = self.model.query(image, question)
        return result['answer'].strip().lower()
    
    def analyze(self, image: Image.Image, object_class: str = "object",
                yolo_confidence: float = 0.0) -> VLMAnalysis:
        """3-query optimized analysis from your existing system"""
        start = time.time()
        obj = object_class
        yolo_guided = yolo_confidence > 0.0
        
        print(f"\n[VLM Analysis #{self.query_count + 1}]")
        if yolo_guided:
            print(f"  YOLO guidance: {obj} @ {yolo_confidence:.2f}")
        
        # Query 1: Material + Weight
        q1 = f"This {obj}: what material (plastic/metal/ceramic/glass/wood) and weight (light/medium/heavy)?"
        a1 = self.query(image, q1)
        print(f"  Q1: {a1}")
        
        material = 'unknown'
        for mat in ['plastic', 'metal', 'ceramic', 'glass', 'wood', 'paper']:
            if mat in a1:
                material = mat
                break
        
        if 'light' in a1:
            weight_cat, weight_g = 'light', 50
        elif 'heavy' in a1:
            weight_cat, weight_g = 'heavy', 700
        else:
            weight_cat, weight_g = 'medium', 250
        
        # Query 2: Fragility + Size
        q2 = f"This {material} {obj}: is it fragile (yes/no) and approximately how wide in mm (20/50/80/150)?"
        a2 = self.query(image, q2)
        print(f"  Q2: {a2}")
        
        fragility = 'fragile' if 'yes' in a2 or 'fragile' in a2 else 'not_fragile'
        
        try:
            size_mm = int(''.join(filter(str.isdigit, a2)))
            size_mm = max(10, min(200, size_mm))
        except:
            size_mm = 50
        
        # Query 3: Spatial
        q3 = f"This {obj}: is it close/reachable (yes/no) and blocking a path (yes/no)?"
        a3 = self.query(image, q3)
        print(f"  Q3: {a3}")
        
        reachable = 'yes' in a3 or 'close' in a3 or 'reachable' in a3
        blocking = 'blocking' in a3 or 'block' in a3
        
        # Decision logic (from your existing system)
        graspable = (self.robot.gripper_min_mm <= size_mm <= self.robot.gripper_max_mm)
        weight_ok = weight_g <= self.robot.payload_g
        
        if fragility == 'fragile':
            action = ManipulationAction.AVOID
            reason = f"Fragile {material} object - avoid contact"
        elif weight_ok and graspable and reachable and fragility == 'not_fragile':
            action = ManipulationAction.GRASP
            reason = f"Graspable: {weight_cat} ({weight_g}g), {size_mm}mm {material}"
        elif not graspable and weight_ok and reachable:
            action = ManipulationAction.PUSH
            reason = f"Too large ({size_mm}mm) for gripper but can push"
        elif not weight_ok and reachable:
            action = ManipulationAction.PUSH
            reason = f"Too heavy ({weight_g}g) but can push"
        elif not reachable:
            action = ManipulationAction.IGNORE
            reason = "Out of reach"
        elif not blocking:
            action = ManipulationAction.IGNORE
            reason = "Not blocking path"
        else:
            action = ManipulationAction.STOP
            reason = "Uncertain conditions"
        
        analysis_time = (time.time() - start) * 1000
        
        print(f"  → Decision: {action.value}")
        print(f"  → Time: {analysis_time:.0f}ms")
        
        # Calculate combined confidence
        base_confidence = 0.80
        if yolo_guided:
            combined_conf = (base_confidence * 0.7) + (yolo_confidence * 0.3)
        else:
            combined_conf = base_confidence
        
        return VLMAnalysis(
            material=material,
            weight_category=weight_cat,
            weight_grams=weight_g,
            fragility=fragility,
            size_mm=size_mm,
            reachable=reachable,
            path_blocking=blocking,
            graspable=graspable,
            action=action.value,
            reasoning=reason,
            confidence=combined_conf,
            analysis_time_ms=analysis_time,
            yolo_guided=yolo_guided,
            yolo_confidence=yolo_confidence
        )

# Initialize Flask app and VLM
app = Flask(__name__)
robot = RobotSpec()
vlm = Moondream2VLM(robot)

@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({
        'status': 'healthy',
        'model_loaded': vlm.model is not None,
        'total_queries': vlm.query_count,
        'robot_spec': {
            'reach_mm': robot.reach_mm,
            'payload_g': robot.payload_g,
            'gripper_range_mm': f"{robot.gripper_min_mm}-{robot.gripper_max_mm}"
        }
    })

@app.route('/analyze', methods=['POST'])
def analyze():
    """Main inference endpoint - ROS2 compatible"""
    try:
        data = request.json
        
        # Decode base64 image
        img_base64 = data['image']
        img_bytes = base64.b64decode(img_base64)
        image = Image.open(io.BytesIO(img_bytes)).convert('RGB')
        
        # Get object class and YOLO confidence
        object_class = data.get('object_class', 'object')
        yolo_confidence = data.get('yolo_confidence', 0.0)
        
        # Run VLM inference
        analysis = vlm.analyze(image, object_class, yolo_confidence)
        
        # Return as JSON
        return jsonify(asdict(analysis))
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    print("=" * 70)
    print("VLM ROS2 Inference Server")
    print("Moondream2 + TurtleBot3 Integration")
    print("=" * 70)
    
    # Load model
    vlm.load()
    
    print(f"\nRobot Spec:")
    print(f"  Reach: {robot.reach_mm}mm")
    print(f"  Payload: {robot.payload_g}g")
    print(f"  Gripper: {robot.gripper_min_mm}-{robot.gripper_max_mm}mm")
    
    print(f"\n✅ Server ready on http://0.0.0.0:5000")
    print(f"   Endpoints:")
    print(f"     GET  /health  - Health check")
    print(f"     POST /analyze - VLM inference")
    print("=" * 70)
    
    # Start Flask server
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
