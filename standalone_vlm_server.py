#!/usr/bin/env python3
"""
Enhanced VLM Server with Physical Reasoning for Robotics
=========================================================
Performs detailed physical property analysis and manipulation planning.
"""

from __future__ import annotations

from flask import Flask, request, jsonify
import base64
from io import BytesIO
from PIL import Image as PILImage
import torch
import gc
import os
import time
import argparse
from typing import Optional, Dict, List, Any
from pathlib import Path
from dataclasses import dataclass, asdict
from enum import Enum

# Memory optimization
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128,expandable_segments:True'


def cleanup_memory():
    """Force garbage collection and CUDA cache cleanup"""
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


class ManipulationAction(Enum):
    """Robot manipulation actions"""
    GRASP = "GRASP"
    PUSH = "PUSH"
    AVOID = "AVOID"
    IGNORE = "IGNORE"
    STOP = "STOP"
    NAVIGATE_AROUND = "NAVIGATE_AROUND"


@dataclass
class RobotSpec:
    """TurtleBot3 + OpenMANIPULATOR specifications"""
    reach_mm: int = 400          # Maximum reach distance
    payload_g: int = 500          # Maximum payload capacity
    gripper_min_mm: int = 10      # Minimum gripper width
    gripper_max_mm: int = 100     # Maximum gripper width


@dataclass
class ObjectAnalysis:
    """Detailed object analysis result"""
    # Visual identification
    object_class: str
    confidence: float
    
    # Physical properties
    material: str
    weight_category: str          # light/medium/heavy
    weight_estimate_g: int
    fragility: str                # fragile/not_fragile
    size_mm: int
    shape: str                    # cube/sphere/cylinder/irregular
    texture: str                  # smooth/rough/soft/hard
    
    # Spatial properties
    reachable: bool
    distance_estimate: str        # close/medium/far
    path_blocking: bool
    orientation: str              # upright/tilted/lying
    
    # Manipulation feasibility
    graspable: bool
    pushable: bool
    lift_safe: bool
    
    # Decision
    action: str
    reasoning: str
    alternative_actions: List[str]
    
    # Timing
    analysis_time_ms: float


@dataclass
class SceneAnalysis:
    """Multi-object scene analysis"""
    num_objects: int
    objects: List[ObjectAnalysis]
    scene_description: str
    navigation_strategy: str
    priority_order: List[int]     # Indices of objects in priority order
    estimated_completion_time_s: float


class EnhancedVLMServer:
    """
    Enhanced VLM server with physical reasoning for robotics.
    """
    
    def __init__(self, model_id: str = 'vikhyatk/moondream2', 
                 quantization: str = '8bit', device: str = 'cuda',
                 model_cache_dir: Optional[str] = None,
                 robot_spec: Optional[RobotSpec] = None):
        self.model_id = model_id
        self.quantization = quantization
        self.device = device
        self.model_cache_dir = model_cache_dir
        self.robot = robot_spec or RobotSpec()
        self.model = None
        self.tokenizer = None
        self.inference_count = 0
        
        print(f'Initializing Enhanced VLM Server for Robotics...')
        print(f'  Model: {self.model_id}')
        print(f'  Quantization: {self.quantization}')
        print(f'  Device: {self.device}')
        print(f'\nRobot Configuration:')
        print(f'  Reach: {self.robot.reach_mm}mm')
        print(f'  Payload: {self.robot.payload_g}g')
        print(f'  Gripper: {self.robot.gripper_min_mm}-{self.robot.gripper_max_mm}mm')
        
        self.load_model()
        print('\n✅ Enhanced VLM Server ready for physical reasoning!')
    
    def load_model(self):
        """Load Moondream2 model"""
        from transformers import AutoModelForCausalLM, AutoTokenizer
        
        print('\nLoading Moondream2 VLM...')
        cleanup_memory()
        
        # Determine model path
        if self.model_cache_dir:
            model_path = Path(self.model_cache_dir).expanduser().resolve()
            if not model_path.exists():
                raise ValueError(f"Model cache directory does not exist: {model_path}")
            model_source = str(model_path)
            local_files_only = True
        else:
            model_source = self.model_id
            local_files_only = False
        
        load_kwargs = {
            'trust_remote_code': True,
            'low_cpu_mem_usage': True,
            'local_files_only': local_files_only,
        }
        
        # Apply quantization
        if self.quantization == '8bit':
            load_kwargs.update({
                'load_in_8bit': True,
                'device_map': 'auto',
                'torch_dtype': torch.float16,
            })
        elif self.quantization == '4bit':
            load_kwargs.update({
                'load_in_4bit': True,
                'device_map': 'auto',
                'torch_dtype': torch.float16,
            })
        else:
            load_kwargs.update({
                'torch_dtype': torch.float16,
                'device_map': self.device,
            })
        
        # Load model and tokenizer
        self.model = AutoModelForCausalLM.from_pretrained(model_source, **load_kwargs)
        self.tokenizer = AutoTokenizer.from_pretrained(
            model_source, trust_remote_code=True, local_files_only=local_files_only
        )
        
        # Stability patch
        if hasattr(self.model, 'vision_encoder'):
            self.model.vision_encoder.to(dtype=torch.float32)
        elif hasattr(self.model, 'vision'):
            self.model.vision.to(dtype=torch.float32)
        
        print('✓ Model loaded successfully!')
    
    def query(self, image: PILImage.Image, question: str) -> str:
        """Single VLM query"""
        cleanup_memory()
        result = self.model.query(image, question)
        return result['answer'].strip().lower()
    
    def analyze_object(self, image: PILImage.Image, object_hint: str = "object") -> ObjectAnalysis:
        """
        Comprehensive physical analysis of a single object.
        Uses multi-query approach for detailed reasoning.
        """
        start_time = time.time()
        obj = object_hint
        
        print(f'\n[Object Analysis] Analyzing {obj}...')
        
        # Query 1: Material, Weight, Shape
        q1 = f"""Look at this {obj}. What material is it made of (plastic/metal/ceramic/glass/wood/paper/fabric/rubber)? 
        How heavy does it look (light under 100g / medium 100-500g / heavy over 500g)? 
        What shape is it (cube/sphere/cylinder/box/irregular)?"""
        a1 = self.query(image, q1)
        print(f'  Q1 (material/weight/shape): {a1}')
        
        # Parse material
        materials = ['plastic', 'metal', 'ceramic', 'glass', 'wood', 'paper', 'fabric', 'rubber']
        material = next((m for m in materials if m in a1), 'unknown')
        
        # Parse weight
        if 'light' in a1 or 'small' in a1:
            weight_cat, weight_g = 'light', 50
        elif 'heavy' in a1 or 'large' in a1:
            weight_cat, weight_g = 'heavy', 700
        else:
            weight_cat, weight_g = 'medium', 250
        
        # Parse shape
        shapes = ['cube', 'sphere', 'cylinder', 'box', 'irregular']
        shape = next((s for s in shapes if s in a1), 'irregular')
        
        # Query 2: Fragility, Size, Texture
        q2 = f"""This {material} {obj}: Is it fragile or breakable (yes/no)? 
        How wide is it approximately in millimeters (20/50/80/120/150)? 
        What is its texture (smooth/rough/soft/hard)?"""
        a2 = self.query(image, q2)
        print(f'  Q2 (fragility/size/texture): {a2}')
        
        # Parse fragility
        fragility = 'fragile' if any(word in a2 for word in ['yes', 'fragile', 'breakable', 'delicate']) else 'not_fragile'
        
        # Parse size
        try:
            size_mm = int(''.join(filter(str.isdigit, a2)))
            size_mm = max(10, min(200, size_mm))
        except:
            size_mm = 60  # Default
        
        # Parse texture
        textures = ['smooth', 'rough', 'soft', 'hard']
        texture = next((t for t in textures if t in a2), 'smooth')
        
        # Query 3: Spatial Understanding
        q3 = f"""This {obj}: Is it close and within reach (yes/no)? 
        Is it blocking a path or in the way (yes/no)? 
        What is its orientation - is it standing upright, tilted, or lying down?"""
        a3 = self.query(image, q3)
        print(f'  Q3 (spatial): {a3}')
        
        # Parse spatial properties
        reachable = any(word in a3 for word in ['yes', 'close', 'reach', 'near'])
        blocking = any(word in a3 for word in ['blocking', 'way', 'path', 'obstructing'])
        
        if 'far' in a3 or 'distant' in a3:
            distance = 'far'
        elif 'close' in a3 or 'near' in a3:
            distance = 'close'
        else:
            distance = 'medium'
        
        if 'upright' in a3 or 'standing' in a3:
            orientation = 'upright'
        elif 'lying' in a3 or 'flat' in a3:
            orientation = 'lying'
        else:
            orientation = 'tilted'
        
        # Manipulation Feasibility Analysis
        graspable = (self.robot.gripper_min_mm <= size_mm <= self.robot.gripper_max_mm)
        weight_ok = weight_g <= self.robot.payload_g
        lift_safe = graspable and weight_ok and fragility == 'not_fragile'
        pushable = not fragility == 'fragile' and size_mm >= 30
        
        # Decision Logic with Detailed Reasoning
        alternative_actions = []
        
        if fragility == 'fragile':
            action = ManipulationAction.AVOID
            reasoning = (f"Object is fragile ({material}). Risk of damage is high. "
                        f"Recommend navigation around obstacle or requesting human assistance.")
            alternative_actions = ["NAVIGATE_AROUND", "REQUEST_HUMAN_HELP"]
            
        elif lift_safe and reachable:
            action = ManipulationAction.GRASP
            reasoning = (f"Graspable {material} {obj} within robot constraints. "
                        f"Size {size_mm}mm fits gripper ({self.robot.gripper_min_mm}-{self.robot.gripper_max_mm}mm), "
                        f"weight ~{weight_g}g under payload limit ({self.robot.payload_g}g). "
                        f"Execute precision grasp with {size_mm + 5}mm gripper opening.")
            alternative_actions = ["PUSH"] if pushable else []
            
        elif pushable and reachable and blocking:
            action = ManipulationAction.PUSH
            reasoning = (f"Object too large ({size_mm}mm) or heavy ({weight_g}g) for grasping, "
                        f"but can be pushed. Blocking path requires clearance. "
                        f"Execute gentle push maneuver perpendicular to travel direction.")
            alternative_actions = ["NAVIGATE_AROUND"]
            
        elif not reachable and blocking:
            action = ManipulationAction.NAVIGATE_AROUND
            reasoning = (f"Object out of reach ({distance}) but blocking path. "
                        f"Navigation around obstacle is optimal strategy. "
                        f"Plan detour path maintaining {self.robot.reach_mm}mm clearance.")
            alternative_actions = ["STOP", "REQUEST_HUMAN_HELP"]
            
        elif not blocking:
            action = ManipulationAction.IGNORE
            reasoning = (f"Object not obstructing current path and out of reach. "
                        f"No manipulation required. Continue with primary task.")
            alternative_actions = []
            
        else:
            action = ManipulationAction.STOP
            reasoning = (f"Uncertain conditions detected. Object properties: {material}, "
                        f"{weight_cat} weight, {size_mm}mm size, {fragility}. "
                        f"Recommend manual inspection or updated sensor data.")
            alternative_actions = ["REQUEST_CLARIFICATION"]
        
        analysis_time = (time.time() - start_time) * 1000
        
        return ObjectAnalysis(
            object_class=obj,
            confidence=0.85,
            material=material,
            weight_category=weight_cat,
            weight_estimate_g=weight_g,
            fragility=fragility,
            size_mm=size_mm,
            shape=shape,
            texture=texture,
            reachable=reachable,
            distance_estimate=distance,
            path_blocking=blocking,
            orientation=orientation,
            graspable=graspable,
            pushable=pushable,
            lift_safe=lift_safe,
            action=action.value,
            reasoning=reasoning,
            alternative_actions=alternative_actions,
            analysis_time_ms=analysis_time
        )
    
    def analyze_scene(self, image: PILImage.Image) -> SceneAnalysis:
        """
        Analyze entire scene with multiple objects.
        Provides navigation and manipulation strategy.
        """
        start_time = time.time()
        
        print(f'\n[Scene Analysis] Analyzing multi-object scene...')
        
        # Query 1: Object detection and counting
        q1 = "How many distinct objects do you see in this image? List them briefly."
        a1 = self.query(image, q1)
        print(f'  Scene detection: {a1}')
        
        # Parse number of objects (simple heuristic)
        try:
            num_objects = int(''.join(filter(str.isdigit, a1.split()[0])))
        except:
            num_objects = a1.lower().count('and') + 1
        
        # Query 2: Scene layout
        q2 = """Describe the spatial layout: Which objects are closest? 
        Which are blocking a straight path? Which are on the sides?"""
        a2 = self.query(image, q2)
        print(f'  Spatial layout: {a2}')
        
        # Query 3: Overall navigation strategy
        q3 = """To navigate through this scene safely, what would be the best approach? 
        Should obstacles be moved, avoided, or is the path clear?"""
        a3 = self.query(image, q3)
        print(f'  Navigation strategy: {a3}')
        
        # Create simplified scene description
        scene_description = (f"Scene contains approximately {num_objects} objects. "
                           f"Layout: {a2[:100]}... "
                           f"Recommended approach: {a3[:100]}...")
        
        # For demo, analyze first object in detail
        # In production, you'd detect and analyze each object
        objects = [self.analyze_object(image, "primary object")]
        
        # Determine priority order (closer + blocking = higher priority)
        priority_order = [0]  # In real system, sort by blocking + distance
        
        # Estimate completion time
        estimated_time = sum(3.0 if obj.action == 'GRASP' else 
                           2.0 if obj.action == 'PUSH' else 
                           1.5 if obj.action == 'NAVIGATE_AROUND' else 
                           0.5 for obj in objects)
        
        analysis_time = (time.time() - start_time) * 1000
        
        return SceneAnalysis(
            num_objects=num_objects,
            objects=objects,
            scene_description=scene_description,
            navigation_strategy=a3,
            priority_order=priority_order,
            estimated_completion_time_s=estimated_time
        )


# Flask app
app = Flask(__name__)
vlm_server: Optional[EnhancedVLMServer] = None


@app.route('/health', methods=['GET'])
def health():
    """Health check"""
    return jsonify({
        'status': 'healthy',
        'model_loaded': vlm_server is not None,
        'inference_count': vlm_server.inference_count if vlm_server else 0,
        'capabilities': ['object_analysis', 'scene_analysis', 'physical_reasoning']
    })


@app.route('/analyze', methods=['POST'])
def analyze():
    """
    Detailed object analysis endpoint (compatible with ROS2 bridge).
    
    Request JSON:
    {
        "image": "<base64 JPEG>",
        "object_class": "cup",  # optional hint
        "timestamp": 1234567890.123
    }
    
    Returns ObjectAnalysis as JSON
    """
    if vlm_server is None:
        return jsonify({'error': 'VLM server not initialized'}), 500
    
    try:
        data = request.get_json()
        
        if not data or 'image' not in data:
            return jsonify({'error': 'Missing image in request'}), 400
        
        # Decode image
        img_base64 = data['image']
        img_bytes = base64.b64decode(img_base64)
        image = PILImage.open(BytesIO(img_bytes))
        
        object_hint = data.get('object_class', 'object')
        
        # Perform analysis
        analysis = vlm_server.analyze_object(image, object_hint)
        vlm_server.inference_count += 1
        
        return jsonify(asdict(analysis))
        
    except Exception as e:
        print(f'Analysis error: {e}')
        return jsonify({'error': str(e)}), 500


@app.route('/analyze_scene', methods=['POST'])
def analyze_scene():
    """
    Multi-object scene analysis endpoint.
    
    Request JSON:
    {
        "image": "<base64 JPEG>",
        "timestamp": 1234567890.123
    }
    
    Returns SceneAnalysis as JSON
    """
    if vlm_server is None:
        return jsonify({'error': 'VLM server not initialized'}), 500
    
    try:
        data = request.get_json()
        
        if not data or 'image' not in data:
            return jsonify({'error': 'Missing image in request'}), 400
        
        # Decode image
        img_base64 = data['image']
        img_bytes = base64.b64decode(img_base64)
        image = PILImage.open(BytesIO(img_bytes))
        
        # Perform scene analysis
        analysis = vlm_server.analyze_scene(image)
        vlm_server.inference_count += 1
        
        return jsonify(asdict(analysis))
        
    except Exception as e:
        print(f'Scene analysis error: {e}')
        return jsonify({'error': str(e)}), 500


@app.route('/inference', methods=['POST'])
def inference():
    """Basic inference endpoint (backwards compatible)"""
    if vlm_server is None:
        return jsonify({'error': 'VLM server not initialized'}), 500
    
    try:
        data = request.get_json()
        
        if not data or 'image' not in data:
            return jsonify({'error': 'Missing image in request'}), 400
        
        img_base64 = data['image']
        prompt = data.get('prompt', 'Describe this image in detail.')
        
        img_bytes = base64.b64decode(img_base64)
        image = PILImage.open(BytesIO(img_bytes))
        
        start_time = time.time()
        result = vlm_server.query(image, prompt)
        inference_time_ms = (time.time() - start_time) * 1000
        
        vlm_server.inference_count += 1
        
        return jsonify({
            'result': result,
            'inference_time_ms': inference_time_ms,
            'timestamp': data.get('timestamp', time.time())
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Enhanced VLM Server with Physical Reasoning for Robotics'
    )
    parser.add_argument('--host', type=str, default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5000)
    parser.add_argument('--model', type=str, default='vikhyatk/moondream2')
    parser.add_argument('--quantization', type=str, default='8bit',
                       choices=['8bit', '4bit', 'none'])
    parser.add_argument('--device', type=str, default='cuda',
                       choices=['cuda', 'cpu'])
    parser.add_argument('--model-cache-dir', type=str, default=None)
    
    # Robot specs
    parser.add_argument('--reach-mm', type=int, default=400)
    parser.add_argument('--payload-g', type=int, default=500)
    parser.add_argument('--gripper-min-mm', type=int, default=10)
    parser.add_argument('--gripper-max-mm', type=int, default=100)
    
    args = parser.parse_args()
    
    # Initialize robot spec
    robot_spec = RobotSpec(
        reach_mm=args.reach_mm,
        payload_g=args.payload_g,
        gripper_min_mm=args.gripper_min_mm,
        gripper_max_mm=args.gripper_max_mm
    )
    
    # Initialize VLM server
    global vlm_server
    vlm_server = EnhancedVLMServer(
        model_id=args.model,
        quantization=args.quantization,
        device=args.device,
        model_cache_dir=args.model_cache_dir,
        robot_spec=robot_spec
    )
    
    # Start Flask server
    print(f'\n{"="*70}')
    print(f'🤖 Enhanced VLM Server for Robotics')
    print(f'{"="*70}')
    print(f'Server: http://{args.host}:{args.port}')
    print(f'Endpoints:')
    print(f'  GET  /health         - Health check')
    print(f'  POST /analyze        - Single object analysis (detailed)')
    print(f'  POST /analyze_scene  - Multi-object scene analysis')
    print(f'  POST /inference      - Basic VLM inference')
    print(f'{"="*70}\n')
    
    app.run(host=args.host, port=args.port, threaded=False)


if __name__ == '__main__':
    main()
