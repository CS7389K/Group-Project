#!/usr/bin/env python3
"""
Enhanced VLM Inference Server for ROS2 Bridge
Detailed physical reasoning and multi-object scene understanding
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
from typing import List, Optional
from enum import Enum

os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128'

def cleanup():
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

class ManipulationAction(Enum):
    GRASP = "GRASP"
    PUSH = "PUSH"
    AVOID = "AVOID"
    IGNORE = "IGNORE"
    STOP = "STOP"
    NAVIGATE_AROUND = "NAVIGATE_AROUND"

@dataclass
class RobotSpec:
    reach_mm: int = 400
    payload_g: int = 500
    gripper_min_mm: int = 10
    gripper_max_mm: int = 100

@dataclass
class VLMAnalysis:
    # Object identification
    object_class: str
    confidence: float
    
    # Physical properties (detailed)
    material: str
    weight_category: str
    weight_grams: int
    fragility: str
    size_mm: int
    shape: str                    # NEW: cube/sphere/cylinder/irregular
    texture: str                  # NEW: smooth/rough/soft/hard
    
    # Spatial properties (detailed)
    reachable: bool
    distance_estimate: str        # NEW: close/medium/far
    path_blocking: bool
    orientation: str              # NEW: upright/tilted/lying
    
    # Manipulation feasibility (detailed)
    graspable: bool
    pushable: bool               # NEW
    lift_safe: bool              # NEW
    gripper_opening_mm: int      # NEW: recommended gripper width
    
    # Decision (enhanced)
    action: str
    reasoning: str               # ENHANCED: much more detailed
    alternative_actions: List[str]  # NEW: backup strategies
    execution_steps: List[str]   # NEW: step-by-step instructions
    
    # Multi-object awareness
    scene_context: str           # NEW: understanding of surrounding objects
    navigation_advice: str       # NEW: path planning recommendations
    
    # Metadata
    analysis_time_ms: float
    yolo_guided: bool = False
    yolo_confidence: float = 0.0

@dataclass
class SceneAnalysis:
    """Multi-object scene understanding"""
    num_objects: int
    object_list: List[str]
    scene_description: str
    primary_obstacles: List[str]
    navigation_strategy: str
    recommended_sequence: List[str]  # Order to handle objects
    estimated_completion_time_s: float
    safety_concerns: List[str]

class EnhancedMoondream2VLM:
    """Enhanced VLM with detailed physical reasoning"""
    
    def __init__(self, robot_spec: RobotSpec):
        self.robot = robot_spec
        self.model = None
        self.query_count = 0
        
    def load(self):
        from transformers import AutoModelForCausalLM
        
        print("Loading Enhanced Moondream2 VLM...")
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
        print("✅ Enhanced VLM loaded with physical reasoning capabilities")
    
    def query(self, image: Image.Image, question: str) -> str:
        cleanup()
        self.query_count += 1
        result = self.model.query(image, question)
        return result['answer'].strip().lower()
    
    def analyze_detailed(self, image: Image.Image, object_class: str = "object",
                        yolo_confidence: float = 0.0) -> VLMAnalysis:
        """
        Enhanced analysis with detailed physical reasoning.
        Uses 5-query approach for comprehensive understanding.
        """
        start = time.time()
        obj = object_class
        yolo_guided = yolo_confidence > 0.0
        
        print(f"\n{'='*70}")
        print(f"[Enhanced VLM Analysis #{self.query_count + 1}]")
        if yolo_guided:
            print(f"YOLO-Guided: {obj} (confidence: {yolo_confidence:.2f})")
        print(f"{'='*70}")
        
        # ===== Query 1: Material, Weight, Shape =====
        q1 = f"""Analyze this {obj} carefully. 
        What material is it made of (plastic/metal/ceramic/glass/wood/paper/fabric/rubber)? 
        Estimate its weight: light (under 100g), medium (100-500g), or heavy (over 500g)?
        What is its shape: cube, sphere, cylinder, box, or irregular?"""
        
        a1 = self.query(image, q1)
        print(f"\n[Q1] Material/Weight/Shape:")
        print(f"  → {a1}")
        
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
        
        # ===== Query 2: Fragility, Size, Texture =====
        q2 = f"""Examine this {material} {obj} more closely.
        Is it fragile or breakable? Would it break if dropped or squeezed? (yes/no)
        Approximately how wide is it in millimeters? (estimate: 20/50/80/120/150/200)
        What is its surface texture: smooth, rough, soft, or hard?"""
        
        a2 = self.query(image, q2)
        print(f"\n[Q2] Fragility/Size/Texture:")
        print(f"  → {a2}")
        
        # Parse fragility with more nuance
        fragility_indicators = ['yes', 'fragile', 'breakable', 'delicate', 'brittle']
        fragility = 'fragile' if any(word in a2 for word in fragility_indicators) else 'not_fragile'
        
        # Parse size
        try:
            size_mm = int(''.join(filter(str.isdigit, a2)))
            size_mm = max(10, min(250, size_mm))
        except:
            size_mm = 60  # Default medium size
        
        # Parse texture
        textures = ['smooth', 'rough', 'soft', 'hard']
        texture = next((t for t in textures if t in a2), 'smooth')
        
        # ===== Query 3: Spatial Positioning =====
        q3 = f"""Analyze the position of this {obj} in the scene.
        Is it close and within arm's reach, or far away? (close/medium/far)
        Is it blocking a path or passageway? (yes/no)
        What is its orientation: standing upright, tilted at an angle, or lying flat?"""
        
        a3 = self.query(image, q3)
        print(f"\n[Q3] Spatial Position:")
        print(f"  → {a3}")
        
        # Parse distance
        if 'far' in a3 or 'distant' in a3 or 'away' in a3:
            distance = 'far'
        elif 'close' in a3 or 'near' in a3 or 'reach' in a3:
            distance = 'close'
        else:
            distance = 'medium'
        
        reachable = distance in ['close', 'medium']
        
        # Parse blocking
        blocking = any(word in a3 for word in ['blocking', 'block', 'way', 'path', 'obstructing'])
        
        # Parse orientation
        if 'upright' in a3 or 'standing' in a3 or 'vertical' in a3:
            orientation = 'upright'
        elif 'lying' in a3 or 'flat' in a3 or 'horizontal' in a3:
            orientation = 'lying'
        else:
            orientation = 'tilted'
        
        # ===== Query 4: Scene Context (Multi-object awareness) =====
        q4 = f"""Look at the entire scene around this {obj}.
        Are there other objects nearby? If so, what are they?
        Is there a clear path around this object, or is it surrounded?
        What would be the safest way to approach this object?"""
        
        a4 = self.query(image, q4)
        print(f"\n[Q4] Scene Context:")
        print(f"  → {a4}")
        
        scene_context = a4[:150] + "..." if len(a4) > 150 else a4
        
        # ===== Query 5: Manipulation Feasibility =====
        q5 = f"""Consider manipulating this {material} {obj}.
        Could it be safely picked up with a robotic gripper?
        If not graspable, could it be pushed or moved by applying force?
        Are there any risks or concerns with touching or moving this object?"""
        
        a5 = self.query(image, q5)
        print(f"\n[Q5] Manipulation Assessment:")
        print(f"  → {a5}")
        
        # ===== Physical Feasibility Analysis =====
        graspable = (self.robot.gripper_min_mm <= size_mm <= self.robot.gripper_max_mm)
        weight_ok = weight_g <= self.robot.payload_g
        lift_safe = graspable and weight_ok and fragility == 'not_fragile'
        
        # Pushability depends on size and fragility
        pushable = not (fragility == 'fragile') and size_mm >= 30 and weight_g <= 2000
        
        # Recommended gripper opening
        gripper_opening_mm = size_mm + 5 if graspable else 0
        
        # ===== Decision Logic with Detailed Reasoning =====
        alternative_actions = []
        execution_steps = []
        navigation_advice = ""
        
        if fragility == 'fragile':
            action = ManipulationAction.AVOID
            reasoning = (
                f"⚠️ FRAGILE OBJECT DETECTED: This {material} {obj} is breakable and poses a risk of damage. "
                f"Material properties indicate brittleness. "
                f"Physical contact could result in breakage or injury from fragments. "
                f"Robot safety protocol: maintain minimum {self.robot.reach_mm}mm clearance. "
                f"Recommended: Navigate around obstacle using path planning with elevated safety margins."
            )
            alternative_actions = ["NAVIGATE_AROUND", "REQUEST_HUMAN_ASSISTANCE"]
            execution_steps = [
                "1. Stop current motion immediately",
                "2. Calculate detour path with 400mm+ safety clearance",
                "3. Update costmap to mark object as high-penalty zone",
                "4. Execute navigation around obstacle",
                "5. Resume primary task after clearance"
            ]
            navigation_advice = f"Plan wide berth. Avoid collision at all costs. Consider path recomputation."
            
        elif lift_safe and reachable:
            action = ManipulationAction.GRASP
            reasoning = (
                f"✓ GRASP FEASIBLE: Object meets all manipulation criteria. "
                f"Physical analysis: {material} material, {weight_cat} weight (~{weight_g}g < {self.robot.payload_g}g limit), "
                f"dimensions {size_mm}mm (within gripper range {self.robot.gripper_min_mm}-{self.robot.gripper_max_mm}mm). "
                f"Structural integrity: {fragility}, safe for manipulation. "
                f"Shape: {shape}, texture: {texture}, orientation: {orientation}. "
                f"Grasp strategy: precision grasp with {gripper_opening_mm}mm gripper opening. "
                f"Approach from {orientation} orientation for optimal grip stability."
            )
            alternative_actions = ["PUSH"] if pushable else ["STOP_AND_REASSESS"]
            execution_steps = [
                f"1. Approach object from optimal angle based on {orientation} orientation",
                f"2. Position gripper at {gripper_opening_mm}mm opening",
                f"3. Align with object center of mass",
                f"4. Close gripper to secure grip (force: {weight_g * 2}g for safety margin)",
                f"5. Lift vertically 100mm to verify grip security",
                f"6. Transport to target location with {weight_g}g payload compensation",
                "7. Release at destination and retract"
            ]
            navigation_advice = f"Clear path required. Approach distance: {distance}. No obstacles in manipulation workspace."
            
        elif pushable and reachable and blocking:
            action = ManipulationAction.PUSH
            reasoning = (
                f"⚠️ PUSH REQUIRED: Object exceeds grasp parameters but can be moved. "
                f"Size constraint: {size_mm}mm exceeds gripper maximum {self.robot.gripper_max_mm}mm OR "
                f"weight constraint: {weight_g}g exceeds payload {self.robot.payload_g}g. "
                f"However, object is blocking path and must be cleared. "
                f"Material {material} is {fragility}, suitable for force application. "
                f"Push strategy: Apply controlled force perpendicular to travel direction. "
                f"Estimated displacement: 200-300mm sufficient to clear path."
            )
            alternative_actions = ["NAVIGATE_AROUND", "REQUEST_ASSISTANCE"]
            execution_steps = [
                "1. Approach object to contact range (within 50mm)",
                "2. Position end-effector against object center of mass",
                f"3. Apply gradual force (start 10N, max 50N based on {weight_g}g mass)",
                "4. Push perpendicular to intended path direction",
                "5. Monitor displacement (target: 250mm clearance)",
                "6. Verify path is clear before proceeding",
                "7. Resume navigation"
            ]
            navigation_advice = f"After push, verify clearance. May require multiple push operations."
            
        elif not reachable and blocking:
            action = ManipulationAction.NAVIGATE_AROUND
            reasoning = (
                f"🚧 OBSTACLE OUT OF REACH: Object at {distance} distance (>{self.robot.reach_mm}mm max reach). "
                f"Object is blocking intended path but cannot be manipulated due to distance constraint. "
                f"Navigation strategy: path recomputation required. "
                f"Scene analysis indicates {scene_context}. "
                f"Optimal approach: circumnavigate obstacle maintaining minimum safety clearance."
            )
            alternative_actions = ["STOP_AND_WAIT", "REQUEST_PATH_GUIDANCE"]
            execution_steps = [
                "1. Update occupancy grid with obstacle position",
                "2. Run path planner (e.g., A*, DWA) with updated costmap",
                "3. Calculate detour trajectory",
                f"4. Verify clearance of {self.robot.reach_mm}mm minimum from obstacle",
                "5. Execute planned trajectory",
                "6. Monitor sensors during navigation",
                "7. Replan if unexpected obstacles encountered"
            ]
            navigation_advice = f"Detour required. Estimate +15-30 seconds navigation time. Path complexity: moderate."
            
        elif not blocking:
            action = ManipulationAction.IGNORE
            reasoning = (
                f"✓ NO ACTION REQUIRED: Object not obstructing current objectives. "
                f"Spatial analysis: {distance} distance, not blocking path. "
                f"Reachability: {reachable}, but intervention unnecessary. "
                f"Resource optimization: conserve battery and time by ignoring non-critical object. "
                f"Continue primary task execution without diversion."
            )
            alternative_actions = []
            execution_steps = [
                "1. Log object position in environment map for future reference",
                "2. Continue with primary navigation/manipulation task",
                "3. Monitor peripherally in case object moves into path"
            ]
            navigation_advice = f"Proceed on current path. Object pose: {orientation}, no interference expected."
            
        else:
            action = ManipulationAction.STOP
            reasoning = (
                f"⚠️ UNCERTAIN CONDITIONS: Unable to determine safe manipulation strategy. "
                f"Ambiguous properties: material={material}, weight={weight_cat}({weight_g}g), "
                f"size={size_mm}mm, fragility={fragility}, distance={distance}. "
                f"Conflicting constraints detected. Human operator input required for safety. "
                f"Recommendation: halt operations and request manual assessment."
            )
            alternative_actions = ["REQUEST_HUMAN_ASSESSMENT", "CAPTURE_ADDITIONAL_SENSOR_DATA"]
            execution_steps = [
                "1. Stop all motion immediately",
                "2. Capture high-resolution images from multiple angles",
                "3. Log all sensor data (camera, lidar, ultrasonic)",
                "4. Send alert to human operator with scene snapshot",
                "5. Await manual override or clarification",
                "6. Resume only after explicit approval"
            ]
            navigation_advice = f"Hold position. Do not proceed without clearance."
        
        # ===== Timing =====
        analysis_time = (time.time() - start) * 1000
        
        print(f"\n{'='*70}")
        print(f"DECISION: {action.value}")
        print(f"Analysis Time: {analysis_time:.0f}ms")
        print(f"{'='*70}\n")
        
        # Calculate confidence
        base_confidence = 0.80
        if yolo_guided:
            combined_conf = (base_confidence * 0.7) + (yolo_confidence * 0.3)
        else:
            combined_conf = base_confidence
        
        return VLMAnalysis(
            object_class=obj,
            confidence=combined_conf,
            material=material,
            weight_category=weight_cat,
            weight_grams=weight_g,
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
            gripper_opening_mm=gripper_opening_mm,
            action=action.value,
            reasoning=reasoning,
            alternative_actions=alternative_actions,
            execution_steps=execution_steps,
            scene_context=scene_context,
            navigation_advice=navigation_advice,
            analysis_time_ms=analysis_time,
            yolo_guided=yolo_guided,
            yolo_confidence=yolo_confidence
        )
    
    def analyze_scene(self, image: Image.Image) -> SceneAnalysis:
        """
        Multi-object scene analysis for complex navigation scenarios.
        """
        start = time.time()
        
        print(f"\n{'='*70}")
        print(f"[SCENE ANALYSIS - Multi-Object Detection]")
        print(f"{'='*70}")
        
        # Query 1: Object counting and identification
        q1 = "How many distinct objects do you see? List each object briefly."
        a1 = self.query(image, q1)
        print(f"\n[Objects Detected]:\n  {a1}")
        
        # Parse objects
        try:
            num_objects = len([x for x in a1.split(',') if x.strip()])
        except:
            num_objects = max(1, a1.lower().count('and') + 1)
        
        object_list = [x.strip() for x in a1.replace(' and ', ',').split(',')][:5]
        
        # Query 2: Spatial arrangement
        q2 = """Describe the spatial layout: 
        Which objects are in the foreground (closest)?
        Which are in the background (farthest)?
        Which objects are blocking a straight path forward?"""
        a2 = self.query(image, q2)
        print(f"\n[Spatial Layout]:\n  {a2}")
        
        # Query 3: Navigation strategy
        q3 = """To navigate safely through this scene:
        Should obstacles be moved, pushed aside, or avoided?
        What is the best path: straight, left detour, or right detour?
        Are there any immediate safety concerns?"""
        a3 = self.query(image, q3)
        print(f"\n[Navigation Strategy]:\n  {a3}")
        
        # Extract primary obstacles
        primary_obstacles = []
        if 'blocking' in a2.lower():
            primary_obstacles = [obj for obj in object_list if len(obj) > 2][:3]
        
        # Safety assessment
        safety_concerns = []
        if 'fragile' in a3 or 'glass' in a1.lower() or 'ceramic' in a1.lower():
            safety_concerns.append("Fragile objects present - avoid contact")
        if 'heavy' in a3 or 'large' in a1.lower():
            safety_concerns.append("Heavy objects detected - push may be difficult")
        if num_objects >= 4:
            safety_concerns.append("Cluttered environment - collision risk elevated")
        
        # Recommended sequence (prioritize by blocking + reachability)
        recommended_sequence = []
        if 'move' in a3 or 'push' in a3:
            recommended_sequence.append("Clear blocking obstacles first")
        if 'avoid' in a3 or 'detour' in a3:
            recommended_sequence.append("Navigate around immovable objects")
        recommended_sequence.append("Proceed to target after clearance")
        
        # Time estimation (rough heuristic)
        estimated_time = num_objects * 3.0 + len(safety_concerns) * 2.0
        
        scene_description = (
            f"Scene contains {num_objects} objects: {', '.join(object_list)}. "
            f"Layout: {a2[:100]}... "
            f"Strategy: {a3[:100]}..."
        )
        
        analysis_time = (time.time() - start) * 1000
        print(f"\n{'='*70}")
        print(f"Scene Analysis Complete: {analysis_time:.0f}ms")
        print(f"{'='*70}\n")
        
        return SceneAnalysis(
            num_objects=num_objects,
            object_list=object_list,
            scene_description=scene_description,
            primary_obstacles=primary_obstacles,
            navigation_strategy=a3,
            recommended_sequence=recommended_sequence,
            estimated_completion_time_s=estimated_time,
            safety_concerns=safety_concerns
        )

# Initialize Flask app and VLM
app = Flask(__name__)
robot = RobotSpec()
vlm = EnhancedMoondream2VLM(robot)

@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({
        'status': 'healthy',
        'model_loaded': vlm.model is not None,
        'total_queries': vlm.query_count,
        'capabilities': ['detailed_object_analysis', 'multi_object_scene', 'physical_reasoning', 'navigation_planning'],
        'robot_spec': {
            'reach_mm': robot.reach_mm,
            'payload_g': robot.payload_g,
            'gripper_range_mm': f"{robot.gripper_min_mm}-{robot.gripper_max_mm}"
        }
    })

@app.route('/analyze', methods=['POST'])
def analyze():
    """Enhanced analysis endpoint - detailed physical reasoning"""
    try:
        data = request.json
        
        # Decode base64 image
        img_base64 = data['image']
        img_bytes = base64.b64decode(img_base64)
        image = Image.open(io.BytesIO(img_bytes)).convert('RGB')
        
        # Get object class and YOLO confidence
        object_class = data.get('object_class', 'object')
        yolo_confidence = data.get('yolo_confidence', 0.0)
        
        # Run enhanced VLM inference
        analysis = vlm.analyze_detailed(image, object_class, yolo_confidence)
        
        # Return as JSON
        return jsonify(asdict(analysis))
        
    except Exception as e:
        print(f"Analysis error: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/analyze_scene', methods=['POST'])
def analyze_scene():
    """Multi-object scene analysis endpoint"""
    try:
        data = request.json
        
        # Decode base64 image
        img_base64 = data['image']
        img_bytes = base64.b64decode(img_base64)
        image = Image.open(io.BytesIO(img_bytes)).convert('RGB')
        
        # Run scene analysis
        scene = vlm.analyze_scene(image)
        
        # Return as JSON
        return jsonify(asdict(scene))
        
    except Exception as e:
        print(f"Scene analysis error: {e}")
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    print("=" * 70)
    print("🤖 Enhanced VLM ROS2 Inference Server")
    print("Moondream2 + TurtleBot3 with Physical Reasoning")
    print("=" * 70)
    
    # Load model
    vlm.load()
    
    print(f"\nRobot Configuration:")
    print(f"  Max Reach: {robot.reach_mm}mm")
    print(f"  Max Payload: {robot.payload_g}g")
    print(f"  Gripper Range: {robot.gripper_min_mm}-{robot.gripper_max_mm}mm")
    
    print(f"\n✅ Server ready on http://0.0.0.0:5000")
    print(f"\nEndpoints:")
    print(f"  GET  /health         - Health check + capabilities")
    print(f"  POST /analyze        - Detailed object analysis (5-query)")
    print(f"  POST /analyze_scene  - Multi-object scene understanding")
    print("=" * 70)
    
    # Start Flask server
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
