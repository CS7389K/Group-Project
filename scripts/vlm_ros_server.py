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
    
    def analyze_with_yolo(self, image: Image.Image, yolo_class: str,
                         yolo_confidence: float, yolo_size_mm: int,
                         yolo_spatial: dict) -> VLMAnalysis:
        """
        YOLO-enhanced analysis: Use YOLO data to guide VLM queries
        and cross-validate object recognition
        """
        start = time.time()
        
        print(f"[VLM Analysis #{self.query_count + 1}] YOLO-Enhanced")
        print(f"  YOLO: {yolo_class} @ {yolo_confidence:.2f}, ~{yolo_size_mm}mm")
        
        # Query 1: Material & Weight (cross-validate with YOLO class)
        q1 = f"""I see a {yolo_class} in this image. Confirm this is correct and describe:
1. Material: What is it made of? (plastic/metal/ceramic/glass/wood/paper/fabric/rubber)
2. Weight: Estimate its weight category (light <100g / medium 100-500g / heavy >500g)
3. Does this match the object class '{yolo_class}'? If not, what do you think it actually is?
Provide detailed observations."""
        a1 = self.query(image, q1)
        print(f"  Q1 (Material & Cross-check): {a1}")
        
        # Enhanced material detection
        material = 'unknown'
        material_keywords = {
            'plastic': ['plastic', 'polymer', 'synthetic'],
            'metal': ['metal', 'metallic', 'steel', 'aluminum', 'iron'],
            'ceramic': ['ceramic', 'porcelain', 'clay'],
            'glass': ['glass', 'transparent', 'clear'],
            'wood': ['wood', 'wooden', 'timber'],
            'paper': ['paper', 'cardboard', 'card'],
            'fabric': ['fabric', 'cloth', 'textile'],
            'rubber': ['rubber', 'elastic']
        }
        
        for mat, keywords in material_keywords.items():
            if any(kw in a1 for kw in keywords):
                material = mat
                break
        
        # Weight estimation
        if any(w in a1 for w in ['light', 'lightweight', 'thin', 'small']):
            weight_cat, weight_g = 'light', 50
        elif any(w in a1 for w in ['heavy', 'thick', 'dense', 'large']):
            weight_cat, weight_g = 'heavy', 700
        else:
            weight_cat, weight_g = 'medium', 250
        
        # Cross-validation: Check if VLM agrees with YOLO
        vlm_disagrees = any(w in a1 for w in ['not', 'incorrect', 'actually', 'different', 'wrong'])
        if vlm_disagrees:
            print(f"  ⚠ VLM disagrees with YOLO class '{yolo_class}'")
        
        # Query 2: Fragility & Size (use YOLO size as reference)
        q2 = f"""This {material} {yolo_class} appears to be approximately {yolo_size_mm}mm in size:
1. Fragility: Is it fragile? (yes/no) Explain based on material and structure.
2. Size verification: Does {yolo_size_mm}mm seem accurate? Provide your estimate in mm.
3. Shape: cylindrical/rectangular/spherical/irregular?
4. Graspability: Good grip points? Handles, edges, or smooth surfaces?
Be specific."""
        a2 = self.query(image, q2)
        print(f"  Q2 (Fragility & Size Check): {a2}")
        
        # Fragility detection
        fragile_indicators = ['fragile', 'break', 'delicate', 'brittle', 'yes']
        robust_indicators = ['sturdy', 'durable', 'strong', 'solid', 'no']
        
        if any(ind in a2 for ind in fragile_indicators) and material in ['glass', 'ceramic']:
            fragility = 'fragile'
        elif any(ind in a2 for ind in robust_indicators):
            fragility = 'not_fragile'
        else:
            fragility = 'potentially_fragile'
        
        # Size: Use YOLO size as baseline, adjust if VLM disagrees
        import re
        vlm_numbers = re.findall(r'\d+', a2)
        if vlm_numbers:
            vlm_size = int(vlm_numbers[0])
            # If VLM and YOLO differ significantly, average them
            if abs(vlm_size - yolo_size_mm) > 30:
                size_mm = (vlm_size + yolo_size_mm) // 2
                print(f"  📏 Size adjusted: YOLO {yolo_size_mm}mm + VLM {vlm_size}mm → {size_mm}mm")
            else:
                size_mm = yolo_size_mm
        else:
            size_mm = yolo_size_mm
        
        # Query 3: Spatial (use YOLO spatial data as hint)
        yolo_position = yolo_spatial.get('position', 'unknown')
        q3 = f"""Analyze the spatial context (YOLO detected it in {yolo_position} of frame):
1. Reachability: Is this {yolo_class} within robot arm reach (~400mm)? (yes/no)
2. Path blocking: Is it blocking a movement path? (yes/no)
3. Other objects: Are there other objects nearby? Describe their positions.
4. Accessibility: Can it be accessed without disturbing others?
Provide spatial details."""
        a3 = self.query(image, q3)
        print(f"  Q3 (Spatial Context): {a3}")
        
        # Spatial analysis
        reachable = any(w in a3 for w in ['yes', 'close', 'reachable', 'near', 'within', 'accessible'])
        blocking = any(w in a3 for w in ['blocking', 'block', 'obstruct', 'path', 'way'])
        multiple_objects = any(w in a3 for w in ['other', 'multiple', 'several', 'nearby', 'surrounding'])
        
        # Enhanced Decision Logic with YOLO data
        graspable = (self.robot.gripper_min_mm <= size_mm <= self.robot.gripper_max_mm)
        weight_ok = weight_g <= self.robot.payload_g
        
        # Use YOLO confidence in decision
        high_confidence_yolo = yolo_confidence > 0.75
        
        if fragility == 'fragile':
            action = ManipulationAction.AVOID
            reason = f"SAFETY: Fragile {material} {yolo_class} - high risk of damage"
        elif vlm_disagrees and not high_confidence_yolo:
            action = ManipulationAction.STOP
            reason = f"UNCERTAIN: VLM and YOLO disagree on object type (YOLO:{yolo_class} @ {yolo_confidence:.2f})"
        elif multiple_objects and blocking:
            action = ManipulationAction.STOP
            reason = f"CAUTION: Multiple objects - {yolo_class} blocking but needs careful planning"
        elif weight_ok and graspable and reachable and fragility == 'not_fragile':
            if multiple_objects:
                reason = f"GRASP: {weight_cat} {material} {yolo_class} ({weight_g}g, {size_mm}mm) - navigate around nearby objects"
            else:
                reason = f"GRASP: {weight_cat} {material} {yolo_class} ({weight_g}g, {size_mm}mm) - clear path"
            action = ManipulationAction.GRASP
        elif not graspable and weight_ok and reachable and fragility == 'not_fragile':
            action = ManipulationAction.PUSH
            reason = f"PUSH: {yolo_class} {size_mm}mm too large for gripper (max {self.robot.gripper_max_mm}mm)"
        elif not weight_ok and reachable and fragility == 'not_fragile':
            action = ManipulationAction.PUSH
            reason = f"PUSH: {yolo_class} ~{weight_g}g exceeds payload ({self.robot.payload_g}g)"
        elif fragility == 'potentially_fragile':
            action = ManipulationAction.AVOID
            reason = f"CAUTION: {material} {yolo_class} - unclear fragility"
        elif not reachable:
            action = ManipulationAction.IGNORE
            reason = f"IGNORE: {yolo_class} beyond reach ({self.robot.reach_mm}mm max)"
        elif not blocking:
            action = ManipulationAction.IGNORE
            reason = f"IGNORE: {yolo_class} not blocking path"
        else:
            action = ManipulationAction.STOP
            reason = f"UNCERTAIN: {yolo_class} conditions unclear"
        
        analysis_time = (time.time() - start) * 1000
        
        print(f"  → Decision: {action.value}")
        print(f"  → Reasoning: {reason}")
        print(f"  → Cross-validation: {'CONFLICT' if vlm_disagrees else 'OK'}")
        print(f"  → Time: {analysis_time:.0f}ms")
        
        # Combined confidence: YOLO + VLM agreement
        if vlm_disagrees:
            combined_conf = 0.50  # Low confidence on disagreement
        else:
            combined_conf = (0.70 * 0.7) + (yolo_confidence * 0.3)  # Weighted average
        
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
            yolo_guided=True,
            yolo_confidence=yolo_confidence
        )
    
    def analyze(self, image: Image.Image, object_class: str = "object",
                yolo_confidence: float = 0.0) -> VLMAnalysis:
        """Enhanced multi-query analysis with detailed prompts"""
        start = time.time()
        obj = object_class
        yolo_guided = yolo_confidence > 0.0
        
        print(f"\n[VLM Analysis #{self.query_count + 1}]")
        if yolo_guided:
            print(f"  YOLO guidance: {obj} @ {yolo_confidence:.2f}")
        
        # Query 1: Comprehensive Material & Physical Properties
        q1 = f"""Analyze this {obj} in the image and describe:
1. Material composition: Is it made of plastic, metal, ceramic, glass, wood, paper, fabric, or rubber?
2. Weight estimation: Does it appear light (under 100g), medium (100-500g), or heavy (over 500g)?
3. Surface texture: Is it smooth, rough, glossy, or matte?
Provide specific details about what you observe."""
        a1 = self.query(image, q1)
        print(f"  Q1 (Material & Weight): {a1}")
        
        # Enhanced material detection
        material = 'unknown'
        material_keywords = {
            'plastic': ['plastic', 'polymer', 'synthetic'],
            'metal': ['metal', 'metallic', 'steel', 'aluminum', 'iron'],
            'ceramic': ['ceramic', 'porcelain', 'clay'],
            'glass': ['glass', 'transparent', 'clear'],
            'wood': ['wood', 'wooden', 'timber'],
            'paper': ['paper', 'cardboard', 'card'],
            'fabric': ['fabric', 'cloth', 'textile'],
            'rubber': ['rubber', 'elastic']
        }
        
        for mat, keywords in material_keywords.items():
            if any(kw in a1 for kw in keywords):
                material = mat
                break
        
        # Enhanced weight estimation
        if any(w in a1 for w in ['light', 'lightweight', 'thin', 'small']):
            weight_cat, weight_g = 'light', 50
        elif any(w in a1 for w in ['heavy', 'thick', 'dense', 'large']):
            weight_cat, weight_g = 'heavy', 700
        else:
            weight_cat, weight_g = 'medium', 250
        
        # Query 2: Fragility, Size & Shape Analysis
        q2 = f"""Examine this {material} {obj} carefully:
1. Fragility: Is it fragile and likely to break easily? Answer yes or no and explain why.
2. Dimensions: Estimate its width/diameter in millimeters (e.g., 20mm, 50mm, 80mm, 150mm).
3. Shape: Describe its shape (cylindrical, rectangular, spherical, irregular).
4. Graspability: Does it have features suitable for grasping (handles, edges, flat surfaces)?
Be specific with measurements and observations."""
        a2 = self.query(image, q2)
        print(f"  Q2 (Fragility & Size): {a2}")
        
        # Enhanced fragility detection
        fragile_indicators = ['fragile', 'break', 'delicate', 'brittle', 'yes']
        robust_indicators = ['sturdy', 'durable', 'strong', 'solid', 'no']
        
        if any(ind in a2 for ind in fragile_indicators) and material in ['glass', 'ceramic']:
            fragility = 'fragile'
        elif any(ind in a2 for ind in robust_indicators):
            fragility = 'not_fragile'
        else:
            fragility = 'potentially_fragile'
        
        # Enhanced size extraction
        try:
            # Extract number from response
            import re
            numbers = re.findall(r'\d+', a2)
            if numbers:
                size_mm = int(numbers[0])
                size_mm = max(10, min(200, size_mm))
            else:
                size_mm = 50
        except:
            size_mm = 50
        
        # Query 3: Spatial Awareness & Multi-Object Scene
        q3 = f"""Analyze the spatial context of this {obj}:
1. Distance: Is this {obj} within arm's reach (close, within 400mm) or far away? Answer yes for close, no for far.
2. Path blocking: Is this {obj} blocking a clear path or movement area? Answer yes or no.
3. Multiple objects: Are there other objects nearby? If yes, describe their positions relative to this {obj}.
4. Accessibility: Can this {obj} be easily accessed without disturbing other objects?
Provide clear spatial details."""
        a3 = self.query(image, q3)
        print(f"  Q3 (Spatial Context): {a3}")
        
        # Enhanced spatial understanding
        reachable = any(w in a3 for w in ['yes', 'close', 'reachable', 'near', 'within', 'accessible'])
        blocking = any(w in a3 for w in ['blocking', 'block', 'obstruct', 'path', 'way'])
        multiple_objects = any(w in a3 for w in ['other', 'multiple', 'several', 'nearby', 'surrounding'])
        
        # Enhanced Decision Logic with Multi-Object Awareness
        graspable = (self.robot.gripper_min_mm <= size_mm <= self.robot.gripper_max_mm)
        weight_ok = weight_g <= self.robot.payload_g
        
        # Priority 1: Safety - Fragile objects
        if fragility == 'fragile':
            action = ManipulationAction.AVOID
            reason = f"SAFETY: Fragile {material} object detected - high risk of damage"
        
        # Priority 2: Multi-object scene - requires caution
        elif multiple_objects and blocking:
            action = ManipulationAction.STOP
            reason = f"CAUTION: Multiple objects detected - {obj} blocking path but requires careful planning"
        
        # Priority 3: Optimal grasp conditions
        elif weight_ok and graspable and reachable and fragility == 'not_fragile':
            if multiple_objects:
                reason = f"GRASP: {weight_cat} {material} ({weight_g}g), {size_mm}mm - accessible but navigate around nearby objects"
            else:
                reason = f"GRASP: {weight_cat} {material} ({weight_g}g), {size_mm}mm - clear grasp path"
            action = ManipulationAction.GRASP
        
        # Priority 4: Push if not graspable but manageable
        elif not graspable and weight_ok and reachable and fragility == 'not_fragile':
            action = ManipulationAction.PUSH
            reason = f"PUSH: {size_mm}mm {material} too large for gripper ({self.robot.gripper_max_mm}mm max) but pushable"
        
        # Priority 5: Too heavy but could push
        elif not weight_ok and reachable and fragility == 'not_fragile':
            action = ManipulationAction.PUSH
            reason = f"PUSH: {weight_cat} {material} (~{weight_g}g) exceeds payload ({self.robot.payload_g}g) but can push if needed"
        
        # Priority 6: Potentially fragile - exercise caution
        elif fragility == 'potentially_fragile':
            action = ManipulationAction.AVOID
            reason = f"CAUTION: {material} {obj} - unclear fragility, recommend avoiding contact"
        
        # Priority 7: Out of reach
        elif not reachable:
            action = ManipulationAction.IGNORE
            reason = f"IGNORE: {obj} beyond robot reach ({self.robot.reach_mm}mm max)"
        
        # Priority 8: Not blocking path
        elif not blocking:
            action = ManipulationAction.IGNORE
            reason = f"IGNORE: {obj} not blocking movement path - no action required"
        
        # Fallback: Uncertain
        else:
            action = ManipulationAction.STOP
            reason = f"UNCERTAIN: {obj} conditions unclear - requires human assessment"
        
        analysis_time = (time.time() - start) * 1000
        
        print(f"  → Decision: {action.value}")
        print(f"  → Reasoning: {reason}")
        print(f"  → Multi-object scene: {multiple_objects}")
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
    """Main inference endpoint with YOLO integration"""
    try:
        data = request.json
        
        # Decode base64 image
        img_base64 = data['image']
        img_bytes = base64.b64decode(img_base64)
        image = Image.open(io.BytesIO(img_bytes)).convert('RGB')
        
        # Check for YOLO data
        yolo_enabled = data.get('yolo_enabled', False)
        
        if yolo_enabled and 'primary_object' in data:
            # YOLO-guided analysis
            primary = data['primary_object']
            object_class = primary['class']
            yolo_confidence = primary['confidence']
            
            # Extract dimension hints from YOLO
            estimated_size = primary.get('estimated_size_mm', {})
            max_dim_mm = estimated_size.get('max_dimension', 0)
            
            print(f"\n[YOLO-Enhanced Analysis]")
            print(f"  YOLO Class: {object_class} @ {yolo_confidence:.2f}")
            print(f"  Estimated Size: ~{max_dim_mm}mm")
            
            # Run enhanced VLM analysis
            analysis = vlm.analyze_with_yolo(
                image, 
                yolo_class=object_class,
                yolo_confidence=yolo_confidence,
                yolo_size_mm=max_dim_mm,
                yolo_spatial=primary.get('spatial', {})
            )
        else:
            # Standard VLM-only analysis
            object_class = data.get('object_class', 'object')
            yolo_confidence = 0.0
            
            print(f"\n[VLM-Only Analysis]")
            analysis = vlm.analyze(image, object_class, yolo_confidence)
        
        # Return as JSON
        return jsonify(asdict(analysis))
        
    except Exception as e:
        print(f"ERROR in /analyze: {e}")
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
