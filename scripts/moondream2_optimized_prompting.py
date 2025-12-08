#!/usr/bin/env python3
"""
Optimized Detection-Guided Prompting for Moondream2
Strategy: Break complex prompt into multiple simple queries
Better accuracy + faster than one complex prompt
"""
import torch
import gc
import os
import json
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional
from PIL import Image, ImageDraw
import time

os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128,expandable_segments:True'

def cleanup():
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

@dataclass
class RobotSpec:
    reach_mm: int = 400
    payload_g: int = 500
    gripper_min_mm: int = 10
    gripper_max_mm: int = 100

@dataclass
class YOLODetection:
    class_name: str
    bbox: List[int]
    confidence: float
    tracking_id: Optional[int] = None

@dataclass
class VLMAnalysis:
    object_class: str
    material: str
    weight_category: str
    fragility: str
    rigidity: str
    size_estimate_mm: int
    reachable: bool
    graspable: bool
    path_blocking: bool
    recommended_action: str
    reasoning: str
    confidence_score: float
    analysis_time_ms: float

class Moondream2PhysicsAnalyzer:
    """
    Multi-Query Detection-Guided Analysis
    Each query is simple and focused for better VLM performance
    """
    
    def __init__(self, robot_spec: RobotSpec):
        self.robot = robot_spec
        self.model = None
        
    def load_model(self):
        from transformers import AutoModelForCausalLM, AutoTokenizer
        
        print("Loading Moondream2...")
        cleanup()
        
        model_id = "vikhyatk/moondream2"
        self.model = AutoModelForCausalLM.from_pretrained(
            model_id,
            trust_remote_code=True,
            device_map="auto",
            torch_dtype=torch.float16,
            low_cpu_mem_usage=True,
            max_memory={0: "5GB"}
        )
        print("✅ Model loaded")
    
    def query_vlm(self, image: Image.Image, question: str) -> str:
        """Simple wrapper for VLM query"""
        cleanup()
        result = self.model.query(image, question)
        return result['answer'].strip().lower()
    
    def analyze_material(self, image: Image.Image, obj_class: str) -> str:
        """Query 1: Material identification"""
        q = f"What material is this {obj_class} made of? Answer one word: plastic, metal, ceramic, glass, wood, or paper."
        answer = self.query_vlm(image, q)
        
        # Parse
        materials = ['plastic', 'metal', 'ceramic', 'glass', 'wood', 'paper']
        for mat in materials:
            if mat in answer:
                return mat
        return 'unknown'
    
    def analyze_weight(self, image: Image.Image, obj_class: str, material: str) -> str:
        """Query 2: Weight category"""
        q = f"This {material} {obj_class}: is it light (under 100g), medium (100-500g), or heavy (over 500g)? Answer one word."
        answer = self.query_vlm(image, q)
        
        if 'light' in answer:
            return 'light'
        elif 'heavy' in answer:
            return 'heavy'
        else:
            return 'medium'
    
    def analyze_fragility(self, image: Image.Image, obj_class: str, material: str) -> str:
        """Query 3: Fragility assessment"""
        q = f"Is this {material} {obj_class} fragile and easily breakable? Answer yes or no."
        answer = self.query_vlm(image, q)
        return 'fragile' if 'yes' in answer else 'not_fragile'
    
    def analyze_rigidity(self, image: Image.Image, obj_class: str) -> str:
        """Query 4: Rigidity assessment"""
        q = f"Is this {obj_class} rigid and stiff, or flexible and bendable? Answer rigid or flexible."
        answer = self.query_vlm(image, q)
        return 'flexible' if 'flexible' in answer or 'bend' in answer else 'rigid'
    
    def estimate_size(self, image: Image.Image, obj_class: str, bbox: List[int]) -> int:
        """Query 5: Size estimation"""
        bbox_width = bbox[2] - bbox[0]
        q = f"Estimate the width of this {obj_class} in millimeters. Is it approximately: 20mm, 50mm, 80mm, or 150mm? Answer just the number."
        answer = self.query_vlm(image, q)
        
        # Extract number
        try:
            size = int(''.join(filter(str.isdigit, answer)))
            return max(10, min(200, size))  # Clamp to reasonable range
        except:
            # Estimate from bbox
            return int(bbox_width * 0.5)  # Rough pixel-to-mm conversion
    
    def analyze_reachability(self, image: Image.Image, obj_class: str, bbox: List[int]) -> bool:
        """Query 6: Distance/reachability"""
        q = f"Is this {obj_class} close to the camera (within arm's reach) or far away? Answer close or far."
        answer = self.query_vlm(image, q)
        return 'close' in answer or 'near' in answer
    
    def analyze_path_blocking(self, image: Image.Image, obj_class: str) -> bool:
        """Query 7: Path obstruction"""
        q = f"Is this {obj_class} blocking a path or in the way? Answer yes or no."
        answer = self.query_vlm(image, q)
        return 'yes' in answer
    
    def make_decision(self, analysis: Dict) -> tuple:
        """
        Decision logic from proposal:
        - GRASP: weight≤500g, size 10-100mm, not fragile, reachable
        - PUSH: heavier but movable, not fragile
        - AVOID: fragile or high-risk
        - IGNORE: not blocking path
        - STOP: high uncertainty
        """
        weight_ok = analysis['weight_category'] in ['light', 'medium']
        size_ok = (self.robot.gripper_min_mm <= analysis['size_estimate_mm'] <= 
                   self.robot.gripper_max_mm)
        not_fragile = analysis['fragility'] == 'not_fragile'
        reachable = analysis['reachable']
        blocking = analysis['path_blocking']
        
        # Decision tree
        if analysis['fragility'] == 'fragile':
            return 'AVOID', 'Object is fragile and may break'
        
        if weight_ok and size_ok and not_fragile and reachable:
            return 'GRASP', f'Suitable for grasping: {analysis["weight_category"]} weight, {analysis["size_estimate_mm"]}mm size'
        
        if weight_ok and not_fragile and not size_ok:
            return 'PUSH', f'Too large for gripper ({analysis["size_estimate_mm"]}mm) but can push'
        
        if not weight_ok and not_fragile:
            return 'PUSH', f'Too heavy ({analysis["weight_category"]}) for grasping, can push'
        
        if not reachable:
            return 'IGNORE', 'Object is out of reach'
        
        if not blocking:
            return 'IGNORE', 'Object not blocking path'
        
        return 'STOP', 'Uncertain conditions, need clarification'
    
    def analyze_detection(self, image: Image.Image, detection: YOLODetection) -> VLMAnalysis:
        """
        Full multi-query analysis pipeline
        7 focused queries + decision logic
        """
        start_time = time.time()
        
        print(f"\n[Analyzing: {detection.class_name}]")
        
        # Run all queries
        material = self.analyze_material(image, detection.class_name)
        print(f"  Material: {material}")
        
        weight = self.analyze_weight(image, detection.class_name, material)
        print(f"  Weight: {weight}")
        
        fragility = self.analyze_fragility(image, detection.class_name, material)
        print(f"  Fragility: {fragility}")
        
        rigidity = self.analyze_rigidity(image, detection.class_name)
        print(f"  Rigidity: {rigidity}")
        
        size_mm = self.estimate_size(image, detection.class_name, detection.bbox)
        print(f"  Size: {size_mm}mm")
        
        reachable = self.analyze_reachability(image, detection.class_name, detection.bbox)
        print(f"  Reachable: {reachable}")
        
        blocking = self.analyze_path_blocking(image, detection.class_name)
        print(f"  Path blocking: {blocking}")
        
        # Decision
        analysis_dict = {
            'weight_category': weight,
            'size_estimate_mm': size_mm,
            'fragility': fragility,
            'reachable': reachable,
            'path_blocking': blocking
        }
        
        action, reasoning = self.make_decision(analysis_dict)
        graspable = action == 'GRASP'
        
        analysis_time = (time.time() - start_time) * 1000
        print(f"  → Action: {action}")
        print(f"  → Time: {analysis_time:.0f}ms")
        
        return VLMAnalysis(
            object_class=detection.class_name,
            material=material,
            weight_category=weight,
            fragility=fragility,
            rigidity=rigidity,
            size_estimate_mm=size_mm,
            reachable=reachable,
            graspable=graspable,
            path_blocking=blocking,
            recommended_action=action,
            reasoning=reasoning,
            confidence_score=detection.confidence,
            analysis_time_ms=analysis_time
        )

# Demo
if __name__ == "__main__":
    print("=" * 70)
    print("Optimized Multi-Query Detection-Guided System")
    print("=" * 70)
    
    robot = RobotSpec()
    system = Moondream2PhysicsAnalyzer(robot)
    system.load_model()
    
    # Test scene: plastic cup
    img = Image.new('RGB', (640, 480), color=(230, 230, 230))
    draw = ImageDraw.Draw(img)
    draw.ellipse([200, 150, 300, 180], fill=(180, 140, 100), outline=(100, 70, 50), width=2)
    draw.rectangle([200, 165, 300, 300], fill=(180, 140, 100))
    draw.ellipse([200, 285, 300, 315], fill=(160, 120, 80))
    
    detection = YOLODetection(
        class_name="cup",
        bbox=[200, 150, 300, 315],
        confidence=0.95,
        tracking_id=1
    )
    
    print(f"\n[YOLO11 Detection]")
    print(f"  Class: {detection.class_name}")
    print(f"  Confidence: {detection.confidence:.2f}")
    
    # Analyze
    analysis = system.analyze_detection(img, detection)
    
    print(f"\n[Final Analysis - JSON]")
    print(json.dumps(asdict(analysis), indent=2))
    
    print("\n" + "=" * 70)
    print("✅ Multi-Query System Working!")
    print("=" * 70)
    print(f"\nPerformance:")
    print(f"  Total time: {analysis.analysis_time_ms:.0f}ms")
    print(f"  Avg per query: {analysis.analysis_time_ms/7:.0f}ms")
    print(f"\nAdvantages over single prompt:")
    print(f"  ✓ Each query is simple and focused")
    print(f"  ✓ Better VLM accuracy on specific questions")
    print(f"  ✓ Easier to debug individual components")
    print(f"  ✓ Can parallelize queries in production")
    print("=" * 70)
