#!/usr/bin/env python3
"""
PRODUCTION: Moondream2 + YOLO11 Robot Manipulation Pipeline
Optimized for Jetson Xavier NX 8GB with Detection-Guided Prompting
"""
import torch
import gc
import os
import json
from dataclasses import dataclass
from typing import Dict, List, Tuple

# Memory optimization
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
class Detection:
    class_name: str
    bbox: List[int]  # [x1, y1, x2, y2]
    confidence: float
    center: Tuple[int, int]
    size_px: Tuple[int, int]

class MoondreamRobotPerception:
    def __init__(self, robot_spec: RobotSpec):
        self.robot = robot_spec
        self.model = None
        self.tokenizer = None
        
    def load_model(self):
        """Load Moondream2 with strict memory optimization"""
        from transformers import AutoModelForCausalLM, AutoTokenizer
        
        print("Loading Moondream2 (memory-optimized)...")
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
        self.tokenizer = AutoTokenizer.from_pretrained(model_id)
        print("✅ Moondream2 loaded")
        
    def estimate_material(self, image, detection: Detection) -> str:
        """Estimate object material from visual appearance"""
        cleanup()
        prompt = f"The {detection.class_name} in this image is made of what material? Answer in one word: plastic, metal, ceramic, glass, or wood."
        result = self.model.query(image, prompt)
        return result['answer'].strip().lower()
    
    def estimate_weight_category(self, image, detection: Detection, material: str) -> str:
        """Estimate weight: light, medium, heavy"""
        cleanup()
        prompt = f"This {material} {detection.class_name} is: light (<100g), medium (100-500g), or heavy (>500g)? One word answer."
        result = self.model.query(image, prompt)
        answer = result['answer'].strip().lower()
        
        # Parse response
        if 'light' in answer:
            return 'light'
        elif 'medium' in answer:
            return 'medium'
        elif 'heavy' in answer:
            return 'heavy'
        else:
            return 'medium'  # Default conservative
    
    def assess_fragility(self, image, detection: Detection, material: str) -> bool:
        """Check if object is fragile"""
        cleanup()
        prompt = f"Is this {material} {detection.class_name} fragile (easily breaks)? Answer yes or no."
        result = self.model.query(image, prompt)
        return 'yes' in result['answer'].lower()
    
    def check_graspability(self, image, detection: Detection) -> bool:
        """Check if object size fits gripper"""
        cleanup()
        prompt = f"Is this {detection.class_name} small enough to fit in a gripper that opens {self.robot.gripper_min_mm}-{self.robot.gripper_max_mm}mm? Yes or no."
        result = self.model.query(image, prompt)
        return 'yes' in result['answer'].lower()
    
    def detection_guided_analysis(self, image, detection: Detection) -> Dict:
        """
        Full VLM analysis using YOLO detection to guide prompts
        Returns structured decision with reasoning
        """
        print(f"\n[VLM Analysis: {detection.class_name}]")
        
        # Step 1: Material
        material = self.estimate_material(image, detection)
        print(f"  Material: {material}")
        
        # Step 2: Weight
        weight_cat = self.estimate_weight_category(image, detection, material)
        print(f"  Weight: {weight_cat}")
        
        # Step 3: Fragility
        is_fragile = self.assess_fragility(image, detection, material)
        print(f"  Fragile: {is_fragile}")
        
        # Step 4: Graspability
        can_grasp_size = self.check_graspability(image, detection)
        print(f"  Fits gripper: {can_grasp_size}")
        
        # Decision Logic
        weight_ok = weight_cat in ['light', 'medium']
        
        if is_fragile:
            action = "AVOID"
            reason = "Object is fragile"
        elif not can_grasp_size:
            action = "PUSH" if weight_cat != 'heavy' else "IGNORE"
            reason = "Too large for gripper" if weight_cat != 'heavy' else "Too heavy and large"
        elif not weight_ok:
            action = "PUSH"
            reason = f"Too heavy ({weight_cat}) for payload"
        elif can_grasp_size and weight_ok:
            action = "GRASP"
            reason = f"Graspable {material} {weight_cat} object"
        else:
            action = "IGNORE"
            reason = "Uncertain conditions"
        
        return {
            "object": detection.class_name,
            "material": material,
            "weight_category": weight_cat,
            "fragile": is_fragile,
            "graspable_size": can_grasp_size,
            "action": action,
            "reasoning": reason,
            "confidence": detection.confidence
        }

# Demo usage
if __name__ == "__main__":
    from PIL import Image, ImageDraw
    import time
    
    print("=" * 70)
    print("Moondream2 Robot Perception Pipeline - PRODUCTION TEST")
    print("=" * 70)
    
    # Initialize
    robot = RobotSpec()
    perception = MoondreamRobotPerception(robot)
    perception.load_model()
    
    # Create test image
    img = Image.new('RGB', (320, 240), color=(220, 220, 220))
    draw = ImageDraw.Draw(img)
    draw.ellipse([125, 100, 195, 120], fill=(180, 140, 100))
    draw.rectangle([125, 110, 195, 175], fill=(180, 140, 100))
    
    # Simulate YOLO detection
    yolo_det = Detection(
        class_name="cup",
        bbox=[125, 100, 195, 175],
        confidence=0.95,
        center=(160, 137),
        size_px=(70, 75)
    )
    
    print(f"\n[YOLO11 Detection]")
    print(f"  Class: {yolo_det.class_name}")
    print(f"  Confidence: {yolo_det.confidence:.2f}")
    print(f"  BBox: {yolo_det.bbox}")
    
    # Run VLM analysis
    start = time.time()
    result = perception.detection_guided_analysis(img, yolo_det)
    analysis_time = time.time() - start
    
    print(f"\n[Decision]")
    print(f"  Action: {result['action']}")
    print(f"  Reasoning: {result['reasoning']}")
    print(f"  Analysis time: {analysis_time*1000:.0f}ms")
    
    print(f"\n[Full Result JSON]")
    print(json.dumps(result, indent=2))
    
    print("\n" + "=" * 70)
    print("✅ Pipeline ready for integration with YOLO11 + ROS2!")
    print("=" * 70)
    print("\nNext steps:")
    print("  1. Integrate YOLO11 with TensorRT (60 FPS)")
    print("  2. Create ROS2 nodes for perception + planning")
    print("  3. Implement BoT-SORT tracking")
    print("  4. Add decision fusion logic")
    print("  5. Test on actual TurtleBot3 + Jetson Xavier NX")
    print("=" * 70)
