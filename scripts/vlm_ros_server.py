#!/usr/bin/env python3
"""
Enhanced VLM Inference Server with Comprehensive Spatial Reasoning
===================================================================
Features:
- Multi-object spatial relationship understanding
- Physical interaction analysis (stacking, supporting, blocking)
- Zero-shot and Chain-of-Thought (CoT) prompting
- Step-by-step action sequence generation
- Manipulation planning with contingencies
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
from typing import List, Dict, Optional
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
    NAVIGATE_AROUND = "NAVIGATE_AROUND"
    IGNORE = "IGNORE"
    STOP = "STOP"
    REORDER = "REORDER"  # NEW: For multi-object manipulation sequences

@dataclass
class RobotSpec:
    reach_mm: int = 400
    payload_g: int = 500
    gripper_min_mm: int = 10
    gripper_max_mm: int = 100
    base_speed_mps: float = 0.2  # TurtleBot3 max speed

@dataclass
class ActionStep:
    """Individual action in a sequence"""
    step_number: int
    action_type: str
    target_object: str
    description: str
    estimated_time_s: float
    preconditions: List[str]
    expected_outcome: str
    failure_handling: str

@dataclass
class VLMAnalysis:
    # Object properties
    material: str
    weight_category: str
    weight_grams: int
    fragility: str
    size_mm: int
    shape: str  # NEW
    
    # Spatial understanding
    reachable: bool
    path_blocking: bool
    distance_category: str  # NEW: close/medium/far
    orientation: str  # NEW: upright/tilted/lying
    
    # Multi-object context (NEW)
    scene_type: str  # single/clustered/scattered/stacked
    object_relationships: List[str]  # supporting/blocking/adjacent/stacked_on
    interaction_risks: List[str]  # collision/cascade/destabilize
    
    # Decision
    graspable: bool
    action: str
    reasoning: str
    confidence: float
    
    # Action sequences (NEW)
    action_sequence: List[Dict]  # Step-by-step execution plan
    alternative_sequences: List[List[Dict]]  # Backup plans
    estimated_total_time_s: float
    
    # Metadata
    analysis_time_ms: float
    reasoning_method: str  # zero_shot/cot
    yolo_guided: bool = False
    yolo_confidence: float = 0.0

class EnhancedMoondream2VLM:
    """Enhanced VLM with comprehensive spatial reasoning and action planning"""
    
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
        print("✅ Enhanced VLM loaded with spatial reasoning capabilities")
    
    def query(self, image: Image.Image, question: str) -> str:
        cleanup()
        self.query_count += 1
        result = self.model.query(image, question)
        return result['answer'].strip().lower()
    
    def analyze_comprehensive(self, image: Image.Image, 
                            object_class: str = "object",
                            yolo_confidence: float = 0.0,
                            use_cot: bool = True) -> VLMAnalysis:
        """
        Comprehensive analysis with spatial relationships and action planning.
        Uses Chain-of-Thought (CoT) prompting for complex reasoning.
        """
        start = time.time()
        obj = object_class
        yolo_guided = yolo_confidence > 0.0
        
        print(f"\n{'='*80}")
        print(f"[Enhanced VLM Analysis #{self.query_count + 1}]")
        print(f"Object: {obj} | YOLO: {yolo_confidence:.2f} | Method: {'CoT' if use_cot else 'Zero-Shot'}")
        print(f"{'='*80}")
        
        # ========== QUERY 1: Object Properties (Zero-Shot) ==========
        q1 = f"""Examine this {obj} carefully and provide detailed observations:

MATERIAL: What is it made of? Choose from: plastic, metal, ceramic, glass, wood, paper, fabric, rubber, or composite.
Look at: color, texture, surface finish, reflectivity, opacity.

WEIGHT: Estimate the weight category based on size and material:
- Light: under 100g (small plastic items, paper, empty containers)
- Medium: 100-500g (filled cups, small books, standard bottles)
- Heavy: over 500g (large metal objects, full containers, thick ceramic)

SHAPE: Describe the geometric form:
- Cylindrical (bottles, cans, tubes)
- Rectangular (boxes, books, phones)
- Spherical (balls, round objects)
- Irregular (complex shapes, organic forms)

TEXTURE: smooth/rough/glossy/matte/ridged?

Provide specific observations for each category."""

        a1 = self.query(image, q1)
        print(f"\n[Q1] Physical Properties:")
        print(f"  {a1[:200]}...")
        
        # Parse material
        materials = {
            'plastic': ['plastic', 'polymer', 'synthetic', 'acrylic'],
            'metal': ['metal', 'metallic', 'steel', 'aluminum', 'brass', 'iron'],
            'ceramic': ['ceramic', 'porcelain', 'clay', 'pottery'],
            'glass': ['glass', 'transparent', 'clear', 'translucent'],
            'wood': ['wood', 'wooden', 'timber', 'bamboo'],
            'paper': ['paper', 'cardboard', 'card', 'paperboard'],
            'fabric': ['fabric', 'cloth', 'textile', 'cotton'],
            'rubber': ['rubber', 'elastic', 'silicone']
        }
        
        material = 'unknown'
        for mat, keywords in materials.items():
            if any(kw in a1 for kw in keywords):
                material = mat
                break
        
        # Parse weight
        if any(w in a1 for w in ['light', 'lightweight', 'thin', 'small', 'empty']):
            weight_cat, weight_g = 'light', 50
        elif any(w in a1 for w in ['heavy', 'thick', 'dense', 'large', 'full', 'solid']):
            weight_cat, weight_g = 'heavy', 700
        else:
            weight_cat, weight_g = 'medium', 250
        
        # Parse shape
        shapes = ['cylindrical', 'rectangular', 'spherical', 'irregular']
        shape = next((s for s in shapes if s in a1), 'irregular')
        
        # ========== QUERY 2: Fragility & Dimensions (Zero-Shot) ==========
        q2 = f"""Analyze the structural properties of this {material} {obj}:

FRAGILITY ASSESSMENT:
Is this object fragile? Consider:
- Material brittleness ({material} can be fragile if: glass, ceramic, thin plastic)
- Structural weaknesses (thin walls, delicate parts, hollow sections)
- Breakage risk (would it shatter/crack/dent if dropped or squeezed?)

Answer: YES (fragile) or NO (durable), then explain your reasoning.

SIZE ESTIMATION:
Estimate dimensions in millimeters:
- Width/Diameter: ? mm (compare to common objects: credit card 85mm, smartphone 150mm, soda can 65mm)
- Height: ? mm
- Provide your best estimate for the largest dimension.

GRASPABILITY:
- Does it have good grip points? (handles, edges, flat surfaces, ridges)
- Is the surface slippery or textured?
- Are there fragile parts that shouldn't be gripped?

Be specific with measurements and reasoning."""

        a2 = self.query(image, q2)
        print(f"\n[Q2] Structural Analysis:")
        print(f"  {a2[:200]}...")
        
        # Parse fragility
        fragile_indicators = ['fragile', 'break', 'delicate', 'brittle', 'crack', 'shatter', 'yes']
        robust_indicators = ['sturdy', 'durable', 'strong', 'solid', 'robust', 'no']
        
        if any(ind in a2 for ind in fragile_indicators) or material in ['glass', 'ceramic']:
            fragility = 'fragile'
        elif any(ind in a2 for ind in robust_indicators):
            fragility = 'not_fragile'
        else:
            fragility = 'potentially_fragile'
        
        # Parse size
        import re
        numbers = re.findall(r'\d+', a2)
        if numbers:
            size_mm = max([int(n) for n in numbers])
            size_mm = max(10, min(250, size_mm))
        else:
            size_mm = 60
        
        # ========== QUERY 3: Spatial Context & Relationships (CoT) ==========
        if use_cot:
            q3 = f"""Let's think step-by-step about the spatial context of this {obj}.

STEP 1 - OBJECT LOCATION:
Where is this {obj} positioned in the image?
- Foreground (close, large in frame) or Background (far, small in frame)?
- Left, center, or right of the frame?
- Based on size in image, estimate distance: close (<300mm), medium (300-600mm), or far (>600mm)?

STEP 2 - SURROUNDING OBJECTS:
Look carefully at the entire scene:
- How many distinct objects can you see?
- List each object you can identify (even if partial)
- Describe the position of each relative to the {obj}: left/right/behind/in front/above/below

STEP 3 - PHYSICAL RELATIONSHIPS:
Analyze how objects interact physically:
- Is anything touching the {obj}?
- Is the {obj} supporting another object? (something resting on top)
- Is the {obj} being supported by something? (resting on a surface/object)
- Are objects stacked vertically?
- Are objects clustered together or spread apart?

STEP 4 - PATH ANALYSIS:
Imagine a robot needs to reach this {obj}:
- Is there a clear straight path to it?
- Would moving this {obj} disturb other objects?
- If the {obj} were removed, would anything fall or become unstable?
- Are there objects blocking access to it?

STEP 5 - RISK ASSESSMENT:
What could go wrong during manipulation?
- Collision risk with nearby objects?
- Domino effect (moving one causes others to move)?
- Instability (might tip over or roll)?

Provide detailed reasoning for each step."""
        else:
            # Zero-shot version
            q3 = f"""Analyze the spatial context of this {obj}:

Is it within robot reach (~400mm)? Yes or No.
Is it blocking a movement path? Yes or No.
Are there other objects nearby? If yes, list them and their positions.
Describe any physical interactions (touching, stacking, supporting).
Can it be safely manipulated without affecting other objects?

Provide clear spatial details."""
        
        a3 = self.query(image, q3)
        print(f"\n[Q3] Spatial & Relational Reasoning:")
        print(f"  {a3[:300]}...")
        
        # Parse spatial information
        reachable = any(w in a3 for w in ['yes', 'close', 'foreground', 'near', 'reachable', 'within'])
        blocking = any(w in a3 for w in ['blocking', 'block', 'obstruct', 'path'])
        
        if 'close' in a3 or 'foreground' in a3:
            distance_category = 'close'
        elif 'far' in a3 or 'background' in a3:
            distance_category = 'far'
        else:
            distance_category = 'medium'
        
        # Parse orientation
        if 'upright' in a3 or 'standing' in a3 or 'vertical' in a3:
            orientation = 'upright'
        elif 'lying' in a3 or 'flat' in a3 or 'horizontal' in a3:
            orientation = 'lying'
        else:
            orientation = 'tilted'
        
        # ========== QUERY 4: Multi-Object Interaction Analysis (CoT) ==========
        q4 = f"""Analyze the object interactions and relationships in this scene:

OBJECT COUNT: How many objects total in the scene?

INTERACTION TYPES:
For each nearby object, describe:
1. SUPPORTING: Is object A supporting (underneath) object B?
2. STACKED: Are objects vertically arranged on top of each other?
3. ADJACENT: Are objects side-by-side touching?
4. SEPARATE: Are objects near but not touching?

MANIPULATION CONSEQUENCES:
If the {obj} were grasped and lifted:
- Which objects would be affected?
- Would anything fall, tip, or become unstable?
- Would removing it create a gap or opening?

SCENE CLASSIFICATION:
- Single object scene (one object, isolated)
- Clustered scene (objects grouped together)
- Stacked scene (objects on top of each other)
- Scattered scene (objects spread out, not touching)

Describe the scene type and all object relationships."""

        a4 = self.query(image, q4)
        print(f"\n[Q4] Interaction Analysis:")
        print(f"  {a4[:300]}...")
        
        # Parse scene type and relationships
        if 'single' in a4 or 'one object' in a4 or 'isolated' in a4:
            scene_type = 'single'
        elif 'stack' in a4 or 'stacked' in a4 or 'on top' in a4:
            scene_type = 'stacked'
        elif 'cluster' in a4 or 'group' in a4 or 'together' in a4:
            scene_type = 'clustered'
        else:
            scene_type = 'scattered'
        
        # Extract relationships
        relationships = []
        if 'support' in a4:
            relationships.append('supporting_relationship')
        if 'stack' in a4 or 'on top' in a4:
            relationships.append('stacked_vertically')
        if 'touch' in a4 or 'adjacent' in a4:
            relationships.append('adjacent_objects')
        if 'separate' in a4 or 'apart' in a4:
            relationships.append('separated')
        
        # Extract risks
        interaction_risks = []
        if 'fall' in a4 or 'tip' in a4:
            interaction_risks.append('instability_risk')
        if 'collision' in a4 or 'hit' in a4:
            interaction_risks.append('collision_risk')
        if 'disturb' in a4 or 'affect' in a4:
            interaction_risks.append('cascade_effect')
        
        # ========== QUERY 5: Navigation Strategy (CoT) ==========
        q5 = f"""Think step-by-step about how a mobile robot should navigate this scene:

STEP 1 - CURRENT OBSTACLE ASSESSMENT:
- Is the {obj} directly in a likely path?
- How much space is around it?
- Can the robot pass by without touching it?

STEP 2 - NAVIGATION OPTIONS:
Option A: Navigate around (detour left or right)
- Which side has more clearance?
- Estimated detour distance?
- Any obstacles on the detour path?

Option B: Remove/move the {obj} first
- Is this object safe to manipulate?
- Would moving it clear the path effectively?
- Are there risks in moving it?

Option C: Ignore and proceed
- Can the robot safely pass without interaction?

STEP 3 - RECOMMEND STRATEGY:
Based on the scene, which option is best? Explain why.
If multiple objects, what order should they be handled?

Provide detailed navigation reasoning."""

        a5 = self.query(image, q5)
        print(f"\n[Q5] Navigation Strategy:")
        print(f"  {a5[:300]}...")
        
        # ========== DECISION LOGIC & ACTION PLANNING ==========
        graspable = (self.robot.gripper_min_mm <= size_mm <= self.robot.gripper_max_mm)
        weight_ok = weight_g <= self.robot.payload_g
        
        # Decision tree with comprehensive reasoning
        action_sequence = []
        alternative_sequences = []
        
        if fragility == 'fragile':
            action = ManipulationAction.AVOID
            reasoning = (
                f"⚠️ SAFETY CRITICAL: {material} {obj} identified as FRAGILE. "
                f"Risk assessment: high breakage probability. "
                f"Scene context: {scene_type} arrangement with {len(relationships)} object relationships. "
                f"Interaction risks: {', '.join(interaction_risks) if interaction_risks else 'none detected'}. "
                f"Decision: Avoid all physical contact. Recommended: navigate around with wide berth."
            )
            
            # Action sequence for avoiding fragile object
            action_sequence = [
                {
                    'step': 1,
                    'action': 'STOP',
                    'description': 'Halt all motion, assess surroundings',
                    'duration_s': 0.5,
                    'preconditions': [],
                    'outcome': 'Safe stop established'
                },
                {
                    'step': 2,
                    'action': 'SCAN_ENVIRONMENT',
                    'description': 'Update costmap with fragile object zone (400mm safety radius)',
                    'duration_s': 1.0,
                    'preconditions': ['Safe stop'],
                    'outcome': 'Costmap updated with high-penalty zone'
                },
                {
                    'step': 3,
                    'action': 'PLAN_DETOUR',
                    'description': f'Calculate alternate path avoiding {obj} with 400mm+ clearance',
                    'duration_s': 1.5,
                    'preconditions': ['Costmap updated'],
                    'outcome': 'Safe detour path computed'
                },
                {
                    'step': 4,
                    'action': 'EXECUTE_NAVIGATION',
                    'description': f'Navigate around {obj} at reduced speed (50% normal)',
                    'duration_s': 8.0,
                    'preconditions': ['Valid detour path'],
                    'outcome': 'Safely bypassed fragile obstacle'
                },
                {
                    'step': 5,
                    'action': 'VERIFY_CLEARANCE',
                    'description': 'Confirm obstacle cleared, resume normal operation',
                    'duration_s': 0.5,
                    'preconditions': ['Navigation complete'],
                    'outcome': 'Path clear, mission resumed'
                }
            ]
            
            # Alternative: Request human assistance
            alternative_sequences.append([
                {
                    'step': 1,
                    'action': 'REQUEST_HUMAN_ASSISTANCE',
                    'description': f'Alert operator: Fragile {material} {obj} blocking path',
                    'duration_s': 0.5,
                    'outcome': 'Human notified, awaiting response'
                }
            ])
            
        elif scene_type == 'stacked' and blocking:
            action = ManipulationAction.REORDER
            reasoning = (
                f"🔄 COMPLEX SCENE: {scene_type} arrangement detected. "
                f"Objects have physical dependencies: {', '.join(relationships)}. "
                f"Target {obj}: {material}, {weight_cat} ({weight_g}g), {size_mm}mm. "
                f"Manipulation strategy: sequential reordering required. "
                f"Priority: maintain stability, prevent cascade failures. "
                f"Estimated sequence: {len(interaction_risks) + 2} steps."
            )
            
            # Multi-step reordering sequence
            action_sequence = [
                {
                    'step': 1,
                    'action': 'ASSESS_STACK',
                    'description': 'Identify stacking order and supporting relationships',
                    'duration_s': 2.0,
                    'preconditions': [],
                    'outcome': 'Stack hierarchy mapped'
                },
                {
                    'step': 2,
                    'action': 'REMOVE_TOP_OBJECTS',
                    'description': 'Carefully remove objects above target (if any)',
                    'duration_s': 5.0,
                    'preconditions': ['Stack mapped'],
                    'outcome': 'Top objects relocated safely'
                },
                {
                    'step': 3,
                    'action': 'GRASP_TARGET',
                    'description': f'Grasp {obj} with {size_mm + 5}mm gripper opening',
                    'duration_s': 3.0,
                    'preconditions': ['Access cleared'],
                    'outcome': f'{obj} secured in gripper'
                },
                {
                    'step': 4,
                    'action': 'RELOCATE',
                    'description': f'Move {obj} to safe location (200mm+ from stack)',
                    'duration_s': 4.0,
                    'preconditions': ['Stable grasp'],
                    'outcome': f'{obj} relocated, path cleared'
                },
                {
                    'step': 5,
                    'action': 'RESTORE_STACK',
                    'description': 'Optionally restore removed objects (if needed)',
                    'duration_s': 4.0,
                    'preconditions': ['Target relocated'],
                    'outcome': 'Scene stabilized'
                }
            ]
            
        elif weight_ok and graspable and reachable and fragility == 'not_fragile':
            action = ManipulationAction.GRASP
            reasoning = (
                f"✅ GRASP FEASIBLE: Optimal manipulation conditions. "
                f"Object: {material} {obj}, {weight_cat} ({weight_g}g < {self.robot.payload_g}g limit), "
                f"dimensions {size_mm}mm (within {self.robot.gripper_min_mm}-{self.robot.gripper_max_mm}mm range). "
                f"Structural integrity: {fragility}. Shape: {shape}, orientation: {orientation}. "
                f"Scene type: {scene_type}. "
                f"Grasp strategy: {'direct grasp' if scene_type == 'single' else 'careful grasp avoiding neighbors'}. "
                f"Estimated success probability: 85%."
            )
            
            # Grasp execution sequence
            if scene_type == 'single':
                action_sequence = [
                    {
                        'step': 1,
                        'action': 'APPROACH',
                        'description': f'Navigate to {obj} within arm reach (300mm)',
                        'duration_s': 3.0,
                        'preconditions': [],
                        'outcome': 'Positioned for manipulation'
                    },
                    {
                        'step': 2,
                        'action': 'POSITION_GRIPPER',
                        'description': f'Align gripper with {obj} center, orientation: {orientation}',
                        'duration_s': 2.0,
                        'preconditions': ['In reach'],
                        'outcome': 'Gripper aligned'
                    },
                    {
                        'step': 3,
                        'action': 'OPEN_GRIPPER',
                        'description': f'Open to {size_mm + 10}mm (object: {size_mm}mm + 10mm margin)',
                        'duration_s': 0.5,
                        'preconditions': ['Aligned'],
                        'outcome': 'Gripper ready'
                    },
                    {
                        'step': 4,
                        'action': 'CLOSE_GRIPPER',
                        'description': f'Close with {weight_g * 1.5:.0f}g force (1.5x object weight)',
                        'duration_s': 1.0,
                        'preconditions': ['Gripper positioned'],
                        'outcome': 'Secure grasp achieved'
                    },
                    {
                        'step': 5,
                        'action': 'LIFT_TEST',
                        'description': 'Lift 50mm vertically, verify grip security',
                        'duration_s': 1.5,
                        'preconditions': ['Grasp secure'],
                        'outcome': 'Grip verified stable'
                    },
                    {
                        'step': 6,
                        'action': 'TRANSPORT',
                        'description': 'Move to target location (smooth motion, 50% speed)',
                        'duration_s': 5.0,
                        'preconditions': ['Stable grip'],
                        'outcome': 'Object at destination'
                    },
                    {
                        'step': 7,
                        'action': 'PLACE',
                        'description': 'Lower to surface, open gripper, retract',
                        'duration_s': 2.0,
                        'preconditions': ['At destination'],
                        'outcome': f'{obj} placed successfully'
                    }
                ]
            else:
                # Clustered/scattered scene - more careful approach
                action_sequence = [
                    {
                        'step': 1,
                        'action': 'APPROACH_CAREFULLY',
                        'description': f'Navigate to {obj}, avoid nearby objects',
                        'duration_s': 4.0,
                        'preconditions': [],
                        'outcome': 'Positioned safely'
                    },
                    {
                        'step': 2,
                        'action': 'VERIFY_CLEARANCE',
                        'description': 'Check 50mm clearance around object',
                        'duration_s': 1.0,
                        'preconditions': ['Positioned'],
                        'outcome': 'Safe manipulation space confirmed'
                    },
                    {
                        'step': 3,
                        'action': 'PRECISION_GRASP',
                        'description': f'Careful grasp of {obj} ({size_mm}mm), avoid adjacent objects',
                        'duration_s': 3.0,
                        'preconditions': ['Clearance OK'],
                        'outcome': 'Object grasped'
                    },
                    {
                        'step': 4,
                        'action': 'EXTRACT_CAREFULLY',
                        'description': 'Lift vertically 100mm to clear neighbors',
                        'duration_s': 2.0,
                        'preconditions': ['Grasped'],
                        'outcome': 'Extracted without collision'
                    },
                    {
                        'step': 5,
                        'action': 'TRANSPORT',
                        'description': 'Navigate to target (slow, collision-aware)',
                        'duration_s': 6.0,
                        'preconditions': ['Extracted'],
                        'outcome': 'At destination'
                    },
                    {
                        'step': 6,
                        'action': 'PLACE',
                        'description': 'Place object, verify stability',
                        'duration_s': 2.0,
                        'preconditions': ['Transported'],
                        'outcome': 'Task complete'
                    }
                ]
            
            # Alternative: Push instead of grasp
            if not graspable:
                alternative_sequences.append([
                    {
                        'step': 1,
                        'action': 'PUSH',
                        'description': f'Push {obj} aside (too large to grasp)',
                        'duration_s': 3.0,
                        'outcome': 'Object moved'
                    }
                ])
                
        elif not graspable and weight_ok and reachable and fragility == 'not_fragile':
            action = ManipulationAction.PUSH
            reasoning = (
                f"➡️ PUSH STRATEGY: Object dimensions {size_mm}mm exceed gripper maximum {self.robot.gripper_max_mm}mm. "
                f"Weight {weight_g}g is manageable. Material: {material} (durable). "
                f"Push feasibility: HIGH. Direction: perpendicular to intended path. "
                f"Target displacement: 250-300mm (sufficient clearance). "
                f"Force estimation: 10-20N (low-medium). "
                f"Scene type: {scene_type}. Risk: {', '.join(interaction_risks) if interaction_risks else 'minimal'}."
            )
            
            action_sequence = [
                {
                    'step': 1,
                    'action': 'APPROACH',
                    'description': f'Position base aligned with {obj}',
                    'duration_s': 3.0,
                    'preconditions': [],
                    'outcome': 'Aligned for push'
                },
                {
                    'step': 2,
                    'action': 'POSITION_PUSHER',
                    'description': 'Extend arm to contact position (end-effector closed)',
                    'duration_s': 2.0,
                    'preconditions': ['Aligned'],
                    'outcome': 'Ready to push'
                },
                {
                    'step': 3,
                    'action': 'APPLY_FORCE',
                    'description': f'Push {obj} with gradual force (10N → 20N over 2s)',
                    'duration_s': 2.5,
                    'preconditions': ['Contact established'],
                    'outcome': 'Object moving'
                },
                {
                    'step': 4,
                    'action': 'MONITOR_DISPLACEMENT',
                    'description': 'Continue push until 250mm displacement',
                    'duration_s': 2.0,
                    'preconditions': ['Force applied'],
                    'outcome': 'Target displacement reached'
                },
                {
                    'step': 5,
                    'action': 'RETRACT',
                    'description': 'Withdraw arm, verify path clear',
                    'duration_s': 1.5,
                    'preconditions': ['Pushed successfully'],
                    'outcome': 'Path cleared'
                }
            ]
            
        elif not reachable:
            action = ManipulationAction.NAVIGATE_AROUND
            reasoning = (
                f"🚧 OUT OF REACH: {obj} at {distance_category} distance (>{self.robot.reach_mm}mm). "
                f"Cannot manipulate remotely. Scene: {scene_type}. "
                f"Navigation strategy: {'simple bypass' if scene_type == 'single' else 'complex path planning'}. "
                f"Clearance required: 400mm minimum. "
                f"Estimated detour: {'2-3 meters' if blocking else 'minimal'}."
            )
            
            action_sequence = [
                {
                    'step': 1,
                    'action': 'UPDATE_MAP',
                    'description': f'Mark {obj} as static obstacle in costmap',
                    'duration_s': 1.0,
                    'preconditions': [],
                    'outcome': 'Obstacle registered'
                },
                {
                    'step': 2,
                    'action': 'COMPUTE_DETOUR',
                    'description': 'Calculate path around obstacle (A* or DWA planner)',
                    'duration_s': 1.5,
                    'preconditions': ['Map updated'],
                    'outcome': 'Valid path computed'
                },
                {
                    'step': 3,
                    'action': 'EXECUTE_DETOUR',
                    'description': f'Navigate around {obj} maintaining 400mm clearance',
                    'duration_s': 8.0,
                    'preconditions': ['Path computed'],
                    'outcome': 'Obstacle bypassed'
                },
                {
                    'step': 4,
                    'action': 'RESUME_PATH',
                    'description': 'Return to original trajectory',
                    'duration_s': 2.0,
                    'preconditions': ['Detour complete'],
                    'outcome': 'On original path'
                }
            ]
            
        elif not blocking:
            action = ManipulationAction.IGNORE
            reasoning = (
                f"✓ NO ACTION NEEDED: {obj} not obstructing mission objectives. "
                f"Position: {distance_category}, orientation: {orientation}. "
                f"Scene type: {scene_type}. Path: CLEAR. "
                f"Resource optimization: conserve battery and time. "
                f"Continue primary task without deviation."
            )
            
            action_sequence = [
                {
                    'step': 1,
                    'action': 'LOG_OBSERVATION',
                    'description': f'Record {obj} position in world model',
                    'duration_s': 0.2,
                    'preconditions': [],
                    'outcome': 'Object logged'
                },
                {
                    'step': 2,
                    'action': 'CONTINUE',
                    'description': 'Proceed with primary task',
                    'duration_s': 0.0,
                    'preconditions': ['Logged'],
                    'outcome': 'Mission continues'
                }
            ]
            
        else:
            action = ManipulationAction.STOP
            reasoning = (
                f"⚠️ UNCERTAIN: Ambiguous conditions for {obj}. "
                f"Material: {material}, Weight: {weight_cat}, Size: {size_mm}mm, Fragility: {fragility}. "
                f"Scene complexity: {scene_type} with {len(relationships)} relationships. "
                f"Risks: {', '.join(interaction_risks) if interaction_risks else 'unknown'}. "
                f"Recommendation: human assessment required for safe decision."
            )
            
            action_sequence = [
                {
                    'step': 1,
                    'action': 'EMERGENCY_STOP',
                    'description': 'Halt all motion immediately',
                    'duration_s': 0.1,
                    'preconditions': [],
                    'outcome': 'Stopped safely'
                },
                {
                    'step': 2,
                    'action': 'CAPTURE_DATA',
                    'description': 'Record images, sensor data, scene state',
                    'duration_s': 2.0,
                    'preconditions': ['Stopped'],
                    'outcome': 'Data captured'
                },
                {
                    'step': 3,
                    'action': 'REQUEST_OPERATOR',
                    'description': 'Alert human operator with scene snapshot',
                    'duration_s': 0.5,
                    'preconditions': ['Data ready'],
                    'outcome': 'Human notified'
                },
                {
                    'step': 4,
                    'action': 'AWAIT_INSTRUCTION',
                    'description': 'Wait for manual override or guidance',
                    'duration_s': 999.0,
                    'preconditions': ['Operator alerted'],
                    'outcome': 'Awaiting human decision'
                }
            ]
        
        # Calculate total estimated time
        estimated_total_time = sum(step['duration_s'] for step in action_sequence)
        
        analysis_time = (time.time() - start) * 1000
        
        print(f"\n{'='*80}")
        print(f"DECISION: {action.value}")
        print(f"Reasoning: {reasoning[:150]}...")
        print(f"Action Steps: {len(action_sequence)}")
        print(f"Est. Time: {estimated_total_time:.1f}s")
        print(f"Analysis Time: {analysis_time:.0f}ms")
        print(f"{'='*80}\n")
        
        # Combined confidence
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
            shape=shape,
            reachable=reachable,
            path_blocking=blocking,
            distance_category=distance_category,
            orientation=orientation,
            scene_type=scene_type,
            object_relationships=relationships,
            interaction_risks=interaction_risks,
            graspable=graspable,
            action=action.value,
            reasoning=reasoning,
            confidence=combined_conf,
            action_sequence=action_sequence,
            alternative_sequences=alternative_sequences,
            estimated_total_time_s=estimated_total_time,
            analysis_time_ms=analysis_time,
            reasoning_method='cot' if use_cot else 'zero_shot',
            yolo_guided=yolo_guided,
            yolo_confidence=yolo_confidence
        )

# Initialize Flask app
app = Flask(__name__)
robot = RobotSpec()
vlm = EnhancedMoondream2VLM(robot)

@app.route('/health', methods=['GET'])
def health():
    return jsonify({
        'status': 'healthy',
        'model_loaded': vlm.model is not None,
        'total_queries': vlm.query_count,
        'capabilities': [
            'spatial_relationship_analysis',
            'multi_object_reasoning',
            'chain_of_thought_prompting',
            'action_sequence_planning',
            'interaction_risk_assessment'
        ],
        'robot_spec': asdict(robot)
    })

@app.route('/analyze', methods=['POST'])
def analyze():
    try:
        data = request.json
        
        # Decode image
        img_base64 = data['image']
        img_bytes = base64.b64decode(img_base64)
        image = Image.open(io.BytesIO(img_bytes)).convert('RGB')
        
        # Get parameters
        object_class = data.get('object_class', 'object')
        yolo_confidence = data.get('yolo_confidence', 0.0)
        use_cot = data.get('use_cot', True)  # Chain-of-Thought enabled by default
        
        # Run comprehensive analysis
        analysis = vlm.analyze_comprehensive(
            image, 
            object_class=object_class,
            yolo_confidence=yolo_confidence,
            use_cot=use_cot
        )
        
        # Return as JSON
        return jsonify(asdict(analysis))
        
    except Exception as e:
        print(f"ERROR: {e}")
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    print("=" * 80)
    print("🤖 Enhanced VLM ROS2 Inference Server")
    print("Comprehensive Spatial Reasoning & Action Planning")
    print("=" * 80)
    
    vlm.load()
    
    print(f"\n📋 Robot Configuration:")
    print(f"  Max Reach: {robot.reach_mm}mm")
    print(f"  Max Payload: {robot.payload_g}g")
    print(f"  Gripper Range: {robot.gripper_min_mm}-{robot.gripper_max_mm}mm")
    print(f"  Base Speed: {robot.base_speed_mps}m/s")
    
    print(f"\n🧠 Capabilities:")
    print(f"  ✓ Multi-object spatial analysis")
    print(f"  ✓ Physical interaction reasoning")
    print(f"  ✓ Chain-of-Thought (CoT) prompting")
    print(f"  ✓ Action sequence generation")
    print(f"  ✓ Risk assessment & contingency planning")
    
    print(f"\n✅ Server ready on http://0.0.0.0:5000")
    print(f"   Endpoints:")
    print(f"     GET  /health  - Health check")
    print(f"     POST /analyze - Comprehensive VLM analysis")
    print("=" * 80)
    
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)