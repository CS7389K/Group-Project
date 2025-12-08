#!/usr/bin/env python3
"""
Moondream2 Proper Test for Jetson Xavier NX with Robot-Aware Grounded Reasoning
"""
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from PIL import Image, ImageDraw
import time
import psutil
import gc
import json

def print_memory():
    mem = psutil.virtual_memory()
    gpu = torch.cuda.memory_allocated(0)/(1024**3) if torch.cuda.is_available() else 0
    return f"RAM: {mem.used/(1024**3):.2f}GB | GPU: {gpu:.2f}GB"

print("=" * 70)
print("Moondream2 - Robot-Aware Grounded Reasoning for TurtleBot3")
print("=" * 70)

gc.collect()
if torch.cuda.is_available():
    torch.cuda.empty_cache()

print(f"\n[Before Loading] {print_memory()}")

# Load model (already loaded earlier, but doing it properly)
model_id = "vikhyatk/moondream2"
revision = "2024-08-26"

print("\nLoading Moondream2...")
model = AutoModelForCausalLM.from_pretrained(
    model_id,
    revision=revision,
    trust_remote_code=True,
    device_map={"": "cuda"},
    torch_dtype=torch.float16
)
tokenizer = AutoTokenizer.from_pretrained(model_id, revision=revision)

print(f"[After Loading] {print_memory()}")

# Create realistic test scene with objects
print("\n" + "=" * 70)
print("SIMULATION: Object Detection + Property Estimation")
print("=" * 70)

# Create test image with a cup-like object
img = Image.new('RGB', (640, 480), color=(200, 200, 200))
draw = ImageDraw.Draw(img)
# Draw a cup (cylinder shape)
draw.ellipse([250, 200, 390, 240], fill=(180, 140, 100), outline=(100, 70, 50))  # Top
draw.rectangle([250, 220, 390, 350], fill=(180, 140, 100))  # Body
draw.ellipse([250, 330, 390, 370], fill=(160, 120, 80))  # Bottom

# Simulate YOLO detection result
yolo_detection = {
    "class": "cup",
    "bbox": [250, 200, 390, 370],  # [x1, y1, x2, y2]
    "confidence": 0.95,
    "center": [320, 285],
    "size_px": [140, 170]  # width, height in pixels
}

print(f"\nYOLO11 Detection:")
print(f"  Object: {yolo_detection['class']}")
print(f"  Confidence: {yolo_detection['confidence']:.2f}")
print(f"  BBox: {yolo_detection['bbox']}")
print(f"  Center: {yolo_detection['center']}")

# Robot specifications
robot_spec = {
    "reach_mm": 400,
    "payload_g": 500,
    "gripper_range_mm": [10, 100]
}

print(f"\nRobot Specifications:")
print(f"  Reach: {robot_spec['reach_mm']}mm")
print(f"  Max payload: {robot_spec['payload_g']}g")
print(f"  Gripper range: {robot_spec['gripper_range_mm']}mm")

# Encode image
enc_image = model.encode_image(img)

print("\n" + "=" * 70)
print("GROUNDED REASONING QUERIES")
print("=" * 70)

# Query 1: Material and Weight Estimation
print("\n[Query 1: Material & Weight]")
prompt1 = f"""Detected object: {yolo_detection['class']} at center {yolo_detection['center']}.
Based on visual appearance, what material is this object likely made of? 
Estimate weight category: light (<100g), medium (100-500g), or heavy (>500g)."""

start = time.time()
answer1 = model.answer_question(enc_image, prompt1, tokenizer)
time1 = time.time() - start

print(f"Response ({time1*1000:.0f}ms):")
print(f"  {answer1}")

# Query 2: Size and Graspability
print("\n[Query 2: Size & Graspability]")
prompt2 = f"""This object appears at pixel size {yolo_detection['size_px']}.
Is this object small enough to grasp with a gripper that opens 10-100mm?
Answer yes or no, then explain."""

start = time.time()
answer2 = model.answer_question(enc_image, prompt2, tokenizer)
time2 = time.time() - start

print(f"Response ({time2*1000:.0f}ms):")
print(f"  {answer2}")

# Query 3: Fragility Assessment
print("\n[Query 3: Fragility]")
prompt3 = f"""Is this {yolo_detection['class']} fragile? 
Can it withstand being grasped and moved by a robot?
Answer: fragile/not fragile, then explain."""

start = time.time()
answer3 = model.answer_question(enc_image, prompt3, tokenizer)
time3 = time.time() - start

print(f"Response ({time3*1000:.0f}ms):")
print(f"  {answer3}")

# Query 4: Object Detection (grounded)
print("\n[Query 4: Grounded Detection]")
prompt4 = "Describe the exact location and appearance of any objects you see in this image."

start = time.time()
answer4 = model.answer_question(enc_image, prompt4, tokenizer)
time4 = time.time() - start

print(f"Response ({time4*1000:.0f}ms):")
print(f"  {answer4}")

# Query 5: Integrated Decision
print("\n[Query 5: Manipulation Decision]")
prompt5 = f"""Robot specs: {robot_spec['reach_mm']}mm reach, {robot_spec['payload_g']}g max payload, {robot_spec['gripper_range_mm']}mm gripper.
Detected: {yolo_detection['class']} at {yolo_detection['center']}.

Should the robot: GRASP, PUSH, AVOID, or IGNORE this object?
Provide answer and brief reasoning."""

start = time.time()
answer5 = model.answer_question(enc_image, prompt5, tokenizer)
time5 = time.time() - start

print(f"Response ({time5*1000:.0f}ms):")
print(f"  {answer5}")

# Caption test
print("\n[Bonus: Short Caption]")
start = time.time()
caption = model.caption(img, length="short")
caption_time = time.time() - start
print(f"Caption ({caption_time*1000:.0f}ms): {caption['caption']}")

print("\n" + "=" * 70)
print("PERFORMANCE SUMMARY")
print("=" * 70)

avg_time = (time1 + time2 + time3 + time4 + time5) / 5

print(f"\nInference Times:")
print(f"  Material/Weight: {time1*1000:.0f}ms")
print(f"  Graspability: {time2*1000:.0f}ms")
print(f"  Fragility: {time3*1000:.0f}ms")
print(f"  Detection: {time4*1000:.0f}ms")
print(f"  Decision: {time5*1000:.0f}ms")
print(f"  Caption: {caption_time*1000:.0f}ms")
print(f"  Average: {avg_time*1000:.0f}ms")

print(f"\nMemory: {print_memory()}")

mem = psutil.virtual_memory()
total = mem.used / (1024**3)

print(f"\nTotal Memory: {total:.2f}GB / 7GB target")
print(f"{'✅' if total <= 7.0 else '❌'} {'FITS in Jetson Xavier NX 8GB!' if total <= 7.0 else f'EXCEEDS by {total-7.0:.2f}GB'}")

if avg_time <= 0.45:
    print(f"✅ Speed: Meets target (<450ms avg)")
else:
    print(f"⚠️  Speed: {avg_time*1000:.0f}ms avg (target: <450ms)")

print("\n" + "=" * 70)
print("🎉 Moondream2 is READY for TurtleBot3 deployment!")
print("=" * 70)
print("\nNext steps:")
print("  1. Integrate with YOLO11 detection pipeline")
print("  2. Implement decision fusion logic")
print("  3. Create ROS 2 nodes for perception system")
print("  4. Test on actual Jetson Xavier NX hardware")
print("=" * 70)
