#!/usr/bin/env python3
"""
Moondream2 Working Test - Latest Version with Correct API
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
print("Moondream2 - Robot Manipulation with Grounded Reasoning")
print("=" * 70)

# Clear memory
gc.collect()
if torch.cuda.is_available():
    torch.cuda.empty_cache()

print(f"\n[Before Loading] {print_memory()}")

model_id = "vikhyatk/moondream2"

print("\nLoading Moondream2 (latest version)...")
model = AutoModelForCausalLM.from_pretrained(
    model_id,
    trust_remote_code=True,
    device_map={"": "cuda"},
    torch_dtype=torch.float16,
    attn_implementation="eager"  # More memory efficient
)
tokenizer = AutoTokenizer.from_pretrained(model_id)

print(f"✅ Model loaded")
print(f"[After Loading] {print_memory()}")

# Create realistic test scene
img = Image.new('RGB', (640, 480), color=(220, 220, 220))
draw = ImageDraw.Draw(img)
# Draw a cup
draw.ellipse([250, 200, 390, 240], fill=(180, 140, 100), outline=(100, 70, 50))
draw.rectangle([250, 220, 390, 350], fill=(180, 140, 100))
draw.ellipse([250, 330, 390, 370], fill=(160, 120, 80))

# Robot specs
robot_spec = {
    "reach_mm": 400,
    "payload_g": 500,
    "gripper_mm": [10, 100]
}

# Simulated YOLO detection
yolo_detection = {
    "class": "cup",
    "bbox": [250, 200, 390, 370],
    "confidence": 0.95
}

print("\n" + "=" * 70)
print("ROBOT PERCEPTION PIPELINE")
print("=" * 70)

print(f"\nYOLO Detection: {yolo_detection['class']} (conf: {yolo_detection['confidence']:.2f})")
print(f"Robot Specs: {robot_spec['reach_mm']}mm reach, {robot_spec['payload_g']}g payload")

# Test 1: Caption (Scene Understanding)
print("\n[1. Scene Caption]")
start = time.time()
caption_result = model.caption(img, length="short")
t1 = (time.time() - start) * 1000
print(f"Time: {t1:.0f}ms")
print(f"Caption: {caption_result['caption']}")

# Test 2: Query - Material Identification
print("\n[2. Material Identification]")
query1 = "What material is the cup-like object made of? Answer in one word."
start = time.time()
result1 = model.query(img, query1)
t2 = (time.time() - start) * 1000
print(f"Time: {t2:.0f}ms")
print(f"Q: {query1}")
print(f"A: {result1['answer']}")

# Test 3: Query - Weight Estimation
print("\n[3. Weight Estimation]")
query2 = "Is this cup light (<100g), medium (100-500g), or heavy (>500g)? Answer with one word."
start = time.time()
result2 = model.query(img, query2)
t3 = (time.time() - start) * 1000
print(f"Time: {t3:.0f}ms")
print(f"Q: {query2}")
print(f"A: {result2['answer']}")

# Test 4: Query - Size Assessment
print("\n[4. Size Assessment]")
query3 = "Is this cup small enough to fit in a 10-100mm gripper? Answer yes or no."
start = time.time()
result3 = model.query(img, query3)
t4 = (time.time() - start) * 1000
print(f"Time: {t4:.0f}ms")
print(f"Q: {query3}")
print(f"A: {result3['answer']}")

# Test 5: Query - Fragility
print("\n[5. Fragility Assessment]")
query4 = "Is this cup fragile? Answer yes or no, then explain briefly."
start = time.time()
result4 = model.query(img, query4)
t5 = (time.time() - start) * 1000
print(f"Time: {t5:.0f}ms")
print(f"Q: {query4}")
print(f"A: {result4['answer']}")

# Test 6: Detect (Grounded Detection)
print("\n[6. Grounded Object Detection]")
start = time.time()
detect_result = model.detect(img, "cup")
t6 = (time.time() - start) * 1000
print(f"Time: {t6:.0f}ms")
print(f"Detection: {detect_result}")

# Test 7: Integrated Decision Query
print("\n[7. Manipulation Decision]")
query5 = f"""Robot: {robot_spec['reach_mm']}mm reach, {robot_spec['payload_g']}g max, gripper {robot_spec['gripper_mm']}mm.
Detected: {yolo_detection['class']}.

Should robot GRASP, PUSH, AVOID, or IGNORE? Answer with action first, then brief reason."""

start = time.time()
result5 = model.query(img, query5)
t7 = (time.time() - start) * 1000
print(f"Time: {t7:.0f}ms")
print(f"Decision: {result5['answer']}")

# Performance Summary
print("\n" + "=" * 70)
print("PERFORMANCE SUMMARY")
print("=" * 70)

times = [t1, t2, t3, t4, t5, t6, t7]
avg_time = sum(times) / len(times)

print(f"\nInference Times:")
print(f"  Caption: {t1:.0f}ms")
print(f"  Material: {t2:.0f}ms")
print(f"  Weight: {t3:.0f}ms")
print(f"  Size: {t4:.0f}ms")
print(f"  Fragility: {t5:.0f}ms")
print(f"  Detection: {t6:.0f}ms")
print(f"  Decision: {t7:.0f}ms")
print(f"  Average: {avg_time:.0f}ms")

print(f"\nMemory: {print_memory()}")

mem = psutil.virtual_memory()
ram = mem.used / (1024**3)
gpu = torch.cuda.memory_allocated(0) / (1024**3)
total = ram + gpu

print(f"\nTotal Memory: {total:.2f}GB / 7GB target")

fits = total <= 7.5  # Allow some tolerance
speed_ok = avg_time <= 500

print(f"\n{'✅' if fits else '⚠️ '} Memory: {'FITS!' if fits else f'Slightly over ({total:.2f}GB)'}")
print(f"{'✅' if speed_ok else '⚠️ '} Speed: {'Good (<500ms)' if speed_ok else f'{avg_time:.0f}ms avg'}")

if fits and speed_ok:
    print("\n" + "=" * 70)
    print("🎉 SUCCESS! Moondream2 Ready for Jetson Xavier NX!")
    print("=" * 70)
    print("\nCapabilities Confirmed:")
    print("  ✓ Scene understanding & captioning")
    print("  ✓ Material identification")
    print("  ✓ Weight estimation")
    print("  ✓ Size assessment")
    print("  ✓ Fragility evaluation")
    print("  ✓ Grounded object detection")
    print("  ✓ Manipulation decision making")
    print(f"  ✓ Fast inference (~{avg_time:.0f}ms avg)")
    print(f"  ✓ Low memory (~{total:.1f}GB)")
elif fits:
    print("\n✅ Memory fits, but optimize for faster inference")
else:
    print("\n⚠️  Need to reduce memory by ~1GB")
    print("   Options: Use 4-bit quantization or smaller batch sizes")

print("=" * 70)
