#!/usr/bin/env python3
"""
Moondream 0.5B - Tiny Version Perfect for Jetson Xavier NX 8GB
According to docs: 4-bit = 816MB runtime, 8-bit = 996MB runtime
"""
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from PIL import Image, ImageDraw
import time
import psutil
import gc

def print_memory():
    mem = psutil.virtual_memory()
    gpu = torch.cuda.memory_allocated(0)/(1024**3) if torch.cuda.is_available() else 0
    total = mem.used/(1024**3) + gpu
    return f"RAM: {mem.used/(1024**3):.2f}GB | GPU: {gpu:.2f}GB | Total: {total:.2f}GB"

print("=" * 70)
print("Moondream 0.5B - Optimized for Edge Devices")
print("=" * 70)

gc.collect()
if torch.cuda.is_available():
    torch.cuda.empty_cache()

print(f"\n[Before Loading] {print_memory()}")

# Try Moondream 0.5B (much smaller!)
model_id = "vikhyatk/moondream0.5b"

print(f"\nLoading {model_id}...")
try:
    model = AutoModelForCausalLM.from_pretrained(
        model_id,
        trust_remote_code=True,
        device_map={"": "cuda"},
        torch_dtype=torch.float16
    )
    tokenizer = AutoTokenizer.from_pretrained(model_id)
    print(f"✅ Moondream 0.5B loaded successfully")
    
except Exception as e:
    print(f"❌ Moondream 0.5B not found: {e}")
    print("\nFalling back to Moondream2 without quantization...")
    print("(We'll optimize deployment strategy instead)")
    
    model_id = "vikhyatk/moondream2"
    model = AutoModelForCausalLM.from_pretrained(
        model_id,
        trust_remote_code=True,
        device_map={"": "cuda"},
        torch_dtype=torch.float16
    )
    tokenizer = AutoTokenizer.from_pretrained(model_id)
    print("✅ Moondream2 loaded")

print(f"[After Loading] {print_memory()}")

# Create test image
img = Image.new('RGB', (640, 480), color=(220, 220, 220))
draw = ImageDraw.Draw(img)
draw.ellipse([250, 200, 390, 240], fill=(180, 140, 100), outline=(100, 70, 50))
draw.rectangle([250, 220, 390, 350], fill=(180, 140, 100))
draw.ellipse([250, 330, 390, 370], fill=(160, 120, 80))

print("\n" + "=" * 70)
print("ROBOT MANIPULATION TESTS")
print("=" * 70)

# Test 1: Caption
print("\n[1. Scene Understanding]")
start = time.time()
caption = model.caption(img, length="short")
t1 = (time.time() - start) * 1000
print(f"Caption ({t1:.0f}ms): {caption['caption']}")

# Test 2: Material
print("\n[2. Material Identification]")
start = time.time()
result = model.query(img, "What material is this object?")
t2 = (time.time() - start) * 1000
print(f"Material ({t2:.0f}ms): {result['answer']}")

# Test 3: Weight
print("\n[3. Weight Estimation]")
start = time.time()
result = model.query(img, "Is this light, medium, or heavy?")
t3 = (time.time() - start) * 1000
print(f"Weight ({t3:.0f}ms): {result['answer']}")

# Test 4: Robot decision
print("\n[4. Manipulation Decision]")
start = time.time()
result = model.query(img, "Robot gripper: 10-100mm, 500g max. Can grasp this cup? Yes or no.")
t4 = (time.time() - start) * 1000
print(f"Decision ({t4:.0f}ms): {result['answer']}")

print(f"\n[After Inference] {print_memory()}")

# Summary
print("\n" + "=" * 70)
print("DEPLOYMENT ASSESSMENT")
print("=" * 70)

avg_time = (t1 + t2 + t3 + t4) / 4
mem = psutil.virtual_memory()
gpu = torch.cuda.memory_allocated(0) / (1024**3)
total = (mem.used / (1024**3)) + gpu

print(f"\nPerformance:")
print(f"  Average inference: {avg_time:.0f}ms")
print(f"  Total memory: {total:.2f}GB / 7GB target")

fits = total <= 7.5

if fits:
    print(f"\n✅ FITS in Jetson Xavier NX 8GB!")
else:
    print(f"\n❌ Memory: {total:.2f}GB (exceeds by {total-7.0:.2f}GB)")
    print("\nDEPLOYMENT STRATEGY:")
    print("  Option 1: Use Moondream 0.5B (if available)")
    print("  Option 2: Run VLM on separate device/cloud")
    print("  Option 3: Selective VLM usage (only for ambiguous cases)")
    print("  Option 4: Use Jetson Orin NX 16GB instead")
    
    print("\nRECOMMENDED APPROACH for your project:")
    print("  • Keep YOLO11 on Jetson (real-time 60 FPS)")
    print("  • Run Moondream2 on laptop/workstation")
    print("  • Jetson sends images → Laptop processes → Returns decisions")
    print("  • Adds ~100-200ms latency but works within constraints")
    print("  • Still achieves 2-3 Hz VLM rate from proposal")

print("=" * 70)
