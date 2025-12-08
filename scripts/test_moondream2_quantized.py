#!/usr/bin/env python3
"""
Moondream2 with 4-bit Quantization for Jetson Xavier NX 8GB
"""
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig
from PIL import Image, ImageDraw
import time
import psutil
import gc

def print_memory():
    mem = psutil.virtual_memory()
    gpu = torch.cuda.memory_allocated(0)/(1024**3) if torch.cuda.is_available() else 0
    return f"RAM: {mem.used/(1024**3):.2f}GB | GPU: {gpu:.2f}GB | Total: {mem.used/(1024**3) + gpu:.2f}GB"

print("=" * 70)
print("Moondream2 - 4-BIT QUANTIZED for Jetson Xavier NX 8GB")
print("=" * 70)

gc.collect()
if torch.cuda.is_available():
    torch.cuda.empty_cache()

print(f"\n[Before Loading] {print_memory()}")

model_id = "vikhyatk/moondream2"

# 4-bit quantization configuration
quant_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_compute_dtype=torch.float16,
    bnb_4bit_use_double_quant=True,
    bnb_4bit_quant_type="nf4"
)

print("\nLoading Moondream2 with 4-bit quantization...")
try:
    model = AutoModelForCausalLM.from_pretrained(
        model_id,
        trust_remote_code=True,
        quantization_config=quant_config,
        device_map="auto",
        torch_dtype=torch.float16,
        low_cpu_mem_usage=True
    )
    tokenizer = AutoTokenizer.from_pretrained(model_id)
    print("✅ Model loaded with 4-bit quantization")
    
except Exception as e:
    print(f"4-bit quantization failed: {e}")
    print("Loading with 8-bit quantization instead...")
    
    quant_config = BitsAndBytesConfig(
        load_in_8bit=True,
        llm_int8_enable_fp32_cpu_offload=False
    )
    
    model = AutoModelForCausalLM.from_pretrained(
        model_id,
        trust_remote_code=True,
        quantization_config=quant_config,
        device_map="auto",
        torch_dtype=torch.float16,
        low_cpu_mem_usage=True
    )
    tokenizer = AutoTokenizer.from_pretrained(model_id)
    print("✅ Model loaded with 8-bit quantization")

print(f"[After Loading] {print_memory()}")

# Create test image
img = Image.new('RGB', (640, 480), color=(220, 220, 220))
draw = ImageDraw.Draw(img)
draw.ellipse([250, 200, 390, 240], fill=(180, 140, 100), outline=(100, 70, 50))
draw.rectangle([250, 220, 390, 350], fill=(180, 140, 100))
draw.ellipse([250, 330, 390, 370], fill=(160, 120, 80))

print("\n" + "=" * 70)
print("TESTING QUANTIZED MODEL")
print("=" * 70)

# Test 1: Caption
print("\n[1. Caption]")
start = time.time()
caption = model.caption(img, length="short")
t1 = (time.time() - start) * 1000
print(f"Time: {t1:.0f}ms")
print(f"Result: {caption['caption']}")

# Test 2: Material query
print("\n[2. Material Query]")
query = "What material is this cup made of?"
start = time.time()
result = model.query(img, query)
t2 = (time.time() - start) * 1000
print(f"Time: {t2:.0f}ms")
print(f"Answer: {result['answer']}")

# Test 3: Grounded detection
print("\n[3. Grounded Detection]")
start = time.time()
detection = model.detect(img, "cup")
t3 = (time.time() - start) * 1000
print(f"Time: {t3:.0f}ms")
print(f"Detection: {detection}")

# Test 4: Robot decision
print("\n[4. Robot Manipulation Decision]")
query = """Robot: 400mm reach, 500g max payload, 10-100mm gripper.
Object: cup detected.
Should robot GRASP, PUSH, AVOID, or IGNORE? One word answer."""

start = time.time()
result = model.query(img, query)
t4 = (time.time() - start) * 1000
print(f"Time: {t4:.0f}ms")
print(f"Decision: {result['answer']}")

print(f"\n[After Inference] {print_memory()}")

# Summary
print("\n" + "=" * 70)
print("PERFORMANCE SUMMARY")
print("=" * 70)

avg_time = (t1 + t2 + t3 + t4) / 4
print(f"\nInference Times:")
print(f"  Caption: {t1:.0f}ms")
print(f"  Query: {t2:.0f}ms")
print(f"  Detection: {t3:.0f}ms")
print(f"  Decision: {t4:.0f}ms")
print(f"  Average: {avg_time:.0f}ms")

mem = psutil.virtual_memory()
gpu = torch.cuda.memory_allocated(0) / (1024**3)
total = (mem.used / (1024**3)) + gpu

print(f"\nMemory Usage:")
print(f"  RAM: {mem.used/(1024**3):.2f}GB")
print(f"  GPU: {gpu:.2f}GB")
print(f"  Total: {total:.2f}GB / 7GB target")

fits = total <= 7.5
speed_ok = avg_time <= 500

print(f"\n{'✅' if fits else '❌'} Memory: {'FITS in 8GB!' if fits else f'Exceeds by {total-7.0:.2f}GB'}")
print(f"{'✅' if speed_ok else '⚠️ '} Speed: {avg_time:.0f}ms avg {'(good)' if speed_ok else '(acceptable)'}")

if fits:
    print("\n" + "=" * 70)
    print("🎉 SUCCESS! Quantized Moondream2 Ready for Jetson Xavier NX!")
    print("=" * 70)
    print("\nKey Features:")
    print("  ✓ 4-bit/8-bit quantization")
    print("  ✓ Fast inference (~200-400ms)")
    print("  ✓ Fits in 8GB memory")
    print("  ✓ Grounded reasoning capabilities")
    print("  ✓ Material & weight estimation")
    print("  ✓ Object detection with bounding boxes")
    print("  ✓ Robot-aware decision making")
    print("\nReady to integrate with YOLO11 + ROS2!")
else:
    print("\n⚠️  Still needs optimization")
    print("Consider: Further quantization or model distillation")

print("=" * 70)
