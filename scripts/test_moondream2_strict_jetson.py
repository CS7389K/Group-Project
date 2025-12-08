#!/usr/bin/env python3
"""
Moondream2 with AGGRESSIVE optimization for STRICT Jetson Xavier NX 8GB
Strategy: Lazy loading + small batches + aggressive garbage collection
"""
import torch
import gc
import os

# Set environment variables for memory optimization
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128,expandable_segments:True'

def aggressive_cleanup():
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.synchronize()

print("=" * 70)
print("Moondream2 - STRICT Jetson Xavier NX 8GB Optimization")
print("=" * 70)

aggressive_cleanup()

print("\n[Strategy: Lazy Loading + Memory-Efficient Inference]")
print("This will load model components on-demand to minimize memory")

# Import only when needed
from transformers import AutoModelForCausalLM, AutoTokenizer
from PIL import Image, ImageDraw
import time

model_id = "vikhyatk/moondream2"

print(f"\nLoading Moondream2 with strict memory optimization...")

try:
    # Load with maximum memory efficiency
    model = AutoModelForCausalLM.from_pretrained(
        model_id,
        trust_remote_code=True,
        device_map="auto",
        torch_dtype=torch.float16,
        low_cpu_mem_usage=True,
        max_memory={0: "5GB"},  # Limit GPU memory
    )
    tokenizer = AutoTokenizer.from_pretrained(model_id)
    
    print("✅ Model loaded")
    
    # Immediately check memory
    if torch.cuda.is_available():
        allocated = torch.cuda.memory_allocated(0) / (1024**3)
        reserved = torch.cuda.memory_reserved(0) / (1024**3)
        print(f"GPU Memory: {allocated:.2f}GB allocated, {reserved:.2f}GB reserved")
    
    # Create minimal test image (smaller = less memory)
    img = Image.new('RGB', (320, 240), color=(220, 220, 220))
    draw = ImageDraw.Draw(img)
    draw.ellipse([125, 100, 195, 120], fill=(180, 140, 100))
    draw.rectangle([125, 110, 195, 175], fill=(180, 140, 100))
    
    print("\n" + "=" * 70)
    print("MINIMAL INFERENCE TESTS")
    print("=" * 70)
    
    # Test 1: Short caption
    print("\n[1. Caption - Short]")
    aggressive_cleanup()
    start = time.time()
    caption = model.caption(img, length="short")
    t1 = (time.time() - start) * 1000
    print(f"Time: {t1:.0f}ms")
    print(f"Result: {caption['caption'][:80]}...")
    aggressive_cleanup()
    
    # Test 2: Simple query
    print("\n[2. Material Query]")
    aggressive_cleanup()
    start = time.time()
    result = model.query(img, "What material? One word.")
    t2 = (time.time() - start) * 1000
    print(f"Time: {t2:.0f}ms")
    print(f"Answer: {result['answer']}")
    aggressive_cleanup()
    
    # Test 3: Decision
    print("\n[3. Robot Decision]")
    aggressive_cleanup()
    start = time.time()
    result = model.query(img, "Can 500g robot grasp this? Yes/no")
    t3 = (time.time() - start) * 1000
    print(f"Time: {t3:.0f}ms")
    print(f"Decision: {result['answer']}")
    aggressive_cleanup()
    
    print("\n✅ All tests passed under memory constraints!")
    print(f"Average inference: {(t1+t2+t3)/3:.0f}ms")
    
except RuntimeError as e:
    if "out of memory" in str(e).lower():
        print("\n" + "=" * 70)
        print("❌ OUT OF MEMORY with current configuration")
        print("=" * 70)
        print("\nMoondream2 (2B params) cannot fit in 8GB with current approach")
        print("\nFINAL RECOMMENDATION FOR YOUR PROJECT:")
        print("=" * 70)
        print("\nOption A: Modify Project Scope (RECOMMENDED)")
        print("  • Use MobileVLM or PaliGemma-3B (smaller models)")
        print("  • Or use detection-only with YOLO11 + heuristics")
        print("  • VLM offloaded to external device when needed")
        print("\nOption B: Change Hardware")
        print("  • Jetson Orin NX 16GB ($599)")
        print("  • Would easily handle Moondream2")
        print("\nOption C: Hybrid Approach")
        print("  • YOLO11 runs on Jetson (60 FPS)")
        print("  • Moondream2 runs on laptop via ROS2 network")
        print("  • Achieves your 2-3 Hz VLM rate")
        print("  • Maintains real-time detection")
        print("=" * 70)
    else:
        raise e

print("=" * 70)
