#!/usr/bin/env python3
"""
Diagnostic script to test library loading on Jetson
Tests different combinations to fix TLS errors
"""
import ctypes
import os
import sys

print("=" * 60)
print("Jetson TLS Library Diagnostic")
print("=" * 60)

# Library candidates to test
lib_candidates = [
    '/usr/lib/aarch64-linux-gnu/libgomp.so.1',
    '/usr/lib/aarch64-linux-gnu/libopenblas.so.0',
    '/usr/lib/aarch64-linux-gnu/libGLdispatch.so.0',
    '/usr/local/lib/libgomp.so.1',
    '/usr/lib/libgomp.so.1',
]

print("\n1. Checking library files...")
found_libs = []
for lib in lib_candidates:
    exists = os.path.exists(lib)
    status = "✓" if exists else "✗"
    print(f"  {status} {lib}")
    if exists:
        found_libs.append(lib)

print(f"\n2. Found {len(found_libs)} libraries")

print("\n3. Testing ctypes preload...")
for lib in found_libs:
    try:
        ctypes.CDLL(lib, mode=ctypes.RTLD_GLOBAL)
        print(f"  ✓ Successfully preloaded: {lib}")
    except Exception as e:
        print(f"  ✗ Failed to preload {lib}: {e}")

print("\n4. Testing PyTorch import...")
try:
    import torch
    print(f"  ✓ PyTorch imported successfully")
    print(f"    Version: {torch.__version__}")
    print(f"    CUDA available: {torch.cuda.is_available()}")
except Exception as e:
    print(f"  ✗ PyTorch import failed: {e}")

print("\n5. Testing ultralytics import...")
try:
    from ultralytics import YOLO
    print(f"  ✓ Ultralytics imported successfully")
except Exception as e:
    print(f"  ✗ Ultralytics import failed: {e}")

print("\n6. Testing transformers import...")
try:
    from transformers import AutoTokenizer
    print(f"  ✓ Transformers imported successfully")
except Exception as e:
    print(f"  ✗ Transformers import failed: {e}")

print("\n" + "=" * 60)
print("Diagnostic complete!")
print("=" * 60)

if found_libs:
    print("\n💡 Recommended command to run vlm_reasoner.py:")
    preload_str = ":".join(found_libs)
    print(f"   LD_PRELOAD={preload_str} python3 vlm_reasoner.py")
else:
    print("\n⚠ No preload libraries found. You may need to install libgomp1:")
    print("   sudo apt-get install libgomp1")
