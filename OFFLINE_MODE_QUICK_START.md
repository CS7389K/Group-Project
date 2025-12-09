# Offline VLM Server - Quick Start Guide

## Overview

The VLM server now supports **offline mode** - perfect for running on routers or devices without internet access.

## Two Simple Steps

### Step 1: Download Models (With Internet)

On any computer with internet access:

```bash
# Install dependencies (if not already installed)
pip3 install -r standalone_requirements.txt

# Download all model files
python3 download_vlm_models.py --output-dir ./vlm_models
```

This downloads ~2-4 GB of model files to `./vlm_models/`.

### Step 2: Transfer and Run (Without Internet)

Copy the `vlm_models/` folder to your offline device:

```bash
# Using SCP
scp -r ./vlm_models user@router-ip:/path/to/destination/

# Or copy to USB drive, then copy from USB to device
```

On the offline device, run:

```bash
python3 standalone_vlm_server.py \
    --model-cache-dir ./vlm_models \
    --host 0.0.0.0 \
    --port 5000 \
    --device cuda
```

**That's it!** The server now runs completely offline.

## Complete Example

### On PC with Internet:

```bash
git clone https://github.com/CS7389K/Group-Project.git
cd Group-Project

# Install dependencies
pip3 install torch transformers accelerate bitsandbytes flask pillow

# Download models
python3 download_vlm_models.py --output-dir ./vlm_models

# Package everything
tar czf vlm_offline.tar.gz standalone_vlm_server.py vlm_models/
```

### On Offline Router:

```bash
# Extract
tar xzf vlm_offline.tar.gz

# Run (NO internet needed!)
python3 standalone_vlm_server.py --model-cache-dir ./vlm_models --host 0.0.0.0 --port 5000
```

## Deployment on Jetson

On Jetson (unchanged):

```bash
ros2 launch vlm_bridge complete_network_bridge.launch.py \
    vlm_server_url:=http://ROUTER_IP:5000
```

The Jetson sends images to your router's VLM server via HTTP.

## Options

```bash
# Download with 4-bit quantization (smaller size)
python3 download_vlm_models.py --output-dir ./vlm_models --use-4bit

# Run on CPU instead of GPU
python3 standalone_vlm_server.py --model-cache-dir ./vlm_models --device cpu

# Use different quantization
python3 standalone_vlm_server.py --model-cache-dir ./vlm_models --quantization 4bit
```

## Verification

Test the offline server:

```bash
# Health check
curl http://ROUTER_IP:5000/health

# Full test from another machine
python3 tools/test_network_bridge.py --server-url http://ROUTER_IP:5000
```

## Architecture

```
┌────────────────────────────────────────┐
│ JETSON (with ROS2 and internet)        │
│  - Camera publishes images             │
│  - Bridge sends to router via HTTP     │
└──────────────┬─────────────────────────┘
               │
               │ HTTP POST (Base64 JPEG)
               ▼
┌────────────────────────────────────────┐
│ ROUTER (NO internet!)                  │
│  - Pre-downloaded models in vlm_models/│
│  - Runs inference                      │
│  - Returns results via HTTP            │
└────────────────────────────────────────┘
```

## Advantages

✅ **No internet required** on inference device  
✅ **Guaranteed model version** - no unexpected updates  
✅ **Faster startup** - no download wait  
✅ **Works on isolated networks** - perfect for routers  
✅ **More secure** - no external connections  

## Troubleshooting

**"Model cache directory does not exist"**
- Check the path: `ls -la ./vlm_models`
- Verify config.json exists: `ls ./vlm_models/config.json`

**"Out of memory"**
- Use 4-bit quantization: `--quantization 4bit`
- Or run on CPU: `--device cpu`

**Server won't start**
- Check Python version: `python3 --version` (need 3.8+)
- Install dependencies: `pip3 install torch transformers flask`

## File Sizes

- **8-bit model:** ~3.5 GB
- **4-bit model:** ~2.0 GB
- **Full precision:** ~6.5 GB

Choose based on your storage and memory constraints.

## See Also

- `OFFLINE_DEPLOYMENT.md` - Detailed deployment guide
- `COMPLETE_END_TO_END_GUIDE.md` - Full system documentation
- `standalone_vlm_server.py` - Server source code
- `download_vlm_models.py` - Model downloader
