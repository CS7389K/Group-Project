# Offline VLM Server Deployment Guide

This guide explains how to set up the VLM server for offline operation on a router or air-gapped device.

## Overview

The VLM server can run completely offline by pre-downloading all model files and weights. This is ideal for deployment on:
- Small routers without internet access
- Air-gapped systems
- Edge devices with limited connectivity
- Environments where you want guaranteed model versions

## Two-Step Process

### Step 1: Download Models (On Internet-Connected Machine)

First, download all model files on a machine with internet access:

```bash
# Install dependencies
pip3 install -r standalone_requirements.txt

# Download models to local directory
python3 download_vlm_models.py --output-dir ./vlm_models
```

**Options:**
```bash
# Download with 4-bit quantization (smaller size)
python3 download_vlm_models.py --output-dir ./vlm_models --use-4bit

# Download with 8-bit quantization (default, balanced)
python3 download_vlm_models.py --output-dir ./vlm_models --use-8bit

# Custom output directory
python3 download_vlm_models.py --output-dir /mnt/models
```

**Expected size:** 2-4 GB depending on quantization

The script will:
1. Download tokenizer files
2. Download model weights
3. Save everything to the specified directory
4. Show total size and confirm completion

### Step 2: Transfer and Run (On Offline Device)

Transfer the entire model directory to your offline device:

```bash
# Option 1: Using SCP
scp -r ./vlm_models user@router-ip:/path/to/destination/

# Option 2: Using USB drive
# Copy vlm_models/ folder to USB, then mount and copy on target device

# Option 3: Using rsync
rsync -avz ./vlm_models/ user@router-ip:/path/to/destination/vlm_models/
```

On the offline device, run the server pointing to the local model cache:

```bash
# Run with local models (NO internet required)
python3 standalone_vlm_server.py \
    --model-cache-dir ./vlm_models \
    --host 0.0.0.0 \
    --port 5000 \
    --device cuda
```

## Complete Offline Deployment Example

### On Internet-Connected PC:

```bash
# 1. Clone repository
git clone https://github.com/CS7389K/Group-Project.git
cd Group-Project

# 2. Install dependencies
pip3 install -r standalone_requirements.txt

# 3. Download models (this takes a few minutes)
python3 download_vlm_models.py --output-dir ./vlm_models

# 4. Package everything for transfer
tar czf vlm_offline_package.tar.gz \
    standalone_vlm_server.py \
    standalone_requirements.txt \
    vlm_models/

# 5. Transfer to router/offline device
scp vlm_offline_package.tar.gz user@192.168.1.1:/tmp/
```

### On Offline Router/Device:

```bash
# 1. Extract package
cd /opt  # or wherever you want to install
tar xzf /tmp/vlm_offline_package.tar.gz

# 2. Install Python dependencies (one-time, if not already installed)
pip3 install -r standalone_requirements.txt

# 3. Run VLM server
python3 standalone_vlm_server.py \
    --model-cache-dir ./vlm_models \
    --host 0.0.0.0 \
    --port 5000 \
    --device cuda
```

## Verification

Test the offline server:

```bash
# From Jetson or another machine on the network
python3 tools/test_network_bridge.py --server-url http://192.168.1.1:5000
```

Or use curl:

```bash
curl http://192.168.1.1:5000/health
```

## Router-Specific Considerations

### Memory Requirements
- **Minimum:** 4 GB RAM for 4-bit quantization
- **Recommended:** 8 GB RAM for 8-bit quantization
- **Full precision:** 16+ GB RAM

### Storage Requirements
- **Model files:** 2-4 GB
- **System overhead:** 500 MB
- **Total:** ~3-5 GB free space

### CPU vs GPU
```bash
# If router has NVIDIA GPU (rare)
python3 standalone_vlm_server.py --model-cache-dir ./vlm_models --device cuda

# For CPU-only (slower but works on any router)
python3 standalone_vlm_server.py --model-cache-dir ./vlm_models --device cpu
```

### Running as Service (Systemd)

Create `/etc/systemd/system/vlm-server.service`:

```ini
[Unit]
Description=Offline VLM Inference Server
After=network.target

[Service]
Type=simple
User=vlm
WorkingDirectory=/opt/vlm
ExecStart=/usr/bin/python3 standalone_vlm_server.py --model-cache-dir ./vlm_models --host 0.0.0.0 --port 5000
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable vlm-server
sudo systemctl start vlm-server
sudo systemctl status vlm-server
```

## Troubleshooting

### "Model files not found"
- Verify the `vlm_models/` directory exists
- Check that it contains `config.json`, `pytorch_model.bin`, and tokenizer files
- Ensure the path passed to `--model-cache-dir` is correct

### "Out of memory"
- Use 4-bit quantization: `--use-4bit`
- Reduce batch size (model default is already minimal)
- Consider upgrading router RAM or using CPU

### "Module not found" errors
- Install dependencies: `pip3 install -r standalone_requirements.txt`
- Verify Python version: `python3 --version` (need 3.8+)

### Slow inference on CPU
- CPU inference is 10-100x slower than GPU
- Consider using a device with GPU, or increase `inference_rate` on Jetson to reduce frequency

## Network Configuration

### Firewall Rules
```bash
# Allow VLM server port
sudo ufw allow 5000/tcp

# Or for iptables
sudo iptables -A INPUT -p tcp --dport 5000 -j ACCEPT
```

### Static IP Recommendation
Set a static IP on your router for consistent addressing from the Jetson.

## Complete Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ JETSON (TurtleBot3) - with ROS2 and internet               │
│                                                              │
│  ┌──────────────┐    ┌──────────────────────────────────┐  │
│  │   Camera     │───>│  ROS2 Network Bridge             │  │
│  │  Publisher   │    │  - Subscribes to /camera/image   │  │
│  └──────────────┘    │  - Sends HTTP POST to router     │  │
│                      │  - Publishes /vlm/inference_result│ │
│                      └──────────────┬───────────────────┘  │
└─────────────────────────────────────┼──────────────────────┘
                                      │
                                      │ HTTP POST
                                      │ (Base64 JPEG)
                                      ▼
                   ┌──────────────────────────────────────┐
                   │ ROUTER (Offline) - NO internet       │
                   │                                      │
                   │  ┌────────────────────────────────┐ │
                   │  │  Standalone VLM Server         │ │
                   │  │  - Local model cache           │ │
                   │  │  - Moondream2 inference        │ │
                   │  │  - Returns JSON response       │ │
                   │  └────────────────────────────────┘ │
                   └──────────────────────────────────────┘
```

## Summary

**Advantages of offline deployment:**
- ✓ No internet required on inference device
- ✓ Guaranteed model version (no unexpected updates)
- ✓ Faster startup (no download wait)
- ✓ Works in air-gapped environments
- ✓ More secure (isolated network)

**Workflow:**
1. Download models once with internet: `download_vlm_models.py`
2. Transfer to offline device: `scp` or USB
3. Run with local cache: `--model-cache-dir ./vlm_models`
4. No internet needed ever again!

The model cache is completely portable - copy it to any device and it will work.
