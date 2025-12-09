#!/usr/bin/env python3
"""
Standalone VLM HTTP Server (No ROS2)
=====================================
Runs on remote computer WITHOUT ROS2 dependency.
Receives images via HTTP POST, performs Moondream2 inference, returns results.

This server can run on any machine with Python + PyTorch + Transformers.
NO ROS2 installation required!

Usage:
    python3 standalone_vlm_server.py --port 5000 --host 0.0.0.0
"""

from __future__ import annotations

from flask import Flask, request, jsonify
import base64
from io import BytesIO
from PIL import Image as PILImage
import torch
import gc
import os
import time
import argparse
from typing import Optional
from pathlib import Path

# Memory optimization
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128,expandable_segments:True'


def cleanup_memory():
    """Force garbage collection and CUDA cache cleanup"""
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


class StandaloneVLMServer:
    """
    Standalone VLM server without ROS2 dependencies.
    Serves inference via HTTP REST API.
    """
    
    def __init__(self, model_id: str = 'vikhyatk/moondream2', 
                 quantization: str = '8bit', device: str = 'cuda',
                 model_cache_dir: Optional[str] = None):
        self.model_id = model_id
        self.quantization = quantization
        self.device = device
        self.model_cache_dir = model_cache_dir
        self.model = None
        self.tokenizer = None
        self.inference_count = 0
        
        print(f'Initializing Standalone VLM Server...')
        print(f'  Model: {self.model_id}')
        print(f'  Quantization: {self.quantization}')
        print(f'  Device: {self.device}')
        if self.model_cache_dir:
            print(f'  Local Cache: {self.model_cache_dir} (OFFLINE MODE)')
        else:
            print(f'  Mode: Online (will download from HuggingFace)')
        
        self.load_model()
        print('VLM Server ready!')
    
    def load_model(self):
        """Load Moondream2 model with specified configuration"""
        from transformers import AutoModelForCausalLM, AutoTokenizer
        
        print('Loading Moondream2 VLM...')
        cleanup_memory()
        
        # Determine model path (local cache or HuggingFace)
        if self.model_cache_dir:
            model_path = Path(self.model_cache_dir).expanduser().resolve()
            if not model_path.exists():
                raise ValueError(f"Model cache directory does not exist: {model_path}")
            print(f'Using local model cache: {model_path}')
            model_source = str(model_path)
            local_files_only = True
        else:
            print(f'Downloading from HuggingFace: {self.model_id}')
            model_source = self.model_id
            local_files_only = False
        
        load_kwargs = {
            'trust_remote_code': True,
            'low_cpu_mem_usage': True,
            'local_files_only': local_files_only,
        }
        
        # Apply quantization settings
        if self.quantization == '8bit':
            print('Applying 8-bit quantization...')
            load_kwargs.update({
                'load_in_8bit': True,
                'device_map': 'auto',
                'torch_dtype': torch.float16,
                'llm_int8_enable_fp32_cpu_offload': True,
                'llm_int8_skip_modules': ['vision_encoder', 'vision', 'input_layernorm']
            })
        elif self.quantization == '4bit':
            print('Applying 4-bit quantization...')
            load_kwargs.update({
                'load_in_4bit': True,
                'device_map': 'auto',
                'torch_dtype': torch.float16,
            })
        else:
            print('No quantization (FP16)...')
            load_kwargs.update({
                'torch_dtype': torch.float16,
                'device_map': self.device,
            })
        
        # Load model
        self.model = AutoModelForCausalLM.from_pretrained(
            model_source,
            **load_kwargs
        )
        
        # Load tokenizer
        self.tokenizer = AutoTokenizer.from_pretrained(
            model_source,
            trust_remote_code=True,
            local_files_only=local_files_only
        )
        
        # Jetson stability patch for vision encoder
        if hasattr(self.model, 'vision_encoder'):
            print('Applying FP32 patch to vision encoder...')
            self.model.vision_encoder.to(dtype=torch.float32)
        elif hasattr(self.model, 'vision'):
            print('Applying FP32 patch to vision module...')
            self.model.vision.to(dtype=torch.float32)
        
        print('Model loaded successfully!')
    
    def infer(self, image: PILImage.Image, prompt: str) -> str:
        """
        Perform VLM inference on image.
        
        Args:
            image: PIL Image
            prompt: Text prompt
            
        Returns:
            Inference result string
        """
        cleanup_memory()
        
        start_time = time.time()
        
        try:
            result = self.model.query(image, prompt)
            answer = result['answer'].strip()
            
            elapsed_ms = (time.time() - start_time) * 1000
            self.inference_count += 1
            
            print(f'Inference #{self.inference_count} completed in {elapsed_ms:.0f}ms')
            print(f'  Prompt: {prompt}')
            print(f'  Result: {answer[:100]}...')
            
            return answer
            
        except Exception as e:
            print(f'Inference error: {e}')
            raise


# Create Flask app
app = Flask(__name__)
vlm_server: Optional[StandaloneVLMServer] = None


@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({
        'status': 'healthy',
        'model_loaded': vlm_server is not None,
        'inference_count': vlm_server.inference_count if vlm_server else 0
    })


@app.route('/inference', methods=['POST'])
def inference():
    """
    Main inference endpoint.
    
    Expected JSON payload:
    {
        "image": "<base64-encoded JPEG>",
        "prompt": "Describe this image",
        "timestamp": 1234567890.123
    }
    
    Returns JSON:
    {
        "result": "A photo of...",
        "inference_time_ms": 456.78,
        "timestamp": 1234567890.123
    }
    """
    if vlm_server is None:
        return jsonify({'error': 'VLM server not initialized'}), 500
    
    try:
        # Parse request
        data = request.get_json()
        
        if not data or 'image' not in data:
            return jsonify({'error': 'Missing image in request'}), 400
        
        img_base64 = data['image']
        prompt = data.get('prompt', 'Describe this image in detail.')
        request_timestamp = data.get('timestamp', time.time())
        
        # Decode image
        img_bytes = base64.b64decode(img_base64)
        image = PILImage.open(BytesIO(img_bytes))
        
        # Perform inference
        start_time = time.time()
        result = vlm_server.infer(image, prompt)
        inference_time_ms = (time.time() - start_time) * 1000
        
        # Return result
        return jsonify({
            'result': result,
            'inference_time_ms': inference_time_ms,
            'timestamp': request_timestamp,
            'server_timestamp': time.time()
        })
        
    except Exception as e:
        print(f'Inference endpoint error: {e}')
        return jsonify({'error': str(e)}), 500


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Standalone VLM HTTP Server (No ROS2)')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='Host to bind to (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=5000,
                       help='Port to listen on (default: 5000)')
    parser.add_argument('--model', type=str, default='vikhyatk/moondream2',
                       help='HuggingFace model ID (default: vikhyatk/moondream2)')
    parser.add_argument('--quantization', type=str, default='8bit',
                       choices=['8bit', '4bit', 'none'],
                       help='Quantization mode (default: 8bit)')
    parser.add_argument('--device', type=str, default='cuda',
                       choices=['cuda', 'cpu'],
                       help='Device to run on (default: cuda)')
    parser.add_argument('--model-cache-dir', type=str, default=None,
                       help='Path to pre-downloaded model cache (for offline use)')
    
    args = parser.parse_args()
    
    # Initialize VLM server
    global vlm_server
    vlm_server = StandaloneVLMServer(
        model_id=args.model,
        quantization=args.quantization,
        device=args.device,
        model_cache_dir=args.model_cache_dir
    )
    
    # Start Flask server
    print(f'\nStarting HTTP server on {args.host}:{args.port}')
    print(f'Inference endpoint: http://{args.host}:{args.port}/inference')
    print(f'Health check: http://{args.host}:{args.port}/health')
    print('\nPress Ctrl+C to stop\n')
    
    app.run(host=args.host, port=args.port, threaded=False)


if __name__ == '__main__':
    main()
