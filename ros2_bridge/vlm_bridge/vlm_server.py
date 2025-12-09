#!/usr/bin/env python3
"""
VLM Server Node
================
Provides VLM inference as a ROS2 service.
Runs Moondream2 model and responds to vision-language queries.

Service: /vlm/query (vision_msgs/Query)
Input: Image + text prompt
Output: VLM response with reasoning
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
from PIL import Image as PILImage
import time
import gc
import os

# Memory optimization
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128,expandable_segments:True'


def cleanup_memory():
    """Force garbage collection and CUDA cache cleanup"""
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


class VLMServerNode(Node):
    """
    VLM Server Node - Provides Moondream2 inference service
    """
    
    def __init__(self):
        super().__init__('vlm_server')
        
        # Parameters
        self.declare_parameter('model_id', 'vikhyatk/moondream2')
        self.declare_parameter('quantization', '8bit')  # '8bit', '4bit', or 'none'
        self.declare_parameter('device', 'cuda')  # 'cuda' or 'cpu'
        
        self.model_id = self.get_parameter('model_id').value
        self.quantization = self.get_parameter('quantization').value
        self.device = self.get_parameter('device').value
        
        self.get_logger().info(f'Initializing VLM Server...')
        self.get_logger().info(f'  Model: {self.model_id}')
        self.get_logger().info(f'  Quantization: {self.quantization}')
        self.get_logger().info(f'  Device: {self.device}')
        
        # Initialize
        self.bridge = CvBridge()
        self.model = None
        self.tokenizer = None
        
        # Load model
        self.load_model()
        
        # Subscriber for image stream (for continuous monitoring)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for VLM responses
        self.response_pub = self.create_publisher(
            String,
            '/vlm/response',
            10
        )
        
        self.current_image = None
        
        self.get_logger().info('VLM Server ready!')
    
    def load_model(self):
        """Load Moondream2 model with specified configuration"""
        from transformers import AutoModelForCausalLM, AutoTokenizer
        
        self.get_logger().info('Loading Moondream2 VLM...')
        cleanup_memory()
        
        load_kwargs = {
            'trust_remote_code': True,
            'low_cpu_mem_usage': True,
        }
        
        # Apply quantization settings
        if self.quantization == '8bit':
            self.get_logger().info('Applying 8-bit quantization...')
            load_kwargs.update({
                'load_in_8bit': True,
                'device_map': 'auto',
                'torch_dtype': torch.float16,
                'llm_int8_enable_fp32_cpu_offload': True,
                'llm_int8_skip_modules': ['vision_encoder', 'vision', 'input_layernorm']
            })
        elif self.quantization == '4bit':
            self.get_logger().info('Applying 4-bit quantization...')
            load_kwargs.update({
                'load_in_4bit': True,
                'device_map': 'auto',
                'torch_dtype': torch.float16,
            })
        else:
            self.get_logger().info('No quantization (FP16)...')
            load_kwargs.update({
                'torch_dtype': torch.float16,
                'device_map': self.device,
            })
        
        # Load model
        self.model = AutoModelForCausalLM.from_pretrained(
            self.model_id,
            **load_kwargs
        )
        
        # Load tokenizer
        self.tokenizer = AutoTokenizer.from_pretrained(self.model_id)
        
        # Jetson stability patch for vision encoder
        if hasattr(self.model, 'vision_encoder'):
            self.get_logger().info('Applying FP32 patch to vision encoder...')
            self.model.vision_encoder.to(dtype=torch.float32)
        elif hasattr(self.model, 'vision'):
            self.get_logger().info('Applying FP32 patch to vision module...')
            self.model.vision.to(dtype=torch.float32)
        
        self.get_logger().info('Model loaded successfully!')
    
    def image_callback(self, msg: Image):
        """Store latest image for queries"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')
    
    def query_vlm(self, image: PILImage.Image, prompt: str) -> str:
        """Query the VLM with an image and prompt"""
        cleanup_memory()
        
        start_time = time.time()
        
        try:
            result = self.model.query(image, prompt)
            answer = result['answer'].strip()
            
            elapsed_ms = (time.time() - start_time) * 1000
            self.get_logger().info(f'Query completed in {elapsed_ms:.0f}ms')
            
            return answer
            
        except Exception as e:
            self.get_logger().error(f'VLM query error: {e}')
            return f"Error: {str(e)}"


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = VLMServerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down VLM Server...")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
