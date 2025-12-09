#!/usr/bin/env python3
"""
ROS2 to Network Bridge Node
============================
Runs on TurtleBot3 (Jetson Xavier NX).
Subscribes to camera images from ROS2, sends them to remote VLM server via HTTP,
receives inference results, and publishes back to ROS2.

Architecture:
  TurtleBot3 (ROS2) <---> Bridge Node <---> Remote VLM Server (HTTP, no ROS)
  
Topics:
  - Subscribes: /camera/image_raw (sensor_msgs/Image)
  - Publishes: /vlm/inference_result (std_msgs/String)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
import base64
import json
from io import BytesIO
from PIL import Image as PILImage
import time


class ROS2NetworkBridgeNode(Node):
    """
    Bridge between ROS2 (TurtleBot3) and remote VLM server (non-ROS)
    """
    
    def __init__(self):
        super().__init__('ros2_network_bridge')
        
        # Parameters
        self.declare_parameter('vlm_server_url', 'http://192.168.1.100:5000')  # Remote VLM server
        self.declare_parameter('inference_rate', 2.0)  # Hz - limit inference requests
        self.declare_parameter('timeout', 10.0)  # seconds
        self.declare_parameter('prompt', 'Describe this image in detail.')
        
        self.vlm_server_url = self.get_parameter('vlm_server_url').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.prompt = self.get_parameter('prompt').value
        
        self.get_logger().info('Initializing ROS2-Network Bridge...')
        self.get_logger().info(f'  VLM Server: {self.vlm_server_url}')
        self.get_logger().info(f'  Inference Rate: {self.inference_rate} Hz')
        self.get_logger().info(f'  Default Prompt: "{self.prompt}"')
        
        # Initialize
        self.bridge = CvBridge()
        self.last_inference_time = 0.0
        self.inference_interval = 1.0 / self.inference_rate
        
        # Subscribe to camera images from ROS2
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish inference results to ROS2
        self.result_pub = self.create_publisher(
            String,
            '/vlm/inference_result',
            10
        )
        
        # Statistics
        self.frame_count = 0
        self.inference_count = 0
        self.error_count = 0
        
        self.get_logger().info('ROS2-Network Bridge ready!')
        self.get_logger().info('Waiting for images on /camera/image_raw...')
    
    def image_callback(self, msg: Image):
        """
        Callback for ROS2 image messages.
        Throttles inference requests based on configured rate.
        """
        self.frame_count += 1
        
        current_time = time.time()
        
        # Throttle inference requests
        if current_time - self.last_inference_time < self.inference_interval:
            return
        
        self.last_inference_time = current_time
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to PIL Image
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            
            # Send to remote VLM server
            self.get_logger().info(f'Sending image to VLM server (frame {self.frame_count})...')
            result = self.send_inference_request(pil_image, self.prompt)
            
            if result:
                # Publish result back to ROS2
                result_msg = String()
                result_msg.data = result
                self.result_pub.publish(result_msg)
                
                self.inference_count += 1
                self.get_logger().info(f'Published inference result: {result[:100]}...')
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Image callback error: {e}')
    
    def send_inference_request(self, image: PILImage.Image, prompt: str) -> str:
        """
        Send image to remote VLM server via HTTP POST.
        
        Args:
            image: PIL Image
            prompt: Text prompt for VLM
            
        Returns:
            Inference result string, or None if failed
        """
        try:
            # Encode image as base64 JPEG
            buffered = BytesIO()
            image.save(buffered, format="JPEG", quality=85)
            img_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
            
            # Prepare request payload
            payload = {
                'image': img_base64,
                'prompt': prompt,
                'timestamp': time.time()
            }
            
            # Send HTTP POST request to VLM server
            start_time = time.time()
            response = requests.post(
                f'{self.vlm_server_url}/inference',
                json=payload,
                timeout=self.timeout
            )
            elapsed_ms = (time.time() - start_time) * 1000
            
            if response.status_code == 200:
                result = response.json()
                self.get_logger().info(f'Inference completed in {elapsed_ms:.0f}ms')
                return result.get('result', 'No result')
            else:
                self.get_logger().error(f'Server error: {response.status_code} - {response.text}')
                return None
                
        except requests.exceptions.Timeout:
            self.get_logger().error(f'Request timeout ({self.timeout}s)')
            return None
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f'Cannot connect to VLM server at {self.vlm_server_url}')
            return None
        except Exception as e:
            self.get_logger().error(f'Inference request error: {e}')
            return None


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = ROS2NetworkBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down ROS2-Network Bridge...")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
