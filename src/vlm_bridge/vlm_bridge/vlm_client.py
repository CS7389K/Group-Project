#!/usr/bin/env python3
"""
VLM Client Node
================
Example client for testing VLM server.
Subscribes to camera and periodically queries the VLM.

Usage:
    ros2 run vlm_bridge vlm_client --ros-args -p prompt:="What do you see?"
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage


class VLMClientNode(Node):
    """
    VLM Client Node - Test client for VLM server
    """
    
    def __init__(self):
        super().__init__('vlm_client')
        
        # Parameters
        self.declare_parameter('prompt', 'Describe this image in detail.')
        self.declare_parameter('query_rate', 0.5)  # Hz
        
        self.prompt = self.get_parameter('prompt').value
        self.query_rate = self.get_parameter('query_rate').value
        
        self.get_logger().info(f'VLM Client initialized')
        self.get_logger().info(f'  Prompt: "{self.prompt}"')
        self.get_logger().info(f'  Query rate: {self.query_rate} Hz')
        
        # Initialize
        self.bridge = CvBridge()
        self.current_image = None
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Subscribe to VLM responses
        self.response_sub = self.create_subscription(
            String,
            '/vlm/response',
            self.response_callback,
            10
        )
        
        # Timer for periodic queries
        timer_period = 1.0 / self.query_rate
        self.timer = self.create_timer(timer_period, self.query_timer_callback)
        
        self.get_logger().info('VLM Client ready!')
    
    def image_callback(self, msg: Image):
        """Store latest image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')
    
    def response_callback(self, msg: String):
        """Handle VLM responses"""
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'VLM Response:')
        self.get_logger().info(f'{msg.data}')
        self.get_logger().info('=' * 70)
    
    def query_timer_callback(self):
        """Periodic query trigger"""
        if self.current_image is None:
            self.get_logger().warn('No image available yet')
            return
        
        self.get_logger().info(f'Sending query: "{self.prompt}"')
        # In a full implementation, this would call a service
        # For now, this is a placeholder


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = VLMClientNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down VLM Client...")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
