#!/usr/bin/env python3
"""
ROS2 to Network Bridge Node with YOLO Integration
==================================================
Runs on TurtleBot3 (Jetson Xavier NX).
Subscribes to camera images and YOLO detections from ROS2,
sends them to remote VLM server via HTTP,
receives inference results, and publishes back to ROS2.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
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
    with YOLO integration for enhanced object detection
    """
    
    def __init__(self):
        super().__init__('ros2_network_bridge')
        
        # Parameters
        self.declare_parameter('vlm_server_url', 'http://10.43.174.30:5000')
        self.declare_parameter('inference_rate', 2.0)
        self.declare_parameter('timeout', 15.0)
        self.declare_parameter('prompt', 'Describe this image in detail.')
        self.declare_parameter('show_preview', True)
        self.declare_parameter('use_yolo', True)
        
        self.vlm_server_url = self.get_parameter('vlm_server_url').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.prompt = self.get_parameter('prompt').value
        self.show_preview = self.get_parameter('show_preview').value
        self.use_yolo = self.get_parameter('use_yolo').value
        
        self.get_logger().info('='*70)
        self.get_logger().info('ROS2-Network Bridge with YOLO')
        self.get_logger().info('='*70)
        self.get_logger().info(f'VLM Server: {self.vlm_server_url}')
        self.get_logger().info(f'Inference Rate: {self.inference_rate} Hz')
        self.get_logger().info(f'YOLO Guidance: {self.use_yolo}')
        self.get_logger().info(f'Show Preview: {self.show_preview}')
        self.get_logger().info('='*70)
        
        # Initialize
        self.bridge = CvBridge()
        self.last_inference_time = 0.0
        self.inference_interval = 1.0 / self.inference_rate
        self.latest_yolo_detection = None
        
        # QoS Profile - Must match camera publisher (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to camera images from ROS2
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        # Subscribe to YOLO detections
        if self.use_yolo:
            self.yolo_sub = self.create_subscription(
                String,
                '/yolo/detections',
                self.yolo_callback,
                10
            )
            self.get_logger().info('✓ YOLO subscription active')
        
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
        self.last_result = "Waiting for first inference..."
        
        self.get_logger().info('✅ Bridge ready! Waiting for images...')
    
    def yolo_callback(self, msg: String):
        """Receive YOLO detections and store best detection"""
        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])
            
            if detections:
                # Store highest confidence detection
                self.latest_yolo_detection = max(detections, key=lambda x: x['confidence'])
                
                self.get_logger().info(
                    f"YOLO: {self.latest_yolo_detection['class']} "
                    f"@ {self.latest_yolo_detection['confidence']:.2f}",
                    throttle_duration_sec=3.0
                )
        except Exception as e:
            self.get_logger().error(f'YOLO callback error: {e}')
    
    def image_callback(self, msg: Image):
        """
        Callback for ROS2 image messages.
        Throttles inference requests based on configured rate.
        """
        self.frame_count += 1
        
        # Log first frame
        if self.frame_count == 1:
            self.get_logger().info('✓ First camera frame received!')
            self.get_logger().info(f'  Size: {msg.width}x{msg.height}')
        
        # Log every 100 frames
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Frames: {self.frame_count} | Inferences: {self.inference_count}'
            )
        
        current_time = time.time()
        
        # Throttle inference requests
        if current_time - self.last_inference_time < self.inference_interval:
            return
        
        self.last_inference_time = current_time
        
        self.get_logger().info(f'Processing frame {self.frame_count}...')
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Show preview window if enabled
            if self.show_preview:
                display_image = cv_image.copy()
                font = cv2.FONT_HERSHEY_SIMPLEX
                
                # Status
                status = f"Frame: {self.frame_count} | Inferences: {self.inference_count}"
                cv2.putText(display_image, status, (10, 30), 
                           font, 0.6, (0, 255, 0), 2)
                
                # YOLO info
                if self.latest_yolo_detection:
                    yolo_info = f"YOLO: {self.latest_yolo_detection['class']} ({self.latest_yolo_detection['confidence']:.2f})"
                    cv2.putText(display_image, yolo_info, (10, 60),
                               font, 0.6, (255, 255, 0), 2)
                    
                    # Draw YOLO bbox
                    bbox = self.latest_yolo_detection['bbox']
                    cv2.rectangle(display_image, 
                                 (bbox['x1'], bbox['y1']),
                                 (bbox['x2'], bbox['y2']),
                                 (0, 255, 255), 2)
                
                # Last result
                result_lines = []
                if len(self.last_result) > 80:
                    words = self.last_result.split()
                    line = ""
                    for word in words:
                        if len(line + word) < 80:
                            line += word + " "
                        else:
                            result_lines.append(line)
                            line = word + " "
                    if line:
                        result_lines.append(line)
                else:
                    result_lines = [self.last_result]
                
                y_offset = display_image.shape[0] - 20 - (len(result_lines[:3]) * 25)
                for i, line in enumerate(result_lines[:3]):
                    cv2.putText(display_image, line, (10, y_offset + i * 25), 
                               font, 0.5, (0, 255, 255), 1)
                
                cv2.imshow('VLM Bridge - Camera Feed', display_image)
                cv2.waitKey(1)
            
            # Convert to PIL Image
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            
            # Send to remote VLM server
            result = self.send_inference_request(pil_image, self.prompt)
            
            if result:
                self.last_result = result
                
                # Publish result
                result_msg = String()
                result_msg.data = result
                self.result_pub.publish(result_msg)
                
                self.inference_count += 1
                self.get_logger().info(f'✓ Result: {result[:100]}...')
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Image callback error: {e}')
    
    def send_inference_request(self, image: PILImage.Image, prompt: str) -> str:
        """
        Send image to remote VLM server via HTTP POST with YOLO guidance.
        """
        try:
            # Encode image as base64
            buffered = BytesIO()
            image.save(buffered, format="JPEG", quality=85)
            img_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
            
            # Prepare payload
            payload = {
                'image': img_base64,
                'prompt': prompt,
                'timestamp': time.time()
            }
            
            # Add YOLO guidance if available
            if self.use_yolo and self.latest_yolo_detection:
                payload['object_class'] = self.latest_yolo_detection['class']
                payload['yolo_confidence'] = self.latest_yolo_detection['confidence']
                payload['bbox'] = self.latest_yolo_detection['bbox']
                
                self.get_logger().info(
                    f"→ With YOLO: {payload['object_class']} "
                    f"@ {payload['yolo_confidence']:.2f}"
                )
            else:
                payload['object_class'] = 'object'
            
            # Send HTTP POST
            self.get_logger().info(f'→ POST to {self.vlm_server_url}/analyze')
            start_time = time.time()
            
            response = requests.post(
                f'{self.vlm_server_url}/analyze',
                json=payload,
                timeout=self.timeout
            )
            
            elapsed_ms = (time.time() - start_time) * 1000
            
            if response.status_code == 200:
                result = response.json()
                self.get_logger().info(f'✓ Response in {elapsed_ms:.0f}ms')
                
                # Extract result
                if 'result' in result:
                    return result.get('result', 'No result')
                elif 'action' in result:
                    # Enhanced VLM response
                    return f"{result.get('action', '?')}: {result.get('reasoning', '')[:100]}"
                else:
                    return str(result)
            else:
                self.get_logger().error(f'Server error: {response.status_code}')
                return None
                
        except requests.exceptions.Timeout:
            self.get_logger().error(f'Timeout ({self.timeout}s)')
            return None
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f'Cannot connect to {self.vlm_server_url}')
            return None
        except Exception as e:
            self.get_logger().error(f'Request error: {e}')
            return None


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = ROS2NetworkBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()