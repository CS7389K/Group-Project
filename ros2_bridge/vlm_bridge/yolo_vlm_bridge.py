#!/usr/bin/env python3
"""
YOLO + VLM Integrated Bridge
=============================
Combines YOLO11 object detection with VLM reasoning for enhanced decision making.

Architecture:
1. Camera → YOLO11 (object detection + bounding boxes + dimension estimation)
2. Camera + YOLO data → VLM Server (cross-validation + detailed reasoning)
3. VLM decision → ROS2 publisher

Features:
- Cross-validation: YOLO class vs VLM object recognition
- Dimension estimation: YOLO bbox → real-world size estimates for VLM
- Multi-object awareness: YOLO detects all objects, VLM reasons about each
- Spatial context: Position, size, relationships between objects
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
from typing import Optional, Dict, List


class YOLOVLMBridgeNode(Node):
    """
    Integrated bridge combining YOLO detection with VLM reasoning
    """
    
    def __init__(self):
        super().__init__('yolo_vlm_bridge')
        
        # Parameters
        self.declare_parameter('vlm_server_url', 'http://10.43.174.30:5000')
        self.declare_parameter('inference_rate', 2.0)
        self.declare_parameter('timeout', 15.0)
        self.declare_parameter('show_preview', True)
        self.declare_parameter('yolo_required', False)  # Can work without YOLO
        
        self.vlm_server_url = self.get_parameter('vlm_server_url').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.show_preview = self.get_parameter('show_preview').value
        self.yolo_required = self.get_parameter('yolo_required').value
        
        self.get_logger().info('='*70)
        self.get_logger().info('YOLO + VLM Integrated Bridge Starting')
        self.get_logger().info('='*70)
        self.get_logger().info(f'VLM Server: {self.vlm_server_url}')
        self.get_logger().info(f'Inference Rate: {self.inference_rate} Hz')
        self.get_logger().info(f'YOLO Required: {self.yolo_required}')
        
        # State
        self.bridge = CvBridge()
        self.last_inference_time = 0.0
        self.inference_interval = 1.0 / self.inference_rate
        self.frame_count = 0
        self.inference_count = 0
        
        # YOLO detection cache
        self.latest_detections: Optional[Dict] = None
        self.latest_image: Optional[np.ndarray] = None
        self.detections_received = False
        
        # QoS Profile
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos
        )
        
        # Subscribe to YOLO detections
        self.yolo_sub = self.create_subscription(
            String,
            '/yolo/detections',
            self.yolo_callback,
            10
        )
        
        # Publish VLM+YOLO results
        self.result_pub = self.create_publisher(
            String,
            '/vlm/analysis_result',
            10
        )
        
        self.get_logger().info('✅ YOLO+VLM Bridge ready!')
        self.get_logger().info('Waiting for images and YOLO detections...')
        self.get_logger().info('='*70)
    
    def yolo_callback(self, msg: String):
        """Receive YOLO detections"""
        try:
            data = json.loads(msg.data)
            self.latest_detections = data
            self.detections_received = True
            
            if not hasattr(self, '_yolo_logged'):
                self.get_logger().info('✓ YOLO detections received')
                self._yolo_logged = True
                
        except Exception as e:
            self.get_logger().error(f'YOLO callback error: {e}')
    
    def image_callback(self, msg: Image):
        """Process images with YOLO+VLM"""
        self.frame_count += 1
        
        if self.frame_count == 1:
            self.get_logger().info('✓ First camera frame received')
        
        # Throttle inference
        current_time = time.time()
        if (current_time - self.last_inference_time) < self.inference_interval:
            return
        
        # Check YOLO availability
        if self.yolo_required and not self.detections_received:
            if self.frame_count % 100 == 0:
                self.get_logger().warn('Waiting for YOLO detections...')
            return
        
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_image = cv_image
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Process with VLM
            self.last_inference_time = current_time
            self.inference_count += 1
            
            self.get_logger().info(f'\n[Inference #{self.inference_count}]')
            
            # Send to VLM server
            result = self.send_vlm_request(pil_image, self.latest_detections)
            
            if result:
                # Publish result
                result_msg = String()
                result_msg.data = json.dumps(result)
                self.result_pub.publish(result_msg)
                
                # Visualize
                if self.show_preview:
                    self.visualize(cv_image, result)
                
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')
    
    def send_vlm_request(self, image: PILImage.Image, 
                        yolo_data: Optional[Dict]) -> Optional[Dict]:
        """
        Send image + YOLO data to VLM server for integrated analysis
        """
        try:
            # Encode image
            buffered = BytesIO()
            image.save(buffered, format="JPEG", quality=85)
            img_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
            
            # Prepare payload
            payload = {
                'image': img_base64,
                'timestamp': time.time(),
                'frame_id': self.frame_count
            }
            
            # Add YOLO data if available
            if yolo_data and 'detections' in yolo_data:
                detections = yolo_data['detections']
                payload['yolo_detections'] = detections
                payload['yolo_enabled'] = True
                
                # Select primary object (highest confidence or center-most)
                if detections:
                    primary = max(detections, key=lambda d: d['confidence'])
                    payload['primary_object'] = {
                        'class': primary['class'],
                        'confidence': primary['confidence'],
                        'bbox': primary['bbox'],
                        'estimated_size_mm': primary.get('estimated_size_mm', {}),
                        'spatial': primary.get('spatial', {})
                    }
                    
                    self.get_logger().info(
                        f"YOLO: {primary['class']} @ {primary['confidence']:.2f} "
                        f"(~{primary.get('estimated_size_mm', {}).get('max_dimension', '?')}mm)"
                    )
            else:
                payload['yolo_enabled'] = False
                self.get_logger().info('No YOLO data - VLM only mode')
            
            # Send request
            start_time = time.time()
            response = requests.post(
                f'{self.vlm_server_url}/analyze',
                json=payload,
                timeout=self.timeout
            )
            elapsed_ms = (time.time() - start_time) * 1000
            
            if response.status_code == 200:
                result = response.json()
                result['processing_time_ms'] = round(elapsed_ms, 0)
                
                self.get_logger().info(
                    f"VLM: {result.get('action', 'UNKNOWN')} - "
                    f"{result.get('reasoning', 'No reasoning')[:60]}..."
                )
                self.get_logger().info(f"Total time: {elapsed_ms:.0f}ms")
                
                # Cross-validation check
                if yolo_data and 'detections' in yolo_data and detections:
                    self.cross_validate(result, payload.get('primary_object'))
                
                return result
            else:
                self.get_logger().error(f'VLM server error: {response.status_code}')
                return None
                
        except requests.Timeout:
            self.get_logger().error(f'VLM request timeout ({self.timeout}s)')
            return None
        except Exception as e:
            self.get_logger().error(f'VLM request error: {e}')
            return None
    
    def cross_validate(self, vlm_result: Dict, yolo_primary: Optional[Dict]):
        """Cross-validate YOLO and VLM object recognition"""
        if not yolo_primary:
            return
        
        yolo_class = yolo_primary['class'].lower()
        vlm_material = vlm_result.get('material', 'unknown').lower()
        
        # Check for consistency
        consistent = False
        
        # Material mapping (YOLO classes to typical materials)
        material_map = {
            'bottle': ['plastic', 'glass'],
            'cup': ['plastic', 'glass', 'ceramic', 'paper'],
            'book': ['paper', 'wood'],
            'cell phone': ['plastic', 'metal', 'glass'],
            'laptop': ['metal', 'plastic'],
            'keyboard': ['plastic'],
            'mouse': ['plastic'],
            'scissors': ['metal'],
            'bowl': ['ceramic', 'glass', 'plastic', 'metal'],
            'fork': ['metal'],
            'knife': ['metal'],
            'spoon': ['metal']
        }
        
        expected_materials = material_map.get(yolo_class, [])
        if vlm_material in expected_materials:
            consistent = True
        
        # Log cross-validation result
        if consistent:
            self.get_logger().info(
                f"✓ Cross-validation: YOLO '{yolo_class}' ↔ VLM '{vlm_material}' MATCH"
            )
        else:
            self.get_logger().warn(
                f"⚠ Cross-validation: YOLO '{yolo_class}' vs VLM '{vlm_material}' - "
                f"expected {expected_materials}"
            )
    
    def visualize(self, image: np.ndarray, vlm_result: Dict):
        """Visualize YOLO+VLM results"""
        display = image.copy()
        h, w = display.shape[:2]
        
        # Draw YOLO boxes if available
        if self.latest_detections and 'detections' in self.latest_detections:
            for det in self.latest_detections['detections']:
                bbox = det['bbox']
                x1, y1 = bbox['x1'], bbox['y1']
                x2, y2 = bbox['x2'], bbox['y2']
                
                # Box
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # YOLO label
                label = f"{det['class']} {det['confidence']:.2f}"
                cv2.putText(display, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # VLM decision overlay
        action = vlm_result.get('action', 'UNKNOWN')
        material = vlm_result.get('material', 'unknown')
        
        # Color based on action
        color_map = {
            'GRASP': (0, 255, 0),      # Green
            'PUSH': (0, 255, 255),     # Yellow
            'AVOID': (0, 0, 255),      # Red
            'STOP': (0, 165, 255),     # Orange
            'IGNORE': (128, 128, 128)  # Gray
        }
        color = color_map.get(action, (255, 255, 255))
        
        # Top info panel
        cv2.rectangle(display, (0, 0), (w, 120), (0, 0, 0), -1)
        
        cv2.putText(display, f"Action: {action}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(display, f"Material: {material}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        reasoning = vlm_result.get('reasoning', '')[:50]
        cv2.putText(display, reasoning, (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        cv2.putText(display, f"Frame: {self.frame_count} | Inf: {self.inference_count}", 
                   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
        
        cv2.imshow('YOLO + VLM Analysis', display)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLOVLMBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
