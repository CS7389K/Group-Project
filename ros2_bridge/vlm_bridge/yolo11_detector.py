#!/usr/bin/env python3
"""
YOLO11 Detection Node with Jetson TLS Memory Fix
=================================================
CRITICAL: Import cv2 BEFORE torch on Jetson to avoid TLS memory issues!
"""

# JETSON TLS FIX: Import cv2 BEFORE torch/ultralytics
import cv2
import numpy as np

# Now safe to import torch-based libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import time


class YOLO11DetectionNode(Node):
    """YOLO11 object detection node"""
    
    def __init__(self):
        super().__init__('yolo11_detector')
        
        # Parameters
        self.declare_parameter('model_size', 'n')  # n/s/m/l/x (n=fastest)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('show_preview', True)
        
        model_size = self.get_parameter('model_size').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.show_preview = self.get_parameter('show_preview').value
        
        self.get_logger().info('='*70)
        self.get_logger().info('YOLO11 Detection Node Starting')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Model: YOLO11{model_size}')
        self.get_logger().info(f'Confidence Threshold: {self.conf_thresh}')
        
        # Load YOLO11
        self.model = self.load_yolo_model(model_size)
        
        # ROS2 setup
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # QoS matching camera
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
        
        # Publish detections
        self.detection_pub = self.create_publisher(
            String,
            '/yolo/detections',
            10
        )
        
        self.get_logger().info('✅ YOLO11 ready! Waiting for images...')
        self.get_logger().info('='*70)
    
    def load_yolo_model(self, size: str):
        """Load YOLO11 model"""
        from ultralytics import YOLO
        
        model_name = f'yolo11{size}.pt'
        self.get_logger().info(f'Loading {model_name}...')
        
        try:
            model = YOLO(model_name)
            self.get_logger().info('✓ Model loaded')
            
            # Warm-up
            self.get_logger().info('Warming up...')
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            _ = model(dummy, verbose=False)
            self.get_logger().info('✓ Ready')
            
            return model
            
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO11: {e}')
            raise
    
    def image_callback(self, msg: Image):
        """Process images with YOLO11"""
        self.frame_count += 1
        
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run YOLO
            results = self.model(
                cv_image,
                conf=self.conf_thresh,
                verbose=False
            )[0]
            
            # Extract detections
            detections = []
            
            if results.boxes is not None and len(results.boxes) > 0:
                img_height, img_width = cv_image.shape[:2]
                
                for box in results.boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # Calculate dimensions
                    width_px = int(x2 - x1)
                    height_px = int(y2 - y1)
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # Class and confidence
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    class_name = results.names[class_id]
                    
                    # Estimate real-world dimensions (assuming object ~50cm from camera)
                    # Using simple perspective scaling
                    focal_length_approx = 600  # pixels (typical webcam)
                    distance_cm = 50  # assumed distance
                    
                    # Real size estimation: size_real = (size_px * distance) / focal_length
                    width_mm = int((width_px * distance_cm * 10) / focal_length_approx)
                    height_mm = int((height_px * distance_cm * 10) / focal_length_approx)
                    
                    # Normalize position (0-1)
                    norm_x = center_x / img_width
                    norm_y = center_y / img_height
                    
                    detection = {
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': {
                            'x1': int(x1),
                            'y1': int(y1),
                            'x2': int(x2),
                            'y2': int(y2),
                            'width_px': width_px,
                            'height_px': height_px,
                            'center_x': center_x,
                            'center_y': center_y,
                            'normalized_x': round(norm_x, 3),
                            'normalized_y': round(norm_y, 3)
                        },
                        'estimated_size_mm': {
                            'width': width_mm,
                            'height': height_mm,
                            'max_dimension': max(width_mm, height_mm)
                        },
                        'spatial': {
                            'position': 'center' if 0.3 < norm_x < 0.7 else ('left' if norm_x < 0.3 else 'right'),
                            'vertical': 'middle' if 0.3 < norm_y < 0.7 else ('top' if norm_y < 0.3 else 'bottom'),
                            'relative_size': round((width_px * height_px) / (img_width * img_height), 3)
                        }
                    }
                    
                    detections.append(detection)
                
                # Publish detections
                det_msg = String()
                det_msg.data = json.dumps({
                    'timestamp': time.time(),
                    'frame_id': self.frame_count,
                    'detections': detections
                })
                self.detection_pub.publish(det_msg)
                
                # Log
                if self.frame_count % 30 == 0:
                    classes = [d['class'] for d in detections]
                    self.get_logger().info(
                        f'Frame {self.frame_count}: {len(detections)} objects: {classes}'
                    )
            
            # Visualization
            if self.show_preview and detections:
                self.visualize(cv_image, detections)
                
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
    
    def visualize(self, image, detections):
        """Draw boxes on image"""
        display = image.copy()
        
        for det in detections:
            bbox = det['bbox']
            x1, y1, x2, y2 = bbox['x1'], bbox['y1'], bbox['x2'], bbox['y2']
            
            # Box
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Label
            label = f"{det['class']} {det['confidence']:.2f}"
            cv2.putText(display, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Info
        info = f"Frame: {self.frame_count} | Objects: {len(detections)}"
        cv2.putText(display, info, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('YOLO11 Detections', display)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLO11DetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()