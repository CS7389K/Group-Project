#!/usr/bin/env python3
"""
Camera Publisher Node for Network Bridge
=========================================
Simple V4L2 camera publisher for the network bridge package.
Publishes images to /camera/image_raw for the ROS2-Network Bridge.

This is a self-contained camera node - no external dependencies needed!

Usage:
    ros2 run vlm_bridge camera_publisher
    ros2 run vlm_bridge camera_publisher --ros-args -p device:=1
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class CameraPublisherNode(Node):
    """Simple camera publisher using OpenCV VideoCapture"""
    
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters
        self.declare_parameter('device', 0)  # /dev/video0
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('show_preview', False)
        
        # Get parameters
        device = self.get_parameter('device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.show_preview = self.get_parameter('show_preview').value
        
        self.get_logger().info('Initializing Camera Publisher...')
        self.get_logger().info(f'  Device: /dev/video{device}')
        self.get_logger().info(f'  Resolution: {width}x{height}')
        self.get_logger().info(f'  FPS: {fps}')
        
        # Open camera
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error('❌ Failed to open camera!')
            self.get_logger().error('Troubleshooting:')
            self.get_logger().error('  1. List cameras: ls -la /dev/video*')
            self.get_logger().error('  2. Check permissions: sudo usermod -aG video $USER')
            self.get_logger().error('  3. Try different device: --ros-args -p device:=1')
            self.get_logger().error('  4. Test with: v4l2-ctl --list-devices')
            raise RuntimeError('Camera initialization failed')
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        
        # Verify settings
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.get_logger().info('✅ Camera opened successfully')
        self.get_logger().info(f'  Actual: {actual_width}x{actual_height} @ {actual_fps} FPS')
        
        # QoS Profile - Best effort for real-time performance
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            qos_profile
        )
        
        # Bridge and state
        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_log_time = time.time()
        
        # Timer for publishing
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera Publisher ready!')
        self.get_logger().info('Publishing to: /camera/image_raw')
    
    def timer_callback(self):
        """Read and publish camera frame"""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return
        
        try:
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            
            # Publish
            self.publisher.publish(msg)
            self.frame_count += 1
            
            # Log status every 5 seconds
            current_time = time.time()
            if current_time - self.last_log_time >= 5.0:
                fps = self.frame_count / (current_time - self.last_log_time)
                self.get_logger().info(f'Publishing at {fps:.1f} FPS (frame {self.frame_count})')
                self.last_log_time = current_time
                self.frame_count = 0
            
            # Show preview if enabled
            if self.show_preview:
                cv2.imshow('Camera Preview', frame)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {e}')
    
    def __del__(self):
        """Cleanup on shutdown"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if self.show_preview:
            cv2.destroyAllWindows()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = CameraPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down camera...")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
