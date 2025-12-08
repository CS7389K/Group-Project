#!/usr/bin/env python3
"""
Alternative Camera Publisher using V4L2 (Fallback)
===================================================
Use this if nvarguscamerasrc doesn't work with your camera.
This uses standard V4L2 driver instead of Nvidia's nvarguscamerasrc.

Replace camera_publisher.py with this file if you get white images.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisherV4L2(Node):
    """Camera publisher using V4L2 (standard Linux video driver)"""
    
    def __init__(self):
        super().__init__('camera_publisher_v4l2')
        
        # Declare parameters
        self.declare_parameter('camera_device', 0)  # /dev/video0
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        
        # Get parameters
        device = self.get_parameter('camera_device').value
        width = self.get_parameter('camera_width').value
        height = self.get_parameter('camera_height').value
        fps = self.get_parameter('camera_fps').value
        
        self.get_logger().info('Initializing camera with V4L2:')
        self.get_logger().info(f'  Device: /dev/video{device}')
        self.get_logger().info(f'  Resolution: {width}x{height} @ {fps} FPS')
        
        # Open camera using standard OpenCV VideoCapture
        self.cap = cv2.VideoCapture(device)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera! Check:')
            self.get_logger().error(f'  1. Camera detected: ls /dev/video*')
            self.get_logger().error(f'  2. Try different device: camera_device:=1')
            self.get_logger().error(f'  3. Check permissions: sudo usermod -aG video $USER')
            raise RuntimeError('Camera initialization failed')
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # Verify actual settings
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.get_logger().info('✅ Camera opened successfully')
        self.get_logger().info(f'  Actual: {actual_width}x{actual_height} @ {actual_fps} FPS')
        
        # QoS Profile
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
        
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # Timer
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        
        self.get_logger().info(f'Publishing to /camera/image_raw at {fps} Hz')
    
    def timer_callback(self):
        """Capture and publish camera frame"""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame', throttle_duration_sec=5.0)
            return
        
        if frame is None or frame.size == 0:
            self.get_logger().warn('Empty frame received', throttle_duration_sec=5.0)
            return
        
        try:
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            
            self.publisher.publish(msg)
            
            self.frame_count += 1
            if self.frame_count % 300 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
            
            # Debug first frame
            if self.frame_count == 1:
                import numpy as np
                self.get_logger().info(f'First frame: shape={frame.shape}, '
                                      f'mean={np.mean(frame):.1f}, '
                                      f'min={np.min(frame)}, max={np.max(frame)}')
                if np.mean(frame) > 250:
                    self.get_logger().warn('⚠️  Frame appears to be all white! Check camera.')
                    
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {e}')
    
    def __del__(self):
        """Cleanup"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Camera released')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = CameraPublisherV4L2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
