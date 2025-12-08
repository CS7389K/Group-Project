#!/usr/bin/env python3
"""
Camera Publisher Node for TurtleBot3 + Jetson Xavier NX
========================================================
Publishes RPi Camera v2/HQ images using hardware-accelerated GStreamer pipeline.

Topic: /camera/image_raw (sensor_msgs/Image)
Rate: 30 Hz
Hardware: Jetson Xavier NX + Raspberry Pi Camera Module v2/HQ
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    """
    Hardware-accelerated camera publisher using nvarguscamerasrc.
    Optimized for TurtleBot3 on Jetson Xavier NX.
    """
    
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('flip_method', 0)  # 0=none, 2=rotate-180
        
        # Get parameters
        width = self.get_parameter('camera_width').value
        height = self.get_parameter('camera_height').value
        fps = self.get_parameter('camera_fps').value
        flip = self.get_parameter('flip_method').value
        
        # GStreamer pipeline for Jetson Xavier NX + RPi Camera
        # Uses hardware acceleration (nvarguscamerasrc -> nvvidconv)
        self.gstreamer_pipeline = (
            f"nvarguscamerasrc ! "
            f"video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate={fps}/1 ! "
            f"nvvidconv flip-method={flip} ! "
            f"video/x-raw, width={width}, height={height}, format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! appsink"
        )
        
        self.get_logger().info(f'Initializing camera with pipeline:')
        self.get_logger().info(f'  Resolution: {width}x{height} @ {fps} FPS')
        self.get_logger().info(f'  Flip method: {flip}')
        
        # Open camera
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera! Check:')
            self.get_logger().error('  1. Camera cable connection')
            self.get_logger().error('  2. Camera detected: ls /dev/video*')
            self.get_logger().error('  3. GStreamer: gst-inspect-1.0 nvarguscamerasrc')
            raise RuntimeError('Camera initialization failed')
        
        self.get_logger().info('✅ Camera opened successfully')
        
        # QoS Profile: Best Effort for real-time streaming
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
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        
        self.get_logger().info(f'Publishing to /camera/image_raw at {fps} Hz')
    
    def timer_callback(self):
        """Capture and publish camera frame"""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame', throttle_duration_sec=5.0)
            return
        
        try:
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            
            self.publisher.publish(msg)
            
            self.frame_count += 1
            if self.frame_count % 300 == 0:  # Log every 10 seconds at 30 FPS
                self.get_logger().info(f'Published {self.frame_count} frames')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {e}')
    
    def __del__(self):
        """Cleanup on shutdown"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Camera released')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = CameraPublisher()
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
