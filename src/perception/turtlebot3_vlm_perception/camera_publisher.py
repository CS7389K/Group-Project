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
        # Based on working pipeline from CS7389K/Milestone-4
        # Key differences from broken version:
        # 1. sensor-id=0 parameter
        # 2. Capture at native resolution, then scale down
        # 3. drop=1 in appsink to prevent buffering
        self.gst_pipeline= (
            'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), ',
            'width={image_width},height={image_height},framerate=30/1,format=NV12 ! ',
            'nvvidconv ! video/x-raw,format=BGRx,width={image_width},height={image_height} ! ',
            'videoconvert ! video/x-raw,format=BGR ! appsink drop=1'
        )
        self.gst_pipeline = ''.join(self.gst_pipeline).format(image_width=640, image_height=480)
        
        self.get_logger().info('Initializing camera with pipeline:')
        self.get_logger().info(f'  Resolution: {width}x{height} @ {fps} FPS')
        self.get_logger().info(f'  Flip method: {flip}')
        self.get_logger().info(f'  Pipeline: {self.gst_pipeline}')
       
        print(f"GStreamer pipeline: {self.gst_pipeline}")
        self.capture = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)
        print(f"Camera opened: {self.capture.isOpened()}")
        if not self.capture.isOpened():
            raise RuntimeError("Error: Unable to open camera")
        
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
    
    # def timer_callback(self):
    #     """Capture and publish camera frame"""
    #     ret, frame = self.cap.read()
        
    #     if not ret:
    #         self.get_logger().warn('Failed to capture frame', throttle_duration_sec=5.0)
    #         return
        
    #     # Check if frame is valid (not all white/black)
    #     if frame is None or frame.size == 0:
    #         self.get_logger().warn('Empty frame received', throttle_duration_sec=5.0)
    #         return
            
    #     try:
    #         # Convert to ROS Image message
    #         msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         msg.header.frame_id = 'camera_link'
            
    #         self.publisher.publish(msg)
            
    #         self.frame_count += 1
    #         if self.frame_count % 300 == 0:  # Log every 10 seconds at 30 FPS
    #             self.get_logger().info(f'Published {self.frame_count} frames')
            
    #         # Debug: Log first frame stats
    #         if self.frame_count == 1:
    #             import numpy as np
    #             self.get_logger().info(f'First frame stats: shape={frame.shape}, '
    #                                   f'mean={np.mean(frame):.1f}, '
    #                                   f'min={np.min(frame)}, max={np.max(frame)}')
                
    #     except Exception as e:
    #         self.get_logger().error(f'Error publishing frame: {e}')
    
    # def __del__(self):
    #     """Cleanup on shutdown"""
    #     if hasattr(self, 'cap') and self.cap.isOpened():
    #         self.cap.release()
    #         self.get_logger().info('Camera released')
    
    def timer_callback(self):
        """Capture and publish camera frame"""
        ret, frame = self.capture.read()

        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        if frame is None or frame.size == 0:
            self.get_logger().warn('Empty frame received')
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'

            self.publisher.publish(msg)

            self.frame_count += 1
            if self.frame_count % 300 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')

            if self.frame_count == 1:
                import numpy as np
                self.get_logger().info(
                    f'First frame stats: shape={frame.shape}, '
                    f'mean={np.mean(frame):.1f}, '
                    f'min={np.min(frame)}, max={np.max(frame)}'
                )

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
