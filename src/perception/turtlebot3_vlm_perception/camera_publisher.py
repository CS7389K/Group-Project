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
        
        self.get_logger().info(f'Ready to publish to /camera/image_raw at {fps} Hz')
    
    def step(self):
        """Capture and publish camera frame - call this in a loop"""
        # Check if camera is still open
        if not self.capture.isOpened():
            self.get_logger().error("Camera is not open!")
            return
        
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
    
    def shutdown(self):
        """Clean up resources."""
        self.get_logger().info("Shutting down Camera Publisher...")
        
        # Release camera capture
        if self.capture is not None and self.capture.isOpened():
            self.capture.release()
            self.get_logger().info("Camera released")
            import time
            time.sleep(0.5)
        
        self.get_logger().info("Camera Publisher shutdown complete.")
    
    def __del__(self):
        """Destructor to ensure cleanup happens even if shutdown() isn't called."""
        try:
            if hasattr(self, 'capture') and self.capture is not None:
                if self.capture.isOpened():
                    self.capture.release()
        except Exception:
            pass  # Ignore errors during cleanup


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    publisher = CameraPublisher()
        
    try:
        while rclpy.ok():
            publisher.step()
            rclpy.spin_once(publisher, timeout_sec=0.01)
    except KeyboardInterrupt:
        print("\nShutting down Camera Publisher...")
    except Exception as e:
        print(f"Error in Camera Publisher: {e}")
    finally:
        if publisher:
            publisher.shutdown()
            publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
