#!/usr/bin/env python3
"""
Camera Publisher with USB/CSI Support
======================================
Supports both CSI (GStreamer) and USB (V4L2) cameras with automatic fallback.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('flip_method', 0)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('camera_device', '/dev/video1')
        self.declare_parameter('force_v4l2', False)
        
        width = self.get_parameter('camera_width').value
        height = self.get_parameter('camera_height').value
        fps = self.get_parameter('camera_fps').value
        self.show_preview = self.get_parameter('show_preview').value
        device = self.get_parameter('camera_device').value
        force_v4l2 = self.get_parameter('force_v4l2').value
        
        self.get_logger().info(f'Camera Config: {width}x{height} @ {fps} FPS')
        self.get_logger().info(f'USB Device: {device}')
        
        self.capture = None
        self.camera_type = None
        
        # Try GStreamer first
        if not force_v4l2:
            self.get_logger().info('Trying GStreamer (CSI)...')
            gst = (
                f'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),'
                f'width={width},height={height},framerate={fps}/1,format=NV12 ! '
                f'nvvidconv ! video/x-raw,format=BGRx,width={width},height={height} ! '
                f'videoconvert ! video/x-raw,format=BGR ! appsink drop=1'
            )
            self.capture = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
            if self.capture.isOpened():
                self.camera_type = 'GStreamer-CSI'
                self.get_logger().info('✓ CSI camera OK')
            else:
                self.capture = None
        
        # Fallback to V4L2
        if not self.capture or not self.capture.isOpened():
            self.get_logger().info(f'Trying V4L2 (USB): {device}')
            if not os.path.exists(device):
                self.get_logger().error(f'{device} not found!')
                raise RuntimeError(f'No camera at {device}')
            
            self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not self.capture.isOpened():
                raise RuntimeError(f'Failed to open {device}')
            
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.capture.set(cv2.CAP_PROP_FPS, fps)
            
            self.camera_type = f'V4L2-USB-{device}'
            self.get_logger().info(f'✓ USB camera OK')
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos)
        self.bridge = CvBridge()
        self.frame_count = 0
        
        self.get_logger().info(f'Publishing with {self.camera_type}')
    
    def step(self):
        if not self.capture.isOpened():
            return
        
        ret, frame = self.capture.read()
        if not ret or frame is None:
            return

        try:
            if self.show_preview:
                cv2.imshow('Camera', frame)
                cv2.waitKey(1)
            
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.publisher.publish(msg)
            
            self.frame_count += 1
            if self.frame_count % 300 == 0:
                self.get_logger().info(f'{self.frame_count} frames')
        except Exception as e:
            self.get_logger().error(f'{e}')
    
    def shutdown(self):
        if self.show_preview:
            cv2.destroyAllWindows()
        if self.capture:
            self.capture.release()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        while rclpy.ok():
            node.step()
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
