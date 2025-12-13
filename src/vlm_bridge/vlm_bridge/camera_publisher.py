#!/usr/bin/env python3
"""
Camera Publisher with USB/CSI Support
======================================
Supports both CSI (GStreamer) and USB (V4L2) cameras with automatic fallback.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class CameraGStreamer:
    """Camera capture using a GStreamer pipeline."""

    def __init__(self, pipeline: str, width=640, height=480):
        self.pipeline = pipeline
        self.width = width
        self.height = height
        self._cap = None

    def open(self):
        self._cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Unable to open GStreamer pipeline: {self.pipeline}")

    def read(self):
        if self._cap is None:
            raise RuntimeError("Camera not opened")
        return self._cap.read()

    def isOpened(self):
        return self._cap is not None and self._cap.isOpened()

    def release(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class CameraCV2:
    """Camera capture using OpenCV V4L2."""

    def __init__(self, device='/dev/video0', width=640, height=480, fps=30):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self._cap = None

    def open(self):
        if not os.path.exists(self.device):
            raise RuntimeError(f'Camera device {self.device} not found!')
        
        self._cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f'Failed to open camera device {self.device}')
        
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)

    def read(self):
        if self._cap is None:
            raise RuntimeError("Camera not opened")
        return self._cap.read()

    def isOpened(self):
        return self._cap is not None and self._cap.isOpened()

    def release(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class CameraPublisherNode(Node):
    
    _GSTREAMER_PIPELINE = (
        'nvarguscamerasrc sensor-id=0 ! '
        'video/x-raw(memory:NVMM), width={image_width}, height={image_height}, framerate=30/1, format=NV12 ! '
        'nvvidconv ! video/x-raw, format=BGRx, width={image_width}, height={image_height} ! '
        'videoconvert ! video/x-raw, format=BGR ! '
        'appsink max-buffers=1 drop=true sync=false'
    )
    
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('flip_method', 0)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('camera_device', '/dev/video1')
        self.declare_parameter('force_v4l2', False)
        self.declare_parameter('camera_backend', 'auto')  # 'auto', 'gstreamer', 'v4l2'
        self.declare_parameter('gstreamer_pipeline', '')
        
        self._output_width = self.get_parameter('camera_width').value
        self._output_height = self.get_parameter('camera_height').value
        fps = self.get_parameter('camera_fps').value
        self.show_preview = self.get_parameter('show_preview').value
        camera_device = self.get_parameter('camera_device').value
        force_v4l2 = self.get_parameter('force_v4l2').value
        camera_backend = self.get_parameter('camera_backend').value
        gstreamer_pipeline = self.get_parameter('gstreamer_pipeline').value
        
        self.get_logger().info(f'Camera Config: {self._output_width}x{self._output_height} @ {fps} FPS')
        self.get_logger().info(f'USB Device: {camera_device}')
        
        self._camera = None
        self.camera_type = None
        
        # Determine camera backend
        if force_v4l2:
            camera_backend = 'v4l2'
        elif camera_backend == 'auto':
            # Try GStreamer first, fallback to V4L2
            camera_backend = 'gstreamer'
        
        # Camera selection
        if camera_backend == 'gstreamer':
            if not gstreamer_pipeline:
                # Use requested dimensions in the pipeline
                gstreamer_pipeline = self._GSTREAMER_PIPELINE.format(
                    image_width=self._output_width,
                    image_height=self._output_height
                )
            self.get_logger().info(
                f"Opening camera with GStreamer pipeline: {gstreamer_pipeline}")
            self._camera = CameraGStreamer(
                gstreamer_pipeline, width=self._output_width, height=self._output_height)
            try:
                self._camera.open()
                if self._camera.isOpened():
                    self.camera_type = 'GStreamer-CSI'
                    self.get_logger().info('✓ CSI camera opened successfully')
                else:
                    self.get_logger().warn('GStreamer failed, falling back to V4L2')
                    self._camera = None
                    camera_backend = 'v4l2'
            except RuntimeError as e:
                self.get_logger().warn(f'GStreamer failed: {e}, falling back to V4L2')
                self._camera = None
                camera_backend = 'v4l2'
        
        if camera_backend == 'v4l2' or self._camera is None:
            self.get_logger().info(
                f"Opening camera with OpenCV device {camera_device}")
            self._camera = CameraCV2(
                device=camera_device, width=self._output_width, height=self._output_height, fps=fps)
            self._camera.open()
            if not self._camera.isOpened():
                raise RuntimeError(f"Error: Unable to open camera {camera_device}")
            self.camera_type = f'V4L2-USB-{camera_device}'
            self.get_logger().info('✓ USB camera opened successfully')
        
        if not self._camera or not self._camera.isOpened():
            raise RuntimeError("Error: Unable to open camera")
        
        self.get_logger().info("Camera opened successfully")
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Increased from 1 to allow buffering
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos)
        self.get_logger().info('Publisher QoS: BEST_EFFORT, KEEP_LAST(10), VOLATILE')
        self.bridge = CvBridge()
        self.frame_count = 0
        
        self.get_logger().info(f'Publishing with {self.camera_type}')
    
    def step(self):
        if not self._camera.isOpened():
            return
        
        ret, frame = self._camera.read()
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
        if self._camera:
            self._camera.release()


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
