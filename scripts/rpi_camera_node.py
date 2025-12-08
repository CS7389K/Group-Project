#!/usr/bin/env python3
"""
NODE A: RASPBERRY PI CAMERA PUBLISHER (GStreamer/NvArgus)
---------------------------------------------------------
1. Captures video using nvarguscamerasrc (Hardware Accelerated)
2. Publishes to /image_raw (Best Effort QoS)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class JetsonCameraNode(Node):
    def __init__(self):
        super().__init__('jetson_camera_node')
        
        # 1. GStreamer Pipeline for Jetson Xavier NX + RPi Cam v2/HQ
        #    - nvarguscamerasrc: Hardware access
        #    - nvvidconv: Hardware color conversion (NV12 -> BGRx)
        #    - videoconvert: Final software conversion (BGRx -> BGR) for OpenCV
        self.gstreamer_pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=640, height=480, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! appsink"
        )
        
        self.get_logger().info(f"Trying Pipeline: {self.gstreamer_pipeline}")
        
        # 2. Open Camera
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error("❌ Failed to open nvarguscamerasrc! Is the camera connected?")
            raise RuntimeError("Camera Open Failed")
        else:
            self.get_logger().info("✅ Camera Opened Successfully (NvArgus)")

        # 3. Setup Publisher (Topic: /image_raw)
        #    Using 'qos_profile_sensor_data' = Best Effort (Low Latency)
        self.publisher_ = self.create_publisher(Image, '/image_raw', qos_profile_sensor_data)
        self.bridge = CvBridge()
        
        # 4. Timer (Publish at 30 Hz)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert to ROS Message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("⚠️ Frame dropped or camera disconnected")

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = JetsonCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
