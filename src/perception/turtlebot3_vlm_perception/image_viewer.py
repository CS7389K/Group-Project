#!/usr/bin/env python3
"""
Simple OpenCV viewer for ROS2 camera topic
Shows the camera feed in an OpenCV window
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.bridge = CvBridge()
        
        # QoS Profile: Match publisher's BEST_EFFORT settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        self.get_logger().info('Image Viewer Started - Press Q to quit')
        self.frame_count = 0
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            self.frame_count += 1
            
            # Check image statistics
            import numpy as np
            mean_val = np.mean(cv_image)
            min_val = np.min(cv_image)
            max_val = np.max(cv_image)
            
            # Add info overlay
            info_text = f"Frame: {self.frame_count} | Mean: {mean_val:.1f} | Min: {min_val} | Max: {max_val}"
            cv2.putText(cv_image, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Warning if image appears wrong
            if mean_val > 250:
                cv2.putText(cv_image, "WARNING: Image mostly WHITE!", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.get_logger().warn('Image is mostly white - check camera connection!')
            elif mean_val < 5:
                cv2.putText(cv_image, "WARNING: Image mostly BLACK!", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.get_logger().warn('Image is mostly black - camera covered or wrong settings!')
            
            # Display
            cv2.imshow('Camera Feed', cv_image)
            
            # Log first frame stats
            if self.frame_count == 1:
                self.get_logger().info(f'First frame: {cv_image.shape}, mean={mean_val:.1f}')
            
            # Quit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('User pressed Q - exiting')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    viewer = ImageViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
