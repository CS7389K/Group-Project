from setuptools import setup
import os
from glob import glob

package_name = 'vlm_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py') if os.path.exists('launch') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sameer Chowdhury',
    maintainer_email='vsj23@txstate.edu',
    description='ROS2 VLM Bridge for TurtleBot3 with YOLO11',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_network_bridge = vlm_bridge.ros2_network_bridge:main',
            'yolo11_detector = vlm_bridge.yolo11_detector:main',
            'yolo_vlm_bridge = vlm_bridge.yolo_vlm_bridge:main',
            'camera_publisher = vlm_bridge.camera_publisher:main',
            'vlm_client = vlm_bridge.vlm_client:main',
        ],
    },
)