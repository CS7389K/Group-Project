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
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sameer Chowdhury',
    maintainer_email='vsj23@txstate.edu',
    description='ROS2 bridge for Moondream2 VLM inference',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlm_server = vlm_bridge.vlm_server:main',
            'vlm_client = vlm_bridge.vlm_client:main',
            'ros2_network_bridge = vlm_bridge.ros2_network_bridge:main',
            'camera_publisher = vlm_bridge.camera_publisher:main',
        ],
    },
)
