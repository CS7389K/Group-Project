from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_vlm_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TurtleBot3 Team',
    maintainer_email='user@example.com',
    description='Vision-Language Model perception for TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'camera_publisher = {package_name}.camera_publisher:main',
            f'vlm_reasoner = {package_name}.vlm_reasoner:main',
            f'image_viewer = {package_name}.image_viewer:main',
        ],
    },
)
