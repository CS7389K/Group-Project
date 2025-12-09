#!/usr/bin/env python3
"""
Launch file for TurtleBot3 VLM Perception System with YAML Config
Loads parameters from perception_params.yaml
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description loading params from YAML"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('turtlebot3_vlm_perception')
    config_file = os.path.join(pkg_dir, 'config', 'perception_params.yaml')
    
    # Camera Publisher Node with YAML config
    camera_node = Node(
        package='turtlebot3_vlm_perception',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'camera_width': 640,
            'camera_height': 480,
            'camera_fps': 30,
            'flip_method': 0
        }]
    )
    
    # VLM Reasoner Node with YAML config
    vlm_node = Node(
        package='turtlebot3_vlm_perception',
        executable='vlm_reasoner',
        name='vlm_reasoner',
        output='screen',
        parameters=[{
            'analysis_rate': 2.5,
            'detection_threshold': 0.5,
            'use_yolo': True,
            'robot_payload_g': 500,
            'robot_gripper_min_mm': 10,
            'robot_gripper_max_mm': 100
        }]
    )
    
    return LaunchDescription([
        camera_node,
        vlm_node
    ])
