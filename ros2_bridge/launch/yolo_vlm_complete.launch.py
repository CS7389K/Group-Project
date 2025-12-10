#!/usr/bin/env python3
"""
Complete YOLO + VLM System Launch File
=======================================
Launches the full integrated system:
1. Camera publisher (USB with TLS fix)
2. YOLO11 detector (object detection + dimension estimation)
3. YOLO+VLM Bridge (integrated reasoning)

Usage:
    ros2 launch vlm_bridge yolo_vlm_complete.launch.py \
        vlm_server_url:=http://10.43.174.30:5000 \
        camera_device:=/dev/video1 \
        yolo_model:=n \
        inference_rate:=2.0

Parameters:
    vlm_server_url: VLM server URL (required)
    camera_device: Camera device path (default: /dev/video1)
    yolo_model: YOLO model size n/s/m/l/x (default: n - fastest)
    yolo_confidence: Confidence threshold (default: 0.5)
    inference_rate: VLM inference Hz (default: 2.0)
    show_preview: Show visualization windows (default: true)
    yolo_required: Require YOLO for inference (default: false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'vlm_server_url',
            description='VLM server URL (e.g., http://10.43.174.30:5000)'
        ),
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video1',
            description='Camera device path'
        ),
        DeclareLaunchArgument(
            'camera_width',
            default_value='640',
            description='Camera width'
        ),
        DeclareLaunchArgument(
            'camera_height',
            default_value='480',
            description='Camera height'
        ),
        DeclareLaunchArgument(
            'camera_fps',
            default_value='30',
            description='Camera FPS'
        ),
        DeclareLaunchArgument(
            'yolo_model',
            default_value='n',
            description='YOLO11 model size: n(fastest)/s/m/l/x(best)'
        ),
        DeclareLaunchArgument(
            'yolo_confidence',
            default_value='0.5',
            description='YOLO confidence threshold (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'inference_rate',
            default_value='2.0',
            description='VLM inference rate in Hz'
        ),
        DeclareLaunchArgument(
            'timeout',
            default_value='15.0',
            description='VLM request timeout in seconds'
        ),
        DeclareLaunchArgument(
            'show_preview',
            default_value='true',
            description='Show visualization windows'
        ),
        DeclareLaunchArgument(
            'yolo_required',
            default_value='false',
            description='Require YOLO detections before inference'
        ),
        
        # Node 1: Camera Publisher
        Node(
            package='vlm_bridge',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('camera_device'),
                'width': LaunchConfiguration('camera_width'),
                'height': LaunchConfiguration('camera_height'),
                'fps': LaunchConfiguration('camera_fps'),
                'show_preview': LaunchConfiguration('show_preview'),
            }]
        ),
        
        # Node 2: YOLO11 Detector
        Node(
            package='vlm_bridge',
            executable='yolo11_detector',
            name='yolo11_detector',
            output='screen',
            parameters=[{
                'model_size': LaunchConfiguration('yolo_model'),
                'confidence_threshold': LaunchConfiguration('yolo_confidence'),
                'show_preview': LaunchConfiguration('show_preview'),
            }]
        ),
        
        # Node 3: YOLO+VLM Integrated Bridge
        Node(
            package='vlm_bridge',
            executable='yolo_vlm_bridge',
            name='yolo_vlm_bridge',
            output='screen',
            parameters=[{
                'vlm_server_url': LaunchConfiguration('vlm_server_url'),
                'inference_rate': LaunchConfiguration('inference_rate'),
                'timeout': LaunchConfiguration('timeout'),
                'show_preview': LaunchConfiguration('show_preview'),
                'yolo_required': LaunchConfiguration('yolo_required'),
            }]
        ),
    ])
