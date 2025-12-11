#!/usr/bin/env python3
"""
Complete Network Bridge Launch File
====================================
Launches EVERYTHING needed for the network bridge system:
  1. Camera publisher
  2. ROS2-Network Bridge

This is the ONE launch file you need on TurtleBot3!

Usage:
    # Basic launch
    ros2 launch vlm_bridge complete_network_bridge.launch.py \
        vlm_server_url:=http://192.168.1.100:5000
    
    # With custom camera device
    ros2 launch vlm_bridge complete_network_bridge.launch.py \
        vlm_server_url:=http://192.168.1.100:5000 \
        camera_device:=1
    
    # With preview window
    ros2 launch vlm_bridge complete_network_bridge.launch.py \
        vlm_server_url:=http://192.168.1.100:5000 \
        show_camera_preview:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate complete launch description"""
    
    # Camera arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video1',
        description='Camera device for USB cameras (e.g., /dev/video0, /dev/video1)'
    )
    
    force_v4l2_arg = DeclareLaunchArgument(
        'force_v4l2',
        default_value='false',
        description='Force V4L2 (USB) mode, skip GStreamer (CSI) attempt'
    )
    
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera width in pixels'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera height in pixels'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='30',
        description='Camera frame rate'
    )
    
    flip_method_arg = DeclareLaunchArgument(
        'flip_method',
        default_value='0',
        description='Image flip method (0=none, 2=rotate-180)'
    )
    
    show_camera_preview_arg = DeclareLaunchArgument(
        'show_camera_preview',
        default_value='false',
        description='Show camera preview window'
    )
    
    # VLM server arguments
    vlm_server_url_arg = DeclareLaunchArgument(
        'vlm_server_url',
        default_value='http://192.168.1.100:5000',
        description='URL of remote VLM server (REQUIRED - change to your server IP!)'
    )
    
    inference_rate_arg = DeclareLaunchArgument(
        'inference_rate',
        default_value='2.0',
        description='Inference request rate in Hz'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='10.0',
        description='HTTP request timeout in seconds'
    )
    
    prompt_arg = DeclareLaunchArgument(
        'prompt',
        default_value='Describe what you see and identify any objects.',
        description='Default VLM prompt'
    )
    
    show_preview_arg = DeclareLaunchArgument(
        'show_preview',
        default_value='true',
        description='Show CV window with camera feed and VLM results'
    )
    
    # Startup message
    startup_msg = LogInfo(
        msg=[
            '\n',
            '========================================================\n',
            '  Complete Network Bridge System\n',
            '========================================================\n',
            'Starting:\n',
            '  1. Camera Publisher (Auto CSI/USB) -> /camera/image_raw\n',
            '  2. ROS2-Network Bridge -> VLM Server\n',
            '\n',
            'VLM Server URL: ', LaunchConfiguration('vlm_server_url'), '\n',
            'Camera Device: ', LaunchConfiguration('camera_device'), ' (USB fallback)\n',
            '\n',
            'Topics:\n',
            '  - Published: /camera/image_raw (sensor_msgs/Image)\n',
            '  - Published: /vlm/inference_result (std_msgs/String)\n',
            '\n',
            'Monitor results:\n',
            '  ros2 topic echo /vlm/inference_result\n',
            '  ros2 topic hz /camera/image_raw\n',
            '========================================================\n'
        ]
    )
    
    # Camera Publisher Node
    camera_node = Node(
        package='vlm_bridge',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'camera_device': LaunchConfiguration('camera_device'),
            'force_v4l2': LaunchConfiguration('force_v4l2'),
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'flip_method': LaunchConfiguration('flip_method'),
            'show_preview': LaunchConfiguration('show_camera_preview'),
            "camera_backend": "gstreamer"
        }]
    )
    
    # ROS2-Network Bridge Node
    bridge_node = Node(
        package='vlm_bridge',
        executable='ros2_network_bridge',
        name='ros2_network_bridge',
        output='screen',
        parameters=[{
            'vlm_server_url': LaunchConfiguration('vlm_server_url'),
            'inference_rate': LaunchConfiguration('inference_rate'),
            'timeout': LaunchConfiguration('timeout'),
            'prompt': LaunchConfiguration('prompt'),
            'show_preview': LaunchConfiguration('show_preview'),
        }]
    )
    
    return LaunchDescription([
        # Arguments
        camera_device_arg,
        force_v4l2_arg,
        camera_width_arg,
        camera_height_arg,
        camera_fps_arg,
        flip_method_arg,
        show_camera_preview_arg,
        vlm_server_url_arg,
        inference_rate_arg,
        timeout_arg,
        prompt_arg,
        show_preview_arg,
        # Startup message
        startup_msg,
        # Nodes
        camera_node,
        bridge_node,
    ])
