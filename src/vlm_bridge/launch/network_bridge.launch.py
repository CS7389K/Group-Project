#!/usr/bin/env python3
"""
Launch file for ROS2-Network Bridge
====================================
Runs on TurtleBot3 to bridge ROS2 and remote VLM server.

Usage:
    ros2 launch vlm_bridge network_bridge.launch.py
    ros2 launch vlm_bridge network_bridge.launch.py vlm_server_url:=http://192.168.1.50:5000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for network bridge"""
    
    # Declare arguments
    vlm_server_url_arg = DeclareLaunchArgument(
        'vlm_server_url',
        default_value='http://192.168.1.100:5000',
        description='URL of remote VLM server (HTTP)'
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
        default_value='Describe what you see in this image.',
        description='Default VLM prompt'
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
        }]
    )
    
    return LaunchDescription([
        vlm_server_url_arg,
        inference_rate_arg,
        timeout_arg,
        prompt_arg,
        bridge_node,
    ])
