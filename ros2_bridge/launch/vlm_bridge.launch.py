#!/usr/bin/env python3
"""
Launch file for VLM Bridge
============================
Starts VLM server and optional client for testing.

Usage:
    ros2 launch vlm_bridge vlm_bridge.launch.py
    ros2 launch vlm_bridge vlm_bridge.launch.py quantization:=4bit
    ros2 launch vlm_bridge vlm_bridge.launch.py with_client:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for VLM bridge"""
    
    # Declare arguments
    model_id_arg = DeclareLaunchArgument(
        'model_id',
        default_value='vikhyatk/moondream2',
        description='HuggingFace model ID'
    )
    
    quantization_arg = DeclareLaunchArgument(
        'quantization',
        default_value='8bit',
        description='Quantization mode: 8bit, 4bit, or none'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device: cuda or cpu'
    )
    
    with_client_arg = DeclareLaunchArgument(
        'with_client',
        default_value='false',
        description='Launch test client'
    )
    
    prompt_arg = DeclareLaunchArgument(
        'prompt',
        default_value='Describe what you see in this image.',
        description='Query prompt for test client'
    )
    
    # VLM Server Node
    vlm_server = Node(
        package='vlm_bridge',
        executable='vlm_server',
        name='vlm_server',
        output='screen',
        parameters=[{
            'model_id': LaunchConfiguration('model_id'),
            'quantization': LaunchConfiguration('quantization'),
            'device': LaunchConfiguration('device'),
        }]
    )
    
    # VLM Client Node (optional)
    vlm_client = Node(
        package='vlm_bridge',
        executable='vlm_client',
        name='vlm_client',
        output='screen',
        parameters=[{
            'prompt': LaunchConfiguration('prompt'),
            'query_rate': 0.5,
        }],
        condition=LaunchConfiguration('with_client')
    )
    
    return LaunchDescription([
        model_id_arg,
        quantization_arg,
        device_arg,
        with_client_arg,
        prompt_arg,
        vlm_server,
        vlm_client,
    ])
