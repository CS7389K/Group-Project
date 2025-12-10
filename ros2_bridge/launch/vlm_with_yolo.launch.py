from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vlm_server_url',
            default_value='http://10.43.174.30:5000',
            description='VLM server URL'
        ),
        
        DeclareLaunchArgument(
            'rate_hz',
            default_value='2.0',
            description='VLM inference rate'
        ),
        
        # YOLO11 Detector
        Node(
            package='vlm_bridge',
            executable='yolo11_detector',
            name='yolo11_detector',
            parameters=[{
                'model_size': 'n',
                'confidence_threshold': 0.5,
                'show_preview': True,
            }],
            output='screen'
        ),
        
        # Network Bridge with YOLO
        Node(
            package='vlm_bridge',
            executable='ros2_network_bridge',
            name='ros2_network_bridge',
            parameters=[{
                'vlm_server_url': LaunchConfiguration('vlm_server_url'),
                'inference_rate': LaunchConfiguration('rate_hz'),
                'use_yolo': True,
                'show_preview': True,
            }],
            output='screen'
        ),
    ])