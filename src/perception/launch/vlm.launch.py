"""
Launch file for TurtleBot3 VLM Perception System
Starts both camera publisher and VLM reasoner nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for complete vision system"""
    
    # Declare launch arguments
    camera_width = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera frame width'
    )
    
    camera_height = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera frame height'
    )
    
    camera_fps = DeclareLaunchArgument(
        'camera_fps',
        default_value='30',
        description='Camera frame rate'
    )
    
    analysis_rate = DeclareLaunchArgument(
        'analysis_rate',
        default_value='2.5',
        description='VLM analysis rate in Hz (target: 2-3 Hz)'
    )
    
    detection_threshold = DeclareLaunchArgument(
        'detection_threshold',
        default_value='0.5',
        description='YOLO detection confidence threshold'
    )
    
    use_yolo = DeclareLaunchArgument(
        'use_yolo',
        default_value='true',
        description='Enable YOLO object detection'
    )
    
    # Camera Publisher Node
    camera_node = Node(
        package='turtlebot3_vlm_perception',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'flip_method': 0  # 0=none, 2=rotate-180
        }]
    )
    
    # VLM Reasoner Node
    vlm_node = Node(
        package='turtlebot3_vlm_perception',
        executable='vlm_reasoner',
        name='vlm_reasoner',
        output='screen',
        parameters=[{
            'analysis_rate': LaunchConfiguration('analysis_rate'),
            'detection_threshold': LaunchConfiguration('detection_threshold'),
            'use_yolo': LaunchConfiguration('use_yolo'),
            'robot_payload_g': 500,  # grams
            'robot_gripper_min_mm': 10,  # mm
            'robot_gripper_max_mm': 100  # mm
        }]
    )
    
    return LaunchDescription([
        camera_width,
        camera_height,
        camera_fps,
        analysis_rate,
        detection_threshold,
        use_yolo,
        camera_node,
        vlm_node
    ])
