"""
Launch file for camera-only testing
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch camera publisher only for testing"""
    
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
    
    camera_node = Node(
        package='turtlebot3_vlm_perception',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'flip_method': 0
        }]
    )
    
    return LaunchDescription([
        camera_width,
        camera_height,
        camera_fps,
        camera_node
    ])
