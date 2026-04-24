from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_to_camera_color',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_base', 'camera_color_optical_frame']
        ),
    ])
