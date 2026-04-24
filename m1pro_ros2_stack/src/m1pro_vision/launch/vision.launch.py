from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('m1pro_vision'),
        'config', 'vision_params.yaml'
    )

    model_arg = DeclareLaunchArgument(
        'model_path', default_value='yolov8n.pt',
        description='Path to YOLO model (.pt or OpenVINO .xml)'
    )

    device_arg = DeclareLaunchArgument(
        'device', default_value='AUTO',
        description='OpenVINO inference device (AUTO, CPU, GPU)'
    )

    vision_node = Node(
        package='m1pro_vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[
            config,
            {
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
            }
        ]
    )

    return LaunchDescription([model_arg, device_arg, vision_node])
