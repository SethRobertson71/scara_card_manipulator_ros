from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('m1pro_skipbo_vision'),
        'config', 'skipbo_params.yaml'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='Input image topic'
    )

    template_roi_topic_arg = DeclareLaunchArgument(
        'template_roi_topic',
        default_value='/camera/skipbo_template_roi',
        description='Template ROI image topic'
    )

    template_dir_arg = DeclareLaunchArgument(
        'template_dir',
        default_value='/ros2_ws/models/card-templates/templates',
        description='Directory with 1..12 template images'
    )

    node = Node(
        package='m1pro_skipbo_vision',
        executable='skipbo_vision_node',
        name='skipbo_vision_node',
        output='screen',
        parameters=[
            config,
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'template_dir': LaunchConfiguration('template_dir'),
                'template_roi_topic': LaunchConfiguration('template_roi_topic'),
            },
        ],
    )

    return LaunchDescription([image_topic_arg, template_dir_arg, template_roi_topic_arg, node])
