from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Launch m1pro_camera node before skipbo vision node',
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Image topic consumed by skipbo vision node',
    )

    template_dir_arg = DeclareLaunchArgument(
        'template_dir',
        default_value='',
        description='Directory containing 1..12 template images',
    )

    camera_launch = os.path.join(
        get_package_share_directory('m1pro_camera'),
        'launch',
        'camera.launch.py',
    )

    skipbo_launch = os.path.join(
        get_package_share_directory('m1pro_skipbo_vision'),
        'launch',
        'skipbo_vision.launch.py',
    )

    camera_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        condition=IfCondition(LaunchConfiguration('launch_camera')),
    )

    skipbo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(skipbo_launch),
        launch_arguments={
            'image_topic': LaunchConfiguration('image_topic'),
            'template_dir': LaunchConfiguration('template_dir'),
        }.items(),
    )

    return LaunchDescription(
        [
            launch_camera_arg,
            image_topic_arg,
            template_dir_arg,
            camera_include,
            skipbo_include,
        ]
    )
