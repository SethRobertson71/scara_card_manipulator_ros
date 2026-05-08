from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node


def generate_launch_description():
    launch_bringup_arg = DeclareLaunchArgument(
        'launch_bringup',
        default_value='true',
        description='Launch bringup/driver node',
    )

    launch_world_publisher_arg = DeclareLaunchArgument(
        'launch_world_publisher',
        default_value='true',
        description='Launch card world publisher node',
    )

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
        default_value='/ros2_ws/models/card-templates/templates',
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

    bringup_launch = os.path.join(
        get_package_share_directory('m1pro_bringup'),
        'launch',
        'bringup.launch.py',
    )

   

    camera_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        condition=IfCondition(LaunchConfiguration('launch_camera')),
    )

    bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        condition=IfCondition(LaunchConfiguration('launch_bringup')),
    )

    skipbo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(skipbo_launch),
        launch_arguments={
            'image_topic': LaunchConfiguration('image_topic'),
            'template_dir': LaunchConfiguration('template_dir'),
        }.items(),
    )

    # Card world publisher started as a node (entry point installed as executable)
    world_pub_node = Node(
        package='m1pro_skipbo_vision',
        executable='card_world_publisher',
        name='card_world_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_world_publisher')),
    )


    return LaunchDescription(
        [
            launch_bringup_arg,
            launch_camera_arg,
            launch_world_publisher_arg,
            image_topic_arg,
            template_dir_arg,
            bringup_include,
            camera_include,
            skipbo_include,
            world_pub_node
        ]
    )
