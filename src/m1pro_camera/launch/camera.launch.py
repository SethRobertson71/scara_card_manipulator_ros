from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    publish_rectified_arg = DeclareLaunchArgument(
        'publish_rectified',
        default_value='true',
        description='Enable rectify/crop/image_view pipeline that publishes image_rect topics',
    )

    config = os.path.join(
        get_package_share_directory('m1pro_camera'),
        'config',
        'camera_params.yaml',
    )

    # Force manual exposure using the exact V4L2 control names for C960.
    set_manual_exposure = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1 && '
            'v4l2-ctl -d /dev/video0 --set-ctrl=exposure_dynamic_framerate=0 && '
            'v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=75',
        ],
        output='screen',
    )

    # Run usb_cam as a standalone process to avoid component container crashes.
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[
            config,
            {
                'image_raw.enable_pub_plugins': [
                    'image_transport/raw',
                    'image_transport/compressed',
                ],
            },
        ],
    )

    # Optional rectified/cropped pipeline.
    rectified_container = ComposableNodeContainer(
        name='camera_container_rectified',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration('publish_rectified')),
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node',
                namespace='camera',
                parameters=[
                    {
                        'image_rect.enable_pub_plugins': [
                            'image_transport/raw',
                            'image_transport/compressed',
                        ],
                    }
                ],
                remappings=[
                    ('image', 'image_raw'),
                    ('camera_info', 'camera_info'),
                    ('image_rect', 'image_rect'),
                ],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::CropDecimateNode',
                name='crop_decimate_node',
                namespace='camera',
                parameters=[
                    {
                        'offset_x': 0,
                        'offset_y': 0,
                        'width': 1280,
                        'height': 720,
                        'decimation_x': 1,
                        'decimation_y': 1,
                        'image_rect_cropped.enable_pub_plugins': [
                            'image_transport/raw',
                            'image_transport/compressed',
                        ],
                    }
                ],
                remappings=[
                    ('in/image_raw', 'image_rect'),
                    ('in/camera_info', 'camera_info'),
                    ('out/image_raw', 'image_rect_cropped'),
                    ('out/camera_info', 'camera_info_rect'),
                ],
            ),
        ],
        output='screen',
    )

    image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='image_view',
        namespace='camera',
        condition=IfCondition(LaunchConfiguration('publish_rectified')),
        output='screen',
        remappings=[('image', 'image_rect_cropped')],
    )

    camera_info_proxy_node = Node(
        package='m1pro_camera',
        executable='camera_info_service_proxy.py',
        name='camera_info_proxy',
        namespace='camera',
        output='screen',
        parameters=[config],
    )

    start_after_controls = RegisterEventHandler(
        OnProcessExit(
            target_action=set_manual_exposure,
            on_exit=[camera_node, rectified_container, image_view_node, camera_info_proxy_node],
        )
    )

    return LaunchDescription([publish_rectified_arg, set_manual_exposure, start_after_controls])
