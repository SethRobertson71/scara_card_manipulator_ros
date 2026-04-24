from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('m1pro_camera'),
        'config', 'camera_params.yaml'
    )

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

    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_cam',
                plugin='usb_cam::UsbCamNode',
                name='camera',
                namespace='camera',
                parameters=[config],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify',
                namespace='camera',
                remappings=[
                    ('image', 'image_raw'),
                    ('camera_info', 'camera_info'),
                    ('image_rect', 'image_rect'),
                ],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::CropDecimateNode',
                name='crop_decimate',
                namespace='camera',
                parameters=[{
                    'offset_x': 284,
                    'offset_y': 141,
                    'width': 457,
                    'height': 280,
                    'decimation_x': 1,
                    'decimation_y': 1,
                }],
                remappings=[
                    ('in/image_raw', 'image_rect'),
                    ('in/camera_info', 'camera_info'),
                    ('out/image_raw', 'image_rect_cropped'),
                    ('out/camera_info', 'camera_info_rect'),
                ],
            ),
        ],
    )

    start_after_controls = RegisterEventHandler(
        OnProcessExit(
            target_action=set_manual_exposure,
            on_exit=[container],
        )
    )

    return LaunchDescription([set_manual_exposure, start_after_controls])
