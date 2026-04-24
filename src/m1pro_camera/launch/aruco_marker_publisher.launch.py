from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_m1pro_camera = get_package_share_directory('m1pro_camera')
    aruco_params_path = os.path.join(pkg_m1pro_camera, 'config', 'aruco_params.yaml')

    return LaunchDescription([
        Node(
            package='ros2_aruco',
            executable='marker_publisher',
            name='aruco_marker_publisher',
            parameters=[aruco_params_path]
        ),
    ])
