from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='10.12.1.224',
        description='IP address of the Dobot M1 Pro robot controller'
    )

    driver_node = Node(
        package='m1pro_bringup',
        executable='m1pro_driver',
        name='m1pro_driver',
        output='screen',
        parameters=[{
            'robot_ip_address': LaunchConfiguration('robot_ip'),
            'JointStatePublishRate': 10.0,
        }]
    )

    return LaunchDescription([robot_ip_arg, driver_node])
