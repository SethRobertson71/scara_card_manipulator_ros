from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motion_node = Node(
        package='m1pro_motion',
        executable='dobot_color_sorting_node',
        output='screen',
        parameters=[
            {
                'pick_targets_topic': '/skipbo/pick_targets',
                'pick_labels_topic': '/skipbo/pick_target_labels',
            }
        ],
    )

    return LaunchDescription([motion_node])
