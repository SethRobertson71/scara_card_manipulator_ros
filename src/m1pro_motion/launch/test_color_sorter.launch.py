from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    test_sorter = Node(
        package='m1pro_motion',
        executable='test_color_sorter',
        output='screen',
        parameters=[
            {
                'pick_targets_topic': '/world/card_poses',
                'pick_labels_topic': '/skipbo/pick_target_labels',
                'speed_factor': 40,
                'default_pick_z': 140.0,
                'default_pick_r': 0.0,
                'max_abs_workspace_x': 600.0,
                'max_abs_workspace_y': 600.0,
            }
        ],
    )

    return LaunchDescription([test_sorter])
