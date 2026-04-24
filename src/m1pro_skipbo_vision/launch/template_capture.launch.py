from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic',
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/ros2_ws/models/skipbo_templates',
        description='Directory where template images are saved',
    )

    target_number_arg = DeclareLaunchArgument(
        'target_number',
        default_value='1',
        description='Card number to capture (1..12)',
    )

    required_samples_arg = DeclareLaunchArgument(
        'required_samples',
        default_value='4',
        description='How many samples to save before auto-exit',
    )

    capture_interval_arg = DeclareLaunchArgument(
        'capture_interval_sec',
        default_value='0.8',
        description='Minimum interval between captures',
    )

    debug_dir_arg = DeclareLaunchArgument(
        'debug_dir',
        default_value='',
        description='Directory for ROI debug images',
    )

    overlay_topic_arg = DeclareLaunchArgument(
        'overlay_topic',
        default_value='/skipbo/template_overlay',
        description='Raw camera overlay topic',
    )

    rectified_overlay_topic_arg = DeclareLaunchArgument(
        'rectified_overlay_topic',
        default_value='/skipbo/template_rectified',
        description='Rectified card overlay topic',
    )

    show_fallback_roi_arg = DeclareLaunchArgument(
        'show_fallback_roi',
        default_value='true',
        description='Draw fallback ROI when no card is detected',
    )

    fallback_roi_x_arg = DeclareLaunchArgument(
        'fallback_roi_x',
        default_value='0.35',
        description='Fallback ROI x (normalized)',
    )

    fallback_roi_y_arg = DeclareLaunchArgument(
        'fallback_roi_y',
        default_value='0.35',
        description='Fallback ROI y (normalized)',
    )

    fallback_roi_w_arg = DeclareLaunchArgument(
        'fallback_roi_w',
        default_value='0.30',
        description='Fallback ROI width (normalized)',
    )

    fallback_roi_h_arg = DeclareLaunchArgument(
        'fallback_roi_h',
        default_value='0.30',
        description='Fallback ROI height (normalized)',
    )

    roi_mode_arg = DeclareLaunchArgument(
        'roi_mode',
        default_value='auto',
        description='Digit ROI mode: auto or fixed',
    )

    auto_roi_sat_min_arg = DeclareLaunchArgument(
        'auto_roi_sat_min',
        default_value='60',
        description='Auto ROI saturation threshold',
    )

    auto_roi_val_min_arg = DeclareLaunchArgument(
        'auto_roi_val_min',
        default_value='40',
        description='Auto ROI value threshold',
    )

    number_roi_x_arg = DeclareLaunchArgument(
        'number_roi_x',
        default_value='0.54',
        description='Fixed ROI x (normalized)',
    )

    number_roi_y_arg = DeclareLaunchArgument(
        'number_roi_y',
        default_value='0.54',
        description='Fixed ROI y (normalized)',
    )

    number_roi_w_arg = DeclareLaunchArgument(
        'number_roi_w',
        default_value='0.40',
        description='Fixed ROI width (normalized)',
    )

    number_roi_h_arg = DeclareLaunchArgument(
        'number_roi_h',
        default_value='0.40',
        description='Fixed ROI height (normalized)',
    )

    node = Node(
        package='m1pro_skipbo_vision',
        executable='template_capture_node',
        name='template_capture_node',
        output='screen',
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'output_dir': LaunchConfiguration('output_dir'),
                'target_number': LaunchConfiguration('target_number'),
                'required_samples': LaunchConfiguration('required_samples'),
                'capture_interval_sec': LaunchConfiguration('capture_interval_sec'),
                'debug_dir': LaunchConfiguration('debug_dir'),
                'overlay_topic': LaunchConfiguration('overlay_topic'),
                'rectified_overlay_topic': LaunchConfiguration('rectified_overlay_topic'),
                'show_fallback_roi': LaunchConfiguration('show_fallback_roi'),
                'fallback_roi_x': LaunchConfiguration('fallback_roi_x'),
                'fallback_roi_y': LaunchConfiguration('fallback_roi_y'),
                'fallback_roi_w': LaunchConfiguration('fallback_roi_w'),
                'fallback_roi_h': LaunchConfiguration('fallback_roi_h'),
                'roi_mode': LaunchConfiguration('roi_mode'),
                'auto_roi_sat_min': LaunchConfiguration('auto_roi_sat_min'),
                'auto_roi_val_min': LaunchConfiguration('auto_roi_val_min'),
                'number_roi_x': LaunchConfiguration('number_roi_x'),
                'number_roi_y': LaunchConfiguration('number_roi_y'),
                'number_roi_w': LaunchConfiguration('number_roi_w'),
                'number_roi_h': LaunchConfiguration('number_roi_h'),
            }
        ],
    )

    return LaunchDescription(
        [
            image_topic_arg,
            output_dir_arg,
            target_number_arg,
            required_samples_arg,
            capture_interval_arg,
            debug_dir_arg,
            overlay_topic_arg,
            rectified_overlay_topic_arg,
            show_fallback_roi_arg,
            fallback_roi_x_arg,
            fallback_roi_y_arg,
            fallback_roi_w_arg,
            fallback_roi_h_arg,
            roi_mode_arg,
            auto_roi_sat_min_arg,
            auto_roi_val_min_arg,
            number_roi_x_arg,
            number_roi_y_arg,
            number_roi_w_arg,
            number_roi_h_arg,
            node,
        ]
    )
