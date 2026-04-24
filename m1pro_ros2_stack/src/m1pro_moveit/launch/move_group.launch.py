import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    with open(os.path.join(get_package_share_directory(package_name), file_path), 'r') as f:
        return f.read()

def load_yaml(package_name, file_path):
    with open(os.path.join(get_package_share_directory(package_name), file_path), 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.1.6')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    robot_description = {'robot_description': load_file('m1pro_description', 'urdf/m1pro_description.urdf')}
    robot_description_semantic = {'robot_description_semantic': load_file('m1pro_moveit', 'config/m1pro.srdf')}

    kinematics_yaml = load_yaml('m1pro_moveit', 'config/kinematics.yaml')
    robot_description_kinematics = kinematics_yaml.get('/**', {}).get('ros__parameters', {})

    joint_limits_yaml = load_yaml('m1pro_moveit', 'config/joint_limits.yaml')
    robot_description_planning = joint_limits_yaml.get('/**', {}).get('ros__parameters', {})

    controllers_yaml = load_yaml('m1pro_moveit', 'config/moveit_controllers.yaml')
    moveit_controllers = controllers_yaml.get('/**', {}).get('ros__parameters', {})

    ompl_planning = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
        }
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning,
            moveit_controllers,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'monitor_dynamics': False,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
            }
        ]
    )

    driver_node = Node(
        package='m1pro_bringup',
        executable='m1pro_driver',
        name='m1pro_driver',
        output='screen',
        parameters=[{
            'robot_ip_address': LaunchConfiguration('robot_ip'),
            'JointStatePublishRate': 20.0,
        }]
    )

    return LaunchDescription([robot_ip_arg, use_sim_time_arg, robot_state_publisher, driver_node, move_group])
