from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory('mission_manager') + '/config/mission_manager.yaml'
    return LaunchDescription([
        Node(
            package='mission_manager', executable='fake_navigate_to_pose_server',
            name='fake_navigate_to_pose_server',
            parameters=[{'result_sequence': ['hold']}], output='screen'),
        Node(
            package='mission_manager', executable='mission_manager_node',
            name='mission_manager_node',
            parameters=[config, {'require_safety_clear': False}], output='screen'),
    ])
