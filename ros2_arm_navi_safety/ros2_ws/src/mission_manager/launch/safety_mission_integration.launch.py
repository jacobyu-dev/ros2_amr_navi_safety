"""Run the existing safety pipeline with the Phase 12 Mission Manager bridge."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='safety_test_tools', executable='fake_lidar_node', output='screen'),
        Node(package='lidar_safety', executable='lidar_safety_node', output='screen'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'], output='screen'),
        Node(package='tf_monitor', executable='tf_monitor_node', output='screen'),
        Node(package='sensor_watchdog', executable='sensor_watchdog_node', output='screen'),
        Node(package='safety_supervisor', executable='safety_supervisor_node', output='screen'),
        Node(
            package='mission_manager', executable='fake_navigate_to_pose_server',
            parameters=[{'result_sequence': ['hold']}], output='screen'),
        Node(
            package='mission_manager', executable='mission_manager_node',
            parameters=[{'mission_goal_xs': [1.0], 'mission_goal_ys': [0.0], 'mission_goal_yaws': [0.0]}],
            output='screen'),
    ])
