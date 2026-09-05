"""Launch the observer against an already-running Nav2 system.

This repository does not currently contain or install a production Nav2
bringup. The launch therefore never substitutes a fake planner/controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('nav2_path_validator')
    config = os.path.join(share, 'config', 'nav2_path_validator.yaml')
    rviz_config = os.path.join(share, 'rviz', 'nav2_path_validation.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scenario = LaunchConfiguration('scenario')
    global_path_topic = LaunchConfiguration('global_path_topic')
    local_path_topic = LaunchConfiguration('local_path_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    validation_frame = LaunchConfiguration('validation_frame')
    pose_source = LaunchConfiguration('pose_source')
    completion_source = LaunchConfiguration('completion_source')
    results_directory = LaunchConfiguration('results_directory')
    bag_output = LaunchConfiguration('bag_output')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'scenario', default_value='observer',
            description='Result label such as clear, obstacle, or phase15'),
        DeclareLaunchArgument('global_path_topic', default_value='/plan'),
        DeclareLaunchArgument('local_path_topic', default_value='/received_global_plan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('goal_topic', default_value='/mission/active_goal'),
        DeclareLaunchArgument('validation_frame', default_value='map'),
        DeclareLaunchArgument('pose_source', default_value='tf'),
        DeclareLaunchArgument('completion_source', default_value='mission_status'),
        DeclareLaunchArgument('results_directory', default_value='results'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_output', default_value='nav2_path_validation_bag'),
        Node(
            package='nav2_path_validator', executable='nav2_path_validator_node',
            name='nav2_path_validator', output='screen',
            parameters=[config, {
                'use_sim_time': use_sim_time,
                'scenario': scenario,
                'global_path_topic': global_path_topic,
                'local_path_topic': local_path_topic,
                'odom_topic': odom_topic,
                'goal_topic': goal_topic,
                'validation_frame': validation_frame,
                'pose_source': pose_source,
                'completion_source': completion_source,
                'results_directory': results_directory,
            }]),
        Node(
            package='rviz2', executable='rviz2', name='nav2_path_validation_rviz',
            arguments=['-d', rviz_config], condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_output,
                 global_path_topic, local_path_topic, odom_topic,
                 '/tf', '/tf_static', '/scan', '/cmd_vel', goal_topic,
                 '/navigate_to_pose/_action/status',
                 '/global_costmap/costmap', '/local_costmap/costmap',
                 '/nav2_path_validation/actual_trajectory',
                 '/nav2_path_validation/state', '/nav2_path_validation/result'],
            condition=IfCondition(LaunchConfiguration('record_bag')), output='screen'),
    ])
