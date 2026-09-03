from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('parent_frame', default_value='base_link'),
        DeclareLaunchArgument('child_frame', default_value='laser_frame'),
        DeclareLaunchArgument('check_period_ms', default_value='100'),
        DeclareLaunchArgument('lookup_timeout_ms', default_value='100'),
        DeclareLaunchArgument('stale_threshold_ms', default_value='500'),
        DeclareLaunchArgument('allow_static_transform', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='tf_monitor', executable='tf_monitor_node', name='tf_monitor_node',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('tf_monitor'), 'config', 'tf_monitor.yaml'
                ]),
                {
                    'parent_frame': LaunchConfiguration('parent_frame'),
                    'child_frame': LaunchConfiguration('child_frame'),
                    'check_period_ms': LaunchConfiguration('check_period_ms'),
                    'lookup_timeout_ms': LaunchConfiguration('lookup_timeout_ms'),
                    'stale_threshold_ms': LaunchConfiguration('stale_threshold_ms'),
                    'allow_static_transform': LaunchConfiguration('allow_static_transform'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ], output='screen'),
    ])
