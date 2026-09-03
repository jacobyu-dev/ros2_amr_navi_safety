from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_watchdog',
            executable='sensor_watchdog_node',
            name='sensor_watchdog_node',
            parameters=[PathJoinSubstitution([
                FindPackageShare('sensor_watchdog'), 'config', 'sensor_watchdog.yaml'
            ])],
            output='screen',
        ),
    ])
