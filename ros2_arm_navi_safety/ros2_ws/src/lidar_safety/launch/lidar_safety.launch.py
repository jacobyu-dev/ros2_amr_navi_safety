from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_safety',
            executable='lidar_safety_node',
            name='lidar_safety_node',
            parameters=[PathJoinSubstitution([
                FindPackageShare('lidar_safety'), 'config', 'lidar_safety.yaml'
            ])],
            output='screen',
        ),
    ])
