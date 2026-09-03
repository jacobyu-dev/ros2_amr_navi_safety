from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory('safety_supervisor') + '/config/safety_supervisor.yaml'
    return LaunchDescription([
        Node(
            package='safety_supervisor',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            parameters=[config],
            output='screen',
        ),
    ])
