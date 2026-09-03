import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Path to Xacro in mir_description (processed dynamically)
    urdf_xacro = os.path.join(
        get_package_share_directory('mir_description'),
        'urdf',
        'mir.urdf.xacro')
    robot_desc = ParameterValue(
        Command(['xacro ', urdf_xacro]), value_type=str
    )

    # Path to RViz config in mir_description
    rviz_config_dir = os.path.join(
        get_package_share_directory('mir_description'),
        'rviz',
        'display.rviz')

    return LaunchDescription([
        # 1. The custom kinematics controller (which now publishes JointState & Odom)
        Node(
            package='mir_control',
            executable='mir_controller',
            name='mir_controller_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        
        # 2. Robot State Publisher to broadcast TF (base_footprint -> other links) based on URDF + JointState
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_desc}]),

        # 3. RViz to visualize the movement
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir]
        )
    ])
