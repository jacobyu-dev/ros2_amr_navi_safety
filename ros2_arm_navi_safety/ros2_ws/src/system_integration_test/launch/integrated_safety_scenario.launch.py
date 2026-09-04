"""Launch the complete event-driven Gazebo safety integration scenario."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    integration_share = get_package_share_directory('system_integration_test')
    simulation_share = get_package_share_directory('amr_simulation')
    scenario_config = os.path.join(integration_share, 'config', 'integration_scenario.yaml')
    obstacle_sdf = os.path.join(
        integration_share, 'models', 'integration_test_obstacle', 'model.sdf')

    headless = LaunchConfiguration('headless')
    record_bag = LaunchConfiguration('record_bag')
    bag_output = LaunchConfiguration('bag_output')
    use_test_navigator = LaunchConfiguration('use_test_navigator')
    gui_render_engine = LaunchConfiguration('gui_render_engine')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_share, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'world': 'empty_0',
            'headless': headless,
            'gui_render_engine': gui_render_engine,
            'scan_topic': '/scan/raw',
            'x': '-5.0',
            'y': '0.0',
        }.items())

    common_sim = {'use_sim_time': True}
    actions = [
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument(
            'gui_render_engine', default_value='ogre',
            description='Gazebo GUI renderer: ogre is the VM-compatible default; ogre2 is optional'),
        DeclareLaunchArgument(
            'use_test_navigator', default_value='true',
            description='Use the deterministic action-compatible test navigator; set false for external Nav2'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_output', default_value='integration_test_bag'),
        simulation,
        TimerAction(period=3.0, actions=[Node(
            package='ros_gz_sim', executable='create', name='spawn_integration_test_obstacle',
            arguments=['-file', obstacle_sdf, '-name', 'integration_test_obstacle',
                       '-x', '100.0', '-y', '100.0', '-z', '0.5'], output='screen')]),
        Node(
            package='safety_test_tools', executable='scan_fault_injector',
            name='scan_fault_injector', parameters=[scenario_config], output='screen'),
        Node(
            package='lidar_safety', executable='lidar_safety_node', name='lidar_safety_node',
            parameters=[os.path.join(
                get_package_share_directory('lidar_safety'), 'config', 'lidar_safety.yaml'),
                common_sim], output='screen'),
        Node(
            package='tf_monitor', executable='tf_monitor_node', name='tf_monitor_node',
            parameters=[{
                'use_sim_time': True,
                'parent_frame': 'base_link',
                'child_frame': 'front_laser_link',
                'check_period_ms': 100,
                'lookup_timeout_ms': 100,
                'stale_threshold_ms': 500,
                'allow_static_transform': True,
                'safety_status_topic': '/safety/tf/status',
            }], output='screen'),
        Node(
            package='sensor_watchdog', executable='sensor_watchdog_node', name='sensor_watchdog_node',
            parameters=[os.path.join(
                get_package_share_directory('sensor_watchdog'), 'config', 'sensor_watchdog.yaml'),
                common_sim], output='screen'),
        Node(
            package='safety_supervisor', executable='safety_supervisor_node',
            name='safety_supervisor_node', parameters=[os.path.join(
                get_package_share_directory('safety_supervisor'), 'config', 'safety_supervisor.yaml'),
                common_sim, {'enable_latency_measurement': True}], output='screen'),
        Node(
            package='safety_supervisor', executable='safety_velocity_gate',
            name='safety_velocity_gate', parameters=[scenario_config], output='screen'),
        Node(
            package='system_integration_test', executable='integration_test_navigator',
            name='integration_test_navigate_to_pose_server', parameters=[scenario_config],
            condition=IfCondition(use_test_navigator), output='screen'),
        Node(
            package='mission_manager', executable='mission_manager_node', name='mission_manager_node',
            parameters=[os.path.join(
                get_package_share_directory('mission_manager'), 'config', 'mission_manager.yaml'),
                scenario_config], output='screen'),
        Node(
            package='system_integration_test', executable='integration_scenario_runner',
            name='integration_scenario_runner', parameters=[scenario_config], output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_output,
                 '/scan', '/odom', '/tf', '/tf_static', '/navigation/cmd_vel', '/cmd_vel',
                 '/safety/lidar/obstacle_detected', '/safety/lidar/status',
                 '/safety/watchdog/status', '/safety/status', '/mission/status',
                 '/safety/fault_injection/events', '/integration_test/scenario_state',
                 '/integration_test/result'],
            condition=IfCondition(record_bag), output='screen'),
    ]
    return LaunchDescription(actions)
