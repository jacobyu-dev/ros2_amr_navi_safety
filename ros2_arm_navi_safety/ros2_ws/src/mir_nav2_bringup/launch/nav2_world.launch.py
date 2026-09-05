"""Launch a local Gazebo world with real Nav2 and the complete safety chain."""

import os
import re

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _base_world(world_name):
    return re.sub(r'_(0|3|10)$', '', world_name)


def _launch(context):
    share = get_package_share_directory('mir_nav2_bringup')
    simulation_share = get_package_share_directory('amr_simulation')
    try:
        nav2_share = get_package_share_directory('nav2_bringup')
    except PackageNotFoundError as error:
        raise RuntimeError(
            'Real Nav2 is not installed. Run: sudo apt-get install '
            'ros-jazzy-navigation2 ros-jazzy-nav2-bringup') from error

    with open(os.path.join(share, 'config', 'worlds.yaml'), encoding='utf-8') as stream:
        world_definitions = yaml.safe_load(stream)['worlds']

    world_name = LaunchConfiguration('world').perform(context)
    base_world = _base_world(world_name)
    if base_world not in world_definitions:
        raise RuntimeError(
            f'Unsupported world "{world_name}". Base world must be one of: '
            + ', '.join(sorted(world_definitions)))

    definition = world_definitions[base_world]
    spawn_x, spawn_y, spawn_yaw = definition['spawn']
    x_override = LaunchConfiguration('x').perform(context)
    y_override = LaunchConfiguration('y').perform(context)
    initial_x = float(x_override) if x_override else float(spawn_x)
    initial_y = float(y_override) if y_override else float(spawn_y)
    initial_yaw = float(spawn_yaw)
    map_yaml = os.path.join(share, 'maps', definition['map'])
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    autostart = LaunchConfiguration('autostart')
    common_sim = {'use_sim_time': use_sim_time}

    actions = [
        LogInfo(msg=f'Nav2 world={world_name}, map={map_yaml}'),
        LogInfo(msg=f'AMCL initial pose: x={initial_x}, y={initial_y}, yaw={initial_yaw}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_share, 'launch', 'simulation.launch.py')),
            launch_arguments={
                'world': world_name,
                'headless': headless,
                'gui_render_engine': LaunchConfiguration('gui_render_engine'),
                'scan_topic': '/scan',
                'cmd_vel_topic': '/safety/cmd_vel',
                'x': str(initial_x),
                'y': str(initial_y),
            }.items()),
        Node(
            package='lidar_safety', executable='lidar_safety_node', name='lidar_safety_node',
            parameters=[os.path.join(
                get_package_share_directory('lidar_safety'), 'config', 'lidar_safety.yaml'),
                common_sim], output='screen'),
        Node(
            package='tf_monitor', executable='tf_monitor_node', name='tf_monitor_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'parent_frame': 'base_link',
                'child_frame': 'front_laser_link',
                'check_period_ms': 100,
                'lookup_timeout_ms': 100,
                'stale_threshold_ms': 500,
                'allow_static_transform': True,
                'safety_status_topic': '/safety/tf/status',
            }], output='screen'),
        Node(
            package='sensor_watchdog', executable='sensor_watchdog_node',
            name='sensor_watchdog_node',
            parameters=[os.path.join(
                get_package_share_directory('sensor_watchdog'), 'config', 'sensor_watchdog.yaml'),
                common_sim], output='screen'),
        Node(
            package='safety_supervisor', executable='safety_supervisor_node',
            name='safety_supervisor_node', parameters=[os.path.join(
                get_package_share_directory('safety_supervisor'),
                'config', 'safety_supervisor.yaml'),
                common_sim], output='screen'),
        Node(
            package='safety_supervisor', executable='safety_velocity_gate',
            name='safety_velocity_gate', parameters=[{
                'use_sim_time': use_sim_time,
                'input_cmd_vel_topic': '/navigation/cmd_vel',
                'output_cmd_vel_topic': '/safety/cmd_vel',
                'safety_topic': '/safety/status',
                'publish_rate_hz': 50.0,
                'command_timeout_ms': 250,
                'safety_timeout_ms': 500,
            }], output='screen'),
        Node(
            package='mission_manager', executable='mission_manager_node',
            name='mission_manager_node',
            parameters=[os.path.join(
                get_package_share_directory('mission_manager'), 'config', 'mission_manager.yaml'),
                common_sim, {
                    'mission_goal_xs': [float(definition['example_goal'][0])],
                    'mission_goal_ys': [float(definition['example_goal'][1])],
                    'mission_goal_yaws': [float(definition['example_goal'][2])],
                }], output='screen'),
        Node(
            package='nav2_path_validator', executable='nav2_path_validator_node',
            name='nav2_path_validator', condition=IfCondition(LaunchConfiguration('validator')),
            parameters=[os.path.join(
                get_package_share_directory('nav2_path_validator'),
                'config', 'nav2_path_validator.yaml'), {
                    'use_sim_time': use_sim_time,
                    'scenario': world_name,
                    'validation_frame': 'map',
                    'pose_source': 'tf',
                    'global_path_topic': '/plan',
                    'local_path_topic': '/received_global_plan',
                    'odom_topic': '/odom',
                    'goal_topic': '/mission/active_goal',
                    'results_directory': LaunchConfiguration('results_directory'),
                }], output='screen'),
        TimerAction(period=3.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                # Nav2 Jazzy evaluates these values in PythonExpression, so
                # they must be Python boolean literals rather than lowercase
                # launch booleans.
                'slam': 'False',
                'use_localization': 'True',
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': LaunchConfiguration('log_level'),
            }.items())]),
        TimerAction(period=6.0, actions=[Node(
            package='mir_nav2_bringup', executable='initial_pose_publisher.py',
            name='world_initial_pose_publisher', parameters=[{
                'use_sim_time': use_sim_time,
                'x': initial_x,
                'y': initial_y,
                'yaw': initial_yaw,
                'frame_id': 'map',
            }], output='screen')]),
        TimerAction(period=4.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'rviz_launch.py')),
            condition=IfCondition(rviz),
            launch_arguments={
                'use_namespace': 'false',
                'namespace': '',
                'use_sim_time': use_sim_time,
            }.items())]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='warehouse_0',
            description=(
                'empty, maze, corridor, bookstore, warehouse, '
                'warehouse_detailed; optional _0/_3/_10')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('gui_render_engine', default_value='ogre'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('validator', default_value='true'),
        DeclareLaunchArgument('results_directory', default_value='results/nav2_world'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'x', default_value='',
            description='Override robot spawn and AMCL initial x'),
        DeclareLaunchArgument(
            'y', default_value='',
            description='Override robot spawn and AMCL initial y'),
        OpaqueFunction(function=_launch),
    ])
