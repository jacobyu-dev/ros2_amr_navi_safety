"""Launch the complete Phase 8 fake-LiDAR safety demonstration."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_test_tools', executable='fake_lidar_node', name='fake_lidar_node',
            parameters=[{
                'scan_topic': '/scan',
                'set_fault_mode_service': '/fake_lidar/set_fault_mode',
                'fault_event_topic': '/safety/fault_injection/events',
                'normal_publish_period_ms': 100,
                'slow_publish_period_ms': 2000,
                'stale_timestamp_offset_ms': 1000,
            }], output='screen'),
        Node(
            package='lidar_safety', executable='lidar_safety_node', name='lidar_safety_node',
            parameters=[{
                'scan_topic': '/scan',
                'fault_event_topic': '/safety/fault_injection/events',
            }], output='screen'),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='fake_lidar_static_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
            ], output='screen'),
        Node(
            package='tf_monitor', executable='tf_monitor_node', name='tf_monitor_node',
            parameters=[{
                'parent_frame': 'base_link', 'child_frame': 'laser_frame',
                'check_period_ms': 50, 'lookup_timeout_ms': 20,
                'stale_threshold_ms': 500, 'allow_static_transform': True,
                'safety_status_topic': '/safety/tf/status',
            }], output='screen'),
        Node(
            package='sensor_watchdog', executable='sensor_watchdog_node', name='sensor_watchdog_node',
            parameters=[{
                'scan_topic': '/scan', 'tf_status_topic': '/safety/tf/status',
                'watchdog_status_topic': '/safety/watchdog/status',
                'fault_event_topic': '/safety/fault_injection/events',
                'lidar_timeout_ms': 500, 'tf_timeout_ms': 500,
                'watchdog_check_period_ms': 50, 'startup_grace_period_ms': 2000,
            }], output='screen'),
        Node(
            package='safety_supervisor', executable='safety_supervisor_node',
            name='safety_supervisor_node', parameters=[{
                'evaluation_rate_hz': 20.0, 'source_timeout_sec': 0.5,
                'startup_timeout_sec': 5.0, 'system_status_topic': '/safety/status',
                'enable_latency_measurement': True,
                'enable_latency_csv': True,
                'latency_output_path': 'safety_latency_results.csv',
            }], output='screen'),
    ])
