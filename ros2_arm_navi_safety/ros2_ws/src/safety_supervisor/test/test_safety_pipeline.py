import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from arm_navi_safety_interfaces.msg import SafetyStatus
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    lidar = launch_ros.actions.Node(
        package='lidar_safety', executable='lidar_safety_node',
        parameters=[{
            'scan_topic': '/test/safety_pipeline/scan',
            'stop_distance_m': 0.7,
            'front_angle_deg': 30.0,
        }], output='screen')
    tf_monitor = launch_ros.actions.Node(
        package='tf_monitor', executable='tf_monitor_node',
        parameters=[{
            'parent_frame': 'base_link', 'child_frame': 'laser_frame',
            'check_period_ms': 50, 'lookup_timeout_ms': 20,
            'stale_threshold_ms': 200, 'safety_status_topic': '/safety/tf/status',
            'output_prefix': '/test/safety_pipeline/tf_monitor',
        }], output='screen')
    watchdog = launch_ros.actions.Node(
        package='sensor_watchdog', executable='sensor_watchdog_node',
        parameters=[{
            'scan_topic': '/test/safety_pipeline/scan',
            'tf_status_topic': '/safety/tf/status',
            'watchdog_status_topic': '/safety/watchdog/status',
            'lidar_timeout_ms': 500,
            'tf_timeout_ms': 500,
            'watchdog_check_period_ms': 50,
            'startup_grace_period_ms': 2000,
        }], output='screen')
    supervisor = launch_ros.actions.Node(
        package='safety_supervisor', executable='safety_supervisor_node',
        parameters=[{
            'evaluation_rate_hz': 50.0,
            'source_timeout_sec': 0.5,
            'startup_timeout_sec': 2.0,
            'system_status_topic': '/test/safety_pipeline/system_status',
        }], output='screen')
    return launch.LaunchDescription([
        lidar, tf_monitor, watchdog, supervisor, launch_testing.actions.ReadyToTest()])


class TestSafetyPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('safety_pipeline_test_client')
        cls.scan_publisher = cls.node.create_publisher(
            LaserScan, '/test/safety_pipeline/scan', 10)
        cls.tf_broadcaster = TransformBroadcaster(cls.node)
        cls.system_status = None
        cls.node.create_subscription(
            SafetyStatus, '/test/safety_pipeline/system_status',
            lambda message: setattr(cls, 'system_status', message), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def publish_scan(self, front_distance):
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.pi / 6.0
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [5.0, 5.0, front_distance, front_distance, front_distance, 5.0, 5.0]
        self.scan_publisher.publish(scan)

    def publish_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_frame'
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def wait_for(self, expected_level, front_distance, publish_tf, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.publish_scan(front_distance)
            if publish_tf:
                self.publish_tf()
            rclpy.spin_once(self.node, timeout_sec=0.03)
            if self.system_status is not None and self.system_status.level == expected_level:
                return
        actual = None if self.system_status is None else self.system_status.level
        self.fail(f'Expected {expected_level}, received {actual}')

    def test_existing_monitors_feed_system_state(self):
        # Actual LiDAR and TF monitor outputs yield a final SAFE state.
        self.wait_for(SafetyStatus.SAFE, front_distance=2.0, publish_tf=True)

        # The Phase 2 LiDAR STOP output propagates through the Supervisor.
        self.wait_for(SafetyStatus.STOP, front_distance=0.4, publish_tf=True)

        # An actual TF stale/failure report overrides the LiDAR STOP as FAULT.
        self.wait_for(SafetyStatus.ERROR, front_distance=0.4, publish_tf=False)


@launch_testing.post_shutdown_test()
class TestSafetyPipelineShutdown(unittest.TestCase):
    def test_nodes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
