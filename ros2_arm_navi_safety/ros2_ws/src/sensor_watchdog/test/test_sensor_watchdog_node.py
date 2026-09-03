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
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
def generate_test_description():
    watchdog = launch_ros.actions.Node(
        package='sensor_watchdog', executable='sensor_watchdog_node',
        parameters=[{
            'scan_topic': '/test/watchdog/scan',
            'tf_status_topic': '/test/watchdog/tf/status',
            'watchdog_status_topic': '/test/watchdog/status',
            'lidar_timeout_ms': 150,
            'tf_timeout_ms': 150,
            'watchdog_check_period_ms': 20,
            'startup_grace_period_ms': 200,
        }], output='screen')
    return launch.LaunchDescription([watchdog, launch_testing.actions.ReadyToTest()])


class TestSensorWatchdogNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('sensor_watchdog_test_client')
        cls.scan_publisher = cls.node.create_publisher(LaserScan, '/test/watchdog/scan', 10)
        cls.tf_publisher = cls.node.create_publisher(
            SafetyStatus, '/test/watchdog/tf/status', 10)
        cls.status = None
        cls.node.create_subscription(
            SafetyStatus, '/test/watchdog/status',
            lambda message: setattr(cls, 'status', message), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.pi / 4.0
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [2.0, 2.0, 2.0, 2.0, 2.0]
        self.scan_publisher.publish(scan)

    def publish_tf_status(self):
        status = SafetyStatus()
        status.header.stamp = self.node.get_clock().now().to_msg()
        status.source = 'tf_monitor'
        status.level = SafetyStatus.SAFE
        status.reason = 'test transform available'
        status.data_valid = True
        self.tf_publisher.publish(status)

    def wait_for(self, expected_level, publish_lidar=False, publish_tf=False, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish_lidar:
                self.publish_scan()
            if publish_tf:
                self.publish_tf_status()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.status is not None and self.status.level == expected_level:
                return self.status
        actual = None if self.status is None else self.status.level
        self.fail(f'Expected watchdog level {expected_level}, received {actual}')

    def test_startup_timeout_and_fault_injection_recovery(self):
        # Never-received inputs become ERROR after the watchdog's grace period.
        self.wait_for(SafetyStatus.ERROR, timeout=1.0)

        # Normal updates produce HEALTHY/SAFE.
        healthy = self.wait_for(SafetyStatus.SAFE, publish_lidar=True, publish_tf=True)
        self.assertIn('lidar=HEALTHY', healthy.reason)
        self.assertIn('tf_monitor=HEALTHY', healthy.reason)

        # Fault 1: stop only LiDAR input.
        lidar_fault = self.wait_for(SafetyStatus.ERROR, publish_tf=True, timeout=1.0)
        self.assertIn('lidar=TIMEOUT', lidar_fault.reason)
        self.wait_for(SafetyStatus.SAFE, publish_lidar=True, publish_tf=True)

        # Fault 2: stop only TF-monitor status updates.
        tf_fault = self.wait_for(SafetyStatus.ERROR, publish_lidar=True, timeout=1.0)
        self.assertIn('tf_monitor=TIMEOUT', tf_fault.reason)
        self.wait_for(SafetyStatus.SAFE, publish_lidar=True, publish_tf=True)

        # Fault 3: stop every monitored input, then verify independent recovery.
        self.wait_for(SafetyStatus.ERROR, timeout=1.0)
        self.wait_for(SafetyStatus.SAFE, publish_lidar=True, publish_tf=True)


@launch_testing.post_shutdown_test()
class TestSensorWatchdogNodeShutdown(unittest.TestCase):
    def test_node_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
