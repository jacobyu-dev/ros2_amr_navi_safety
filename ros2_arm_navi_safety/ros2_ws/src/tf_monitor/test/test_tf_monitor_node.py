import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    normal_monitor = launch_ros.actions.Node(
        package='tf_monitor', executable='tf_monitor_node', name='tf_monitor_node',
        parameters=[{
            'parent_frame': 'base_link', 'child_frame': 'laser_frame',
            'check_period_ms': 50, 'lookup_timeout_ms': 20,
            'stale_threshold_ms': 300, 'output_prefix': '/tf_monitor',
        }], output='screen')
    wrong_frame_monitor = launch_ros.actions.Node(
        package='tf_monitor', executable='tf_monitor_node', name='tf_monitor_wrong_frame_node',
        parameters=[{
            'parent_frame': 'base_link', 'child_frame': 'nonexistent_frame',
            'check_period_ms': 50, 'lookup_timeout_ms': 20,
            'stale_threshold_ms': 300, 'output_prefix': '/tf_monitor_wrong',
        }], output='screen')
    return launch.LaunchDescription([
        normal_monitor, wrong_frame_monitor, launch_testing.actions.ReadyToTest()])


class TestTfMonitorNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('tf_monitor_test_tf_broadcaster')
        cls.broadcaster = TransformBroadcaster(cls.node)
        cls.normal_healthy = None
        cls.normal_diagnostic = None
        cls.wrong_healthy = None
        cls.wrong_diagnostic = None
        cls.node.create_subscription(
            Bool, '/tf_monitor/healthy',
            lambda message: setattr(cls, 'normal_healthy', message.data), 10)
        cls.node.create_subscription(
            DiagnosticArray, '/tf_monitor/diagnostics',
            lambda message: setattr(cls, 'normal_diagnostic', message), 10)
        cls.node.create_subscription(
            Bool, '/tf_monitor_wrong/healthy',
            lambda message: setattr(cls, 'wrong_healthy', message.data), 10)
        cls.node.create_subscription(
            DiagnosticArray, '/tf_monitor_wrong/diagnostics',
            lambda message: setattr(cls, 'wrong_diagnostic', message), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def publish_dynamic_tf(cls):
        transform = TransformStamped()
        transform.header.stamp = cls.node.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_frame'
        transform.transform.rotation.w = 1.0
        cls.broadcaster.sendTransform(transform)

    @staticmethod
    def diagnostic_status(diagnostic):
        if diagnostic is None or not diagnostic.status:
            return None
        return diagnostic.status[0].message

    def wait_for(self, predicate, publish_tf=False):
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            if publish_tf:
                self.publish_dynamic_tf()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail('Timed out waiting for TF monitor output')

    def test_tf_state_sequence_and_wrong_frame(self):
        # No publisher: frames cannot be looked up.
        self.wait_for(lambda: self.normal_healthy is False and
                      self.diagnostic_status(self.normal_diagnostic) in ('MISSING', 'TIMEOUT'))

        # Dynamic TF is present and fresh.
        self.wait_for(lambda: self.normal_healthy is True and
                      self.diagnostic_status(self.normal_diagnostic) == 'OK', publish_tf=True)

        # Stop publishing. TF2 retains the last transform, which becomes stale.
        self.wait_for(lambda: self.normal_healthy is False and
                      self.diagnostic_status(self.normal_diagnostic) == 'STALE')

        # The second monitor intentionally requests a frame that is never published.
        self.wait_for(lambda: self.wrong_healthy is False and
                      self.diagnostic_status(self.wrong_diagnostic) in ('MISSING', 'TIMEOUT'))


@launch_testing.post_shutdown_test()
class TestTfMonitorNodeShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
