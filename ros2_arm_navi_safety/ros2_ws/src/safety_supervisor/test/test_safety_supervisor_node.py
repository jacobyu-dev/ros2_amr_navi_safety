import time
import threading
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from arm_navi_safety_interfaces.msg import SafetyStatus
from rclpy.node import Node


@pytest.mark.launch_test
def generate_test_description():
    supervisor = launch_ros.actions.Node(
        package='safety_supervisor',
        executable='safety_supervisor_node',
        name='safety_supervisor_node',
        parameters=[{
            'evaluation_rate_hz': 50.0,
            'source_timeout_sec': 0.25,
            'startup_timeout_sec': 2.0,
            'executor_threads': 4,
            'required_sources': ['lidar_safety', 'tf_monitor'],
            'source_topics': ['/test/safety/lidar/status', '/test/safety/tf/status'],
            'system_status_topic': '/test/safety/status',
        }],
        output='screen',
    )
    return launch.LaunchDescription([supervisor, launch_testing.actions.ReadyToTest()])


class TestSafetySupervisorNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('safety_supervisor_test_client')
        cls.lidar_publisher = cls.node.create_publisher(
            SafetyStatus, '/test/safety/lidar/status', 10)
        cls.tf_publisher = cls.node.create_publisher(
            SafetyStatus, '/test/safety/tf/status', 10)
        cls.system_status = None
        cls.node.create_subscription(
            SafetyStatus, '/test/safety/status',
            lambda message: setattr(cls, 'system_status', message), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def status(self, source, level, valid=True, reason='test status'):
        message = SafetyStatus()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.source = source
        message.level = level
        message.reason = reason
        message.data_valid = valid
        return message

    def wait_for(self, predicate, lidar=None, tf=None, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if lidar is not None:
                lidar.header.stamp = self.node.get_clock().now().to_msg()
                self.lidar_publisher.publish(lidar)
            if tf is not None:
                tf.header.stamp = self.node.get_clock().now().to_msg()
                self.tf_publisher.publish(tf)
            rclpy.spin_once(self.node, timeout_sec=0.03)
            if self.system_status is not None and predicate(self.system_status):
                return
        received = None if self.system_status is None else self.system_status.level
        self.fail(f'Timed out waiting for supervisor state; last level={received}')

    def test_state_transitions_timeout_and_recovery(self):
        lidar_safe = self.status('lidar_safety', SafetyStatus.SAFE, reason='front clear')
        tf_safe = self.status('tf_monitor', SafetyStatus.SAFE, reason='transform available')

        # A: INIT -> SAFE
        self.wait_for(lambda status: status.level == SafetyStatus.SAFE, lidar_safe, tf_safe)

        # B: SAFE -> WARNING
        lidar_warning = self.status('lidar_safety', SafetyStatus.WARNING, reason='object approaching')
        self.wait_for(lambda status: status.level == SafetyStatus.WARNING, lidar_warning, tf_safe)

        # C: WARNING -> STOP
        lidar_stop = self.status('lidar_safety', SafetyStatus.STOP, reason='object in stop zone')
        self.wait_for(lambda status: status.level == SafetyStatus.STOP, lidar_stop, tf_safe)

        # D: STOP -> SAFE
        self.wait_for(lambda status: status.level == SafetyStatus.SAFE, lidar_safe, tf_safe)

        # E: a source error is final FAULT.
        tf_fault = self.status('tf_monitor', SafetyStatus.ERROR, valid=False, reason='transform unavailable')
        self.wait_for(lambda status: status.level == SafetyStatus.ERROR, lidar_safe, tf_fault)

        # F: a previously healthy source timing out causes FAULT.
        self.wait_for(lambda status: status.level == SafetyStatus.SAFE, lidar_safe, tf_safe)
        self.wait_for(lambda status: status.level == SafetyStatus.ERROR, timeout=1.0)

        # G: fresh, valid messages recover FAULT -> SAFE.
        self.wait_for(lambda status: status.level == SafetyStatus.SAFE, lidar_safe, tf_safe)

    def test_concurrent_status_stress(self):
        message_count = 500

        def publish_many(publisher, source, level):
            for sequence in range(message_count):
                publisher.publish(self.status(source, level, reason=f'stress-{sequence}'))

        lidar_thread = threading.Thread(
            target=publish_many,
            args=(self.lidar_publisher, 'lidar_safety', SafetyStatus.SAFE))
        tf_thread = threading.Thread(
            target=publish_many,
            args=(self.tf_publisher, 'tf_monitor', SafetyStatus.SAFE))
        lidar_thread.start()
        tf_thread.start()
        lidar_thread.join(timeout=5.0)
        tf_thread.join(timeout=5.0)
        self.assertFalse(lidar_thread.is_alive())
        self.assertFalse(tf_thread.is_alive())

        lidar_safe = self.status('lidar_safety', SafetyStatus.SAFE, reason='post-stress clear')
        tf_safe = self.status('tf_monitor', SafetyStatus.SAFE, reason='post-stress valid')
        self.wait_for(lambda status: status.level == SafetyStatus.SAFE, lidar_safe, tf_safe)


@launch_testing.post_shutdown_test()
class TestSafetySupervisorNodeShutdown(unittest.TestCase):
    def test_node_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
