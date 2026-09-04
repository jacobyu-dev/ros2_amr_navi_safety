import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from arm_navi_safety_interfaces.msg import SafetyStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node


@pytest.mark.launch_test
def generate_test_description():
    gate = launch_ros.actions.Node(
        package='safety_supervisor', executable='safety_velocity_gate',
        parameters=[{
            'input_cmd_vel_topic': '/test/navigation/cmd_vel',
            'output_cmd_vel_topic': '/test/cmd_vel',
            'safety_topic': '/test/safety/status',
            'publish_rate_hz': 100.0,
            'command_timeout_ms': 200,
            'safety_timeout_ms': 300,
        }], output='screen')
    return launch.LaunchDescription([gate, launch_testing.actions.ReadyToTest()])


class TestSafetyVelocityGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('safety_velocity_gate_test_client')
        cls.last_output = None
        cls.command_pub = cls.node.create_publisher(Twist, '/test/navigation/cmd_vel', 10)
        cls.safety_pub = cls.node.create_publisher(SafetyStatus, '/test/safety/status', 10)
        cls.node.create_subscription(
            Twist, '/test/cmd_vel', lambda message: setattr(cls, 'last_output', message), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def publish_inputs(self, level, valid, linear=0.4):
        safety = SafetyStatus()
        safety.header.stamp = self.node.get_clock().now().to_msg()
        safety.source = 'test_supervisor'
        safety.level = level
        safety.data_valid = valid
        safety.reason = 'gate test'
        command = Twist()
        command.linear.x = linear
        self.safety_pub.publish(safety)
        self.command_pub.publish(command)

    def wait_for_output(self, predicate, level, valid, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.publish_inputs(level, valid)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.last_output is not None and predicate(self.last_output):
                return
        self.fail('timed out waiting for gated velocity')

    def test_nonzero_navigation_command_is_blocked_during_fault(self):
        self.wait_for_output(
            lambda command: command.linear.x > 0.3, SafetyStatus.SAFE, True)
        self.wait_for_output(
            lambda command: abs(command.linear.x) < 1.0e-9 and
            abs(command.angular.z) < 1.0e-9,
            SafetyStatus.ERROR, False)


@launch_testing.post_shutdown_test()
class TestSafetyVelocityGateShutdown(unittest.TestCase):
    def test_clean_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
