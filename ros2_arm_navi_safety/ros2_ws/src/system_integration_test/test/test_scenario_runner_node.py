import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    runner = launch_ros.actions.Node(
        package='system_integration_test', executable='integration_scenario_runner',
        name='integration_scenario_runner', parameters=[{
            'scenario_step_timeout_sec': 0.35,
            'recovery_valid_samples': 3,
        }], output='screen')
    return launch.LaunchDescription([runner, launch_testing.actions.ReadyToTest()])


class TestScenarioRunnerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('integration_scenario_runner_test_client')
        cls.states = []
        cls.result = None
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(
            String, '/integration_test/scenario_state',
            lambda message: cls.states.append(message.data), state_qos)
        cls.node.create_subscription(
            String, '/integration_test/result',
            lambda message: setattr(cls, 'result', message.data), result_qos)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_initial_state_topic_and_timeout_result(self):
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and self.result is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertIn('INITIALIZING', self.states)
        self.assertIn('FAILED', self.states)
        self.assertIsNotNone(self.result)
        self.assertTrue(self.result.startswith('FAIL: step timeout in INITIALIZING'))


@launch_testing.post_shutdown_test()
class TestScenarioRunnerShutdown(unittest.TestCase):
    def test_clean_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
