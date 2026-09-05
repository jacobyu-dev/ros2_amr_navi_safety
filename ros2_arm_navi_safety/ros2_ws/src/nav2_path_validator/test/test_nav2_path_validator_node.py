import json
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    validator = launch_ros.actions.Node(
        package='nav2_path_validator', executable='nav2_path_validator_node',
        name='nav2_path_validator', parameters=[{
            'global_path_topic': '/test/plan',
            'local_path_topic': '/test/local_plan',
            'odom_topic': '/test/odom',
            'goal_topic': '/test/goal',
            'validation_frame': 'odom',
            'pose_source': 'odom',
            'completion_source': 'manual',
            'trajectory_sample_distance': 0.01,
            'path_goal_tolerance': 0.10,
            'goal_position_tolerance': 0.10,
            'goal_yaw_tolerance_deg': 5.0,
            'max_tracking_rmse': 0.10,
            'max_tracking_error': 0.10,
            'write_csv': False,
        }], output='screen')
    return launch.LaunchDescription([validator, launch_testing.actions.ReadyToTest()])


class TestPathValidatorNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('path_validator_node_test_client')
        cls.goal_pub = cls.node.create_publisher(PoseStamped, '/test/goal', 10)
        cls.path_pub = cls.node.create_publisher(Path, '/test/plan', 10)
        cls.local_path_pub = cls.node.create_publisher(Path, '/test/local_plan', 10)
        cls.odom_pub = cls.node.create_publisher(Odometry, '/test/odom', 10)
        cls.result = None
        result_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(
            String, '/nav2_path_validation/result',
            lambda message: setattr(cls, 'result', json.loads(message.data)), result_qos)
        cls.finalize_client = cls.node.create_client(
            Trigger, '/nav2_path_validation/finalize_success')
        cls.wait_until(cls.finalize_client.service_is_ready, 'validator service', timeout=10.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def wait_until(cls, predicate, description, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.03)
            if predicate():
                return
        raise AssertionError(f'Timed out waiting for {description}')

    @classmethod
    def publish_goal(cls):
        goal = PoseStamped()
        goal.header.stamp = cls.node.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = 1.0
        goal.pose.orientation.w = 1.0
        cls.goal_pub.publish(goal)

    @classmethod
    def make_path(cls, y=0.0):
        path = Path()
        path.header.stamp = cls.node.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        for index in range(11):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = index / 10.0
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    @classmethod
    def publish_odom(cls, x, y=0.02):
        odom = Odometry()
        odom.header.stamp = cls.node.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.w = 1.0
        cls.odom_pub.publish(odom)

    def test_a_straight_path_and_close_trajectory_pass(self):
        # Repeat initial publications so DDS discovery timing cannot drop them.
        for _ in range(5):
            self.publish_goal()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        path = self.make_path()
        for _ in range(5):
            self.path_pub.publish(path)
            self.local_path_pub.publish(path)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        for index in range(21):
            self.publish_odom(index / 20.0)
            rclpy.spin_once(self.node, timeout_sec=0.03)
        future = self.finalize_client.call_async(Trigger.Request())
        self.wait_until(future.done, 'finalize response')
        self.assertTrue(future.result().success, future.result().message)
        self.wait_until(lambda: self.result is not None, 'validation result')
        self.assertEqual(self.result['result'], 'PASS')
        self.assertTrue(self.result['global_path_received'])
        self.assertTrue(self.result['local_path_received'])
        self.assertEqual(self.result['replan_count'], 0)
        self.assertAlmostEqual(self.result['global_path_length'], 1.0, places=5)
        self.assertLess(self.result['rmse_tracking_error'], 0.03)
        self.assertLess(self.result['goal_position_error'], 0.03)

    def test_b_large_tracking_and_goal_error_fail(self):
        type(self).result = None
        self.publish_goal()
        path = self.make_path()
        for _ in range(3):
            self.path_pub.publish(path)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        for index in range(11):
            self.publish_odom(index / 10.0, y=0.50)
            rclpy.spin_once(self.node, timeout_sec=0.03)
        future = self.finalize_client.call_async(Trigger.Request())
        self.wait_until(future.done, 'failing finalize response')
        self.assertFalse(future.result().success)
        self.wait_until(lambda: self.result is not None, 'failing validation result')
        self.assertEqual(self.result['result'], 'FAIL')
        self.assertGreater(self.result['rmse_tracking_error'], 0.49)
        self.assertGreater(self.result['goal_position_error'], 0.49)


@launch_testing.post_shutdown_test()
class TestPathValidatorNodeShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
