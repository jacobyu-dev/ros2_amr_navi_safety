import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from arm_navi_safety_interfaces.msg import MissionStatus, SafetyStatus
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_srvs.srv import Trigger


PREFIX = '/test/phase10'


@pytest.mark.launch_test
def generate_test_description():
    fake_nav = launch_ros.actions.Node(
        package='mission_manager', executable='fake_navigate_to_pose_server',
        name='fake_navigate_to_pose_server', parameters=[{
            'navigate_action_name': f'{PREFIX}/navigate_to_pose',
            'result_delay_ms': 40,
            'result_sequence': ['hold'],
        }], output='screen')
    manager = launch_ros.actions.Node(
        package='mission_manager', executable='mission_manager_node',
        name='mission_manager_node', parameters=[{
            'safety_topic': f'{PREFIX}/safety/status',
            'mission_status_topic': f'{PREFIX}/mission/status',
            'goal_pose_topic': f'{PREFIX}/mission/goal_pose',
            'active_goal_topic': f'{PREFIX}/mission/active_goal',
            'navigate_action_name': f'{PREFIX}/navigate_to_pose',
            'start_service': f'{PREFIX}/mission/start',
            'cancel_service': f'{PREFIX}/mission/cancel',
            'pause_service': f'{PREFIX}/mission/pause',
            'resume_service': f'{PREFIX}/mission/resume',
            'mission_id': 10,
            'mission_goal_xs': [1.0, 2.0],
            'mission_goal_ys': [0.0, 1.0],
            'mission_goal_yaws': [0.0, 0.0],
        }], output='screen')
    return launch.LaunchDescription([fake_nav, manager, launch_testing.actions.ReadyToTest()])


class TestMissionManagerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('phase10_mission_manager_test_client')
        cls.status = None
        cls.safety_publisher = cls.node.create_publisher(SafetyStatus, f'{PREFIX}/safety/status', 10)
        cls.goal_publisher = cls.node.create_publisher(PoseStamped, f'{PREFIX}/mission/goal_pose', 10)
        cls.active_goal = None
        cls.node.create_subscription(
            PoseStamped, f'{PREFIX}/mission/active_goal',
            lambda message: setattr(cls, 'active_goal', message), 10)
        cls.node.create_subscription(
            MissionStatus, f'{PREFIX}/mission/status',
            lambda message: setattr(cls, 'status', message), 10)
        cls.start_client = cls.node.create_client(Trigger, f'{PREFIX}/mission/start')
        cls.cancel_client = cls.node.create_client(Trigger, f'{PREFIX}/mission/cancel')
        cls.resume_client = cls.node.create_client(Trigger, f'{PREFIX}/mission/resume')
        cls.parameter_client = AsyncParameterClient(cls.node, '/fake_navigate_to_pose_server')
        cls.wait_until(
            lambda: cls.start_client.service_is_ready() and cls.cancel_client.service_is_ready() and
            cls.resume_client.service_is_ready() and cls.parameter_client.services_are_ready(),
            'mission services and fake server parameter service', timeout=10.0)
        cls.publish_safety(SafetyStatus.SAFE, 'test safety clear')
        cls.wait_until(lambda: cls.status is not None, 'initial mission status')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def wait_until(cls, predicate, description, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.03)
            if predicate():
                return
        received = None if cls.status is None else (
            cls.status.mission_state, cls.status.navigation_state, cls.status.current_goal_index)
        raise AssertionError(f'Timed out waiting for {description}; last={received}')

    @classmethod
    def publish_safety(cls, level, reason):
        message = SafetyStatus()
        message.header.stamp = cls.node.get_clock().now().to_msg()
        message.level = level
        message.source = 'test_safety_supervisor'
        message.reason = reason
        message.data_valid = level in (SafetyStatus.SAFE, SafetyStatus.WARNING, SafetyStatus.STOP)
        cls.safety_publisher.publish(message)

    @classmethod
    def set_fake_outcomes(cls, outcomes):
        future = cls.parameter_client.set_parameters([
            Parameter('result_sequence', Parameter.Type.STRING_ARRAY, outcomes)])
        cls.wait_until(future.done, f'fake outcomes {outcomes}')
        result = future.result().results[0]
        if not result.successful:
            raise AssertionError(result.reason)

    @classmethod
    def set_fake_goal_rejection(cls, reject_goals):
        future = cls.parameter_client.set_parameters([
            Parameter('reject_goals', Parameter.Type.BOOL, reject_goals)])
        cls.wait_until(future.done, f'fake reject_goals={reject_goals}')
        result = future.result().results[0]
        if not result.successful:
            raise AssertionError(result.reason)

    @classmethod
    def publish_goal(cls, x=3.0, y=-1.0):
        message = PoseStamped()
        message.header.stamp = cls.node.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.w = 1.0
        cls.goal_publisher.publish(message)

    @classmethod
    def call(cls, client, description):
        future = client.call_async(Trigger.Request())
        cls.wait_until(future.done, description)
        response = future.result()
        if not response.success:
            raise AssertionError(response.message)
        return response

    def test_a_mission_success_across_two_waypoints(self):
        self.set_fake_outcomes(['success', 'success'])
        self.publish_safety(SafetyStatus.SAFE, 'safe before success mission')
        self.call(self.start_client, 'start success mission')
        self.wait_until(
            lambda: self.active_goal is not None and self.active_goal.pose.position.x == 1.0,
            'observable first active goal')
        self.wait_until(
            lambda: self.status is not None and self.status.mission_state == MissionStatus.COMPLETED and
            self.status.navigation_state == MissionStatus.NAVIGATION_IDLE and
            self.status.current_goal_index == 2,
            'mission completion after both goals')

    def test_b_navigation_abort_fails_mission(self):
        self.set_fake_outcomes(['abort'])
        self.publish_safety(SafetyStatus.SAFE, 'safe before abort mission')
        self.call(self.start_client, 'start abort mission')
        self.wait_until(
            lambda: self.status is not None and self.status.mission_state == MissionStatus.FAILED and
            self.status.navigation_state == MissionStatus.NAVIGATION_FAILED,
            'failed mission after action abort')

    def test_c_and_d_safety_stop_requires_explicit_resume(self):
        self.set_fake_outcomes(['hold', 'hold'])
        self.publish_safety(SafetyStatus.SAFE, 'safe before safety scenario')
        self.call(self.start_client, 'start held mission')
        self.wait_until(
            lambda: self.status is not None and self.status.mission_state == MissionStatus.RUNNING and
            self.status.navigation_state == MissionStatus.NAVIGATING,
            'active navigation before safety stop')
        self.publish_safety(SafetyStatus.STOP, 'test obstacle stop')
        self.wait_until(
            lambda: self.status is not None and self.status.mission_state == MissionStatus.PAUSED and
            self.status.navigation_state == MissionStatus.NAVIGATION_PAUSED and
            self.status.current_goal_index == 0,
            'mission paused and waypoint retained')
        self.publish_safety(SafetyStatus.SAFE, 'safety recovered')
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.03)
        self.assertEqual(self.status.mission_state, MissionStatus.PAUSED)
        self.call(self.resume_client, 'explicit resume')
        self.wait_until(
            lambda: self.status is not None and self.status.mission_state == MissionStatus.RUNNING and
            self.status.navigation_state == MissionStatus.NAVIGATING and
            self.status.current_goal_index == 0,
            'resumed navigation at retained waypoint')
        self.call(self.cancel_client, 'cleanup safety scenario')
        self.wait_until(
            lambda: self.status is not None and self.status.mission_state == MissionStatus.CANCELED,
            'cleanup cancellation')

    def test_e_mission_cancel_cancels_active_goal(self):
        self.set_fake_outcomes(['hold'])
        self.publish_safety(SafetyStatus.SAFE, 'safe before cancel mission')
        self.call(self.start_client, 'start cancel mission')
        self.wait_until(
            lambda: self.status is not None and self.status.mission_state == MissionStatus.RUNNING and
            self.status.navigation_state == MissionStatus.NAVIGATING,
            'active navigation before mission cancel')
        self.call(self.cancel_client, 'mission cancel')
        self.wait_until(
            lambda: self.status is not None and self.status.mission_state == MissionStatus.CANCELED and
            self.status.navigation_state == MissionStatus.NAVIGATION_CANCELED,
            'canceled mission and navigation')

    def test_f_external_goal_pose_reports_nav2_feedback_and_completion(self):
        self.set_fake_goal_rejection(False)
        self.set_fake_outcomes(['success'])
        self.publish_safety(SafetyStatus.SAFE, 'safe before external goal')
        self.publish_goal()
        self.wait_until(
            lambda: self.active_goal is not None and self.active_goal.pose.position.x == 3.0 and
            self.active_goal.pose.position.y == -1.0,
            'observable external active goal')
        self.wait_until(
            lambda: self.status is not None and self.status.mission_state == MissionStatus.COMPLETED and
            self.status.total_goals == 1 and self.status.current_goal_index == 1,
            'external PoseStamped goal completion')
        self.assertEqual(self.status.distance_remaining, 0.0)
        self.assertGreaterEqual(self.status.navigation_time_sec, 0.0)

    def test_g_nav2_goal_rejection_fails_external_mission(self):
        self.set_fake_goal_rejection(True)
        self.publish_safety(SafetyStatus.SAFE, 'safe before rejected goal')
        self.publish_goal(4.0, 2.0)
        self.wait_until(
            lambda: self.status is not None and self.status.mission_state == MissionStatus.FAILED and
            self.status.navigation_state == MissionStatus.NAVIGATION_FAILED and
            'rejected' in self.status.reason,
            'Nav2 goal rejection')
        self.set_fake_goal_rejection(False)

    def test_h_external_goal_is_blocked_while_safety_stop_is_active(self):
        self.publish_safety(SafetyStatus.STOP, 'stop blocks external goal')
        # Let the reentrant safety callback establish the STOP gate before
        # publishing the independent external goal callback.
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.03)
        self.publish_goal(5.0, 2.0)
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.03)
        self.assertEqual(self.status.mission_state, MissionStatus.FAILED)
        self.assertEqual(self.status.navigation_state, MissionStatus.NAVIGATION_FAILED)


@launch_testing.post_shutdown_test()
class TestMissionManagerNodeShutdown(unittest.TestCase):
    def test_nodes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
