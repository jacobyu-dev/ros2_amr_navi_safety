"""Phase 12: Safety Supervisor output must override mission commands."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from arm_navi_safety_interfaces.msg import MissionStatus, SafetyStatus
from rclpy.node import Node
from std_msgs.msg import UInt64
from std_srvs.srv import Trigger


PREFIX = '/test/phase12'
SOURCES = ('lidar_safety', 'tf_monitor', 'sensor_watchdog')


@pytest.mark.launch_test
def generate_test_description():
    supervisor = launch_ros.actions.Node(
        package='safety_supervisor', executable='safety_supervisor_node',
        name='phase12_safety_supervisor', parameters=[{
            'evaluation_rate_hz': 50.0,
            'source_timeout_sec': 1.0,
            'startup_timeout_sec': 2.0,
            'required_sources': list(SOURCES),
            'source_topics': [f'{PREFIX}/safety/{source}' for source in SOURCES],
            'system_status_topic': f'{PREFIX}/safety/status',
        }], output='screen')
    fake_nav = launch_ros.actions.Node(
        package='mission_manager', executable='fake_navigate_to_pose_server',
        name='phase12_fake_navigate_to_pose_server', parameters=[{
            'navigate_action_name': f'{PREFIX}/navigate_to_pose',
            'cancel_count_topic': f'{PREFIX}/navigation/cancel_count',
            'result_sequence': ['hold'],
        }], output='screen')
    manager = launch_ros.actions.Node(
        package='mission_manager', executable='mission_manager_node',
        name='phase12_mission_manager', parameters=[{
            'safety_topic': f'{PREFIX}/safety/status',
            'mission_status_topic': f'{PREFIX}/mission/status',
            'navigate_action_name': f'{PREFIX}/navigate_to_pose',
            'start_service': f'{PREFIX}/mission/start',
            'cancel_service': f'{PREFIX}/mission/cancel',
            'pause_service': f'{PREFIX}/mission/pause',
            'resume_service': f'{PREFIX}/mission/resume',
            'mission_id': 12,
            'mission_goal_xs': [1.0],
            'mission_goal_ys': [0.0],
            'mission_goal_yaws': [0.0],
        }], output='screen')
    return launch.LaunchDescription([
        supervisor, fake_nav, manager, launch_testing.actions.ReadyToTest(),
    ])


class TestSafetyMissionIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('phase12_safety_mission_test_client')
        cls.levels = {source: SafetyStatus.SAFE for source in SOURCES}
        cls.safety_publishers = {
            source: cls.node.create_publisher(
                SafetyStatus, f'{PREFIX}/safety/{source}', 10)
            for source in SOURCES
        }
        cls.system_status = None
        cls.mission_status = None
        cls.cancel_count = 0
        cls.node.create_subscription(
            SafetyStatus, f'{PREFIX}/safety/status',
            lambda message: setattr(cls, 'system_status', message), 10)
        cls.node.create_subscription(
            MissionStatus, f'{PREFIX}/mission/status',
            lambda message: setattr(cls, 'mission_status', message), 10)
        cls.node.create_subscription(
            UInt64, f'{PREFIX}/navigation/cancel_count',
            lambda message: setattr(cls, 'cancel_count', message.data), 10)
        cls.start_client = cls.node.create_client(Trigger, f'{PREFIX}/mission/start')
        cls.cancel_client = cls.node.create_client(Trigger, f'{PREFIX}/mission/cancel')
        cls.resume_client = cls.node.create_client(Trigger, f'{PREFIX}/mission/resume')
        cls.wait_until(
            lambda: cls.start_client.service_is_ready() and
            cls.cancel_client.service_is_ready() and cls.resume_client.service_is_ready(),
            'mission services')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def publish_sources(cls):
        for source, level in cls.levels.items():
            message = SafetyStatus()
            message.header.stamp = cls.node.get_clock().now().to_msg()
            message.source = source
            message.level = level
            message.reason = f'phase12 {source} level {level}'
            message.data_valid = level != SafetyStatus.ERROR
            cls.safety_publishers[source].publish(message)

    @classmethod
    def set_source_level(cls, source, level):
        cls.levels[source] = level

    @classmethod
    def wait_until(cls, predicate, description, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cls.publish_sources()
            rclpy.spin_once(cls.node, timeout_sec=0.03)
            if predicate():
                return
        mission = None if cls.mission_status is None else (
            cls.mission_status.mission_state, cls.mission_status.navigation_state)
        raise AssertionError(f'Timed out waiting for {description}; mission={mission}')

    @classmethod
    def call(cls, client, description):
        future = client.call_async(Trigger.Request())
        cls.wait_until(future.done, description)
        return future.result()

    def test_safety_supervisor_overrides_mission_and_requires_explicit_recovery(self):
        # A: all real Supervisor inputs are SAFE, so navigation may start.
        self.wait_until(
            lambda: self.system_status is not None and self.system_status.level == SafetyStatus.SAFE,
            'aggregated SAFE status')
        # The Supervisor and Manager run in separate processes. Receive a few
        # additional aggregate messages so the Manager's safety gate is known
        # clear before checking the start command.
        for _ in range(5):
            self.publish_sources()
            rclpy.spin_once(self.node, timeout_sec=0.03)
        self.assertTrue(self.call(self.start_client, 'start while safe').success)
        self.wait_until(
            lambda: self.mission_status is not None and
            self.mission_status.mission_state == MissionStatus.RUNNING and
            self.mission_status.navigation_state == MissionStatus.NAVIGATING,
            'navigation while safe')

        # B + F: STOP reaches Mission Manager through Supervisor and exactly one
        # action cancellation is accepted even though STOP continues at 50 Hz.
        cancel_count_before_stop = self.cancel_count
        self.set_source_level('lidar_safety', SafetyStatus.STOP)
        self.wait_until(
            lambda: self.system_status is not None and
            self.system_status.level == SafetyStatus.STOP and
            self.mission_status is not None and
            self.mission_status.mission_state == MissionStatus.PAUSED and
            self.mission_status.navigation_state == MissionStatus.NAVIGATION_PAUSED and
            self.cancel_count == cancel_count_before_stop + 1,
            'safety STOP pause and one navigation cancel')
        for _ in range(15):
            self.publish_sources()
            rclpy.spin_once(self.node, timeout_sec=0.03)
        self.assertEqual(self.cancel_count, cancel_count_before_stop + 1)

        # C: a start command remains blocked while the Supervisor is STOP.
        blocked_start = self.call(self.start_client, 'start blocked by STOP')
        self.assertFalse(blocked_start.success)
        self.assertIn('safety', blocked_start.message.lower())

        # E: SAFE recovery itself never restarts a paused mission.
        self.set_source_level('lidar_safety', SafetyStatus.SAFE)
        self.wait_until(
            lambda: self.system_status is not None and self.system_status.level == SafetyStatus.SAFE,
            'SAFE recovery')
        self.assertEqual(self.mission_status.mission_state, MissionStatus.PAUSED)
        self.assertTrue(self.call(self.resume_client, 'explicit resume after STOP').success)
        self.wait_until(
            lambda: self.mission_status is not None and
            self.mission_status.mission_state == MissionStatus.RUNNING and
            self.mission_status.navigation_state == MissionStatus.NAVIGATING,
            'navigation after explicit resume')

        # D: existing SafetyStatus.ERROR is the Supervisor's highest-severity
        # fault output (the project has no separate EMERGENCY enum). It pauses
        # navigation and rejects both start and resume while the fault remains.
        self.set_source_level('lidar_safety', SafetyStatus.ERROR)
        self.wait_until(
            lambda: self.system_status is not None and
            self.system_status.level == SafetyStatus.ERROR and
            self.mission_status is not None and
            self.mission_status.mission_state == MissionStatus.PAUSED,
            'fault-level safety stop')
        self.assertFalse(self.call(self.start_client, 'start blocked by fault').success)
        self.assertFalse(self.call(self.resume_client, 'resume blocked by fault').success)

        self.set_source_level('lidar_safety', SafetyStatus.SAFE)
        self.wait_until(
            lambda: self.system_status is not None and self.system_status.level == SafetyStatus.SAFE,
            'SAFE recovery from fault')
        self.assertTrue(self.call(self.resume_client, 'explicit resume after fault').success)
        self.wait_until(
            lambda: self.mission_status is not None and
            self.mission_status.navigation_state == MissionStatus.NAVIGATING,
            'navigation after fault recovery')
        self.assertTrue(self.call(self.cancel_client, 'test cleanup').success)


@launch_testing.post_shutdown_test()
class TestSafetyMissionIntegrationShutdown(unittest.TestCase):
    def test_nodes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
