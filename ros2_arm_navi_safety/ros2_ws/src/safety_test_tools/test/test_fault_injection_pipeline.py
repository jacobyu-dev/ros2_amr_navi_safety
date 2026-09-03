import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from arm_navi_safety_interfaces.msg import SafetyStatus
from arm_navi_safety_interfaces.srv import SetFaultMode
from rclpy.node import Node


TEST_PREFIX = '/test/phase8'


@pytest.mark.launch_test
def generate_test_description():
    fake_lidar = launch_ros.actions.Node(
        package='safety_test_tools', executable='fake_lidar_node',
        parameters=[{
            'scan_topic': f'{TEST_PREFIX}/scan',
            'set_fault_mode_service': f'{TEST_PREFIX}/set_fault_mode',
            'fault_event_topic': f'{TEST_PREFIX}/fault_events',
            'normal_publish_period_ms': 50,
            'slow_publish_period_ms': 1000,
            'stale_timestamp_offset_ms': 1000,
        }], output='screen')
    lidar = launch_ros.actions.Node(
        package='lidar_safety', executable='lidar_safety_node',
        parameters=[{
            'scan_topic': f'{TEST_PREFIX}/scan', 'stop_distance_m': 0.7,
            'front_angle_deg': 30.0,
            'fault_event_topic': f'{TEST_PREFIX}/fault_events',
        }], output='screen')
    static_tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
        ], output='screen')
    tf_monitor = launch_ros.actions.Node(
        package='tf_monitor', executable='tf_monitor_node',
        parameters=[{
            'parent_frame': 'base_link', 'child_frame': 'laser_frame',
            'check_period_ms': 30, 'lookup_timeout_ms': 10, 'stale_threshold_ms': 200,
            'allow_static_transform': True, 'output_prefix': f'{TEST_PREFIX}/tf_monitor',
            'safety_status_topic': f'{TEST_PREFIX}/tf_status',
        }], output='screen')
    watchdog = launch_ros.actions.Node(
        package='sensor_watchdog', executable='sensor_watchdog_node',
        parameters=[{
            'scan_topic': f'{TEST_PREFIX}/scan', 'tf_status_topic': f'{TEST_PREFIX}/tf_status',
            'watchdog_status_topic': f'{TEST_PREFIX}/watchdog_status',
            'fault_event_topic': f'{TEST_PREFIX}/fault_events',
            'lidar_timeout_ms': 250, 'tf_timeout_ms': 250,
            'watchdog_check_period_ms': 25, 'startup_grace_period_ms': 1000,
        }], output='screen')
    supervisor = launch_ros.actions.Node(
        package='safety_supervisor', executable='safety_supervisor_node',
        parameters=[{
            'evaluation_rate_hz': 40.0, 'source_timeout_sec': 0.4,
            'startup_timeout_sec': 1.0,
            'required_sources': ['lidar_safety', 'tf_monitor', 'sensor_watchdog'],
            'source_topics': [
                '/safety/lidar/status', f'{TEST_PREFIX}/tf_status', f'{TEST_PREFIX}/watchdog_status',
            ],
            'system_status_topic': f'{TEST_PREFIX}/system_status',
            'enable_latency_measurement': True,
        }], output='screen')
    return launch.LaunchDescription([
        fake_lidar, lidar, static_tf, tf_monitor, watchdog, supervisor,
        launch_testing.actions.ReadyToTest(),
    ])


class TestFaultInjectionPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('phase8_fault_injection_test_client')
        cls.system_status = None
        cls.watchdog_status = None
        cls.lidar_status = None
        cls.node.create_subscription(
            SafetyStatus, f'{TEST_PREFIX}/system_status',
            lambda message: setattr(cls, 'system_status', message), 10)
        cls.node.create_subscription(
            SafetyStatus, f'{TEST_PREFIX}/watchdog_status',
            lambda message: setattr(cls, 'watchdog_status', message), 10)
        cls.node.create_subscription(
            SafetyStatus, '/safety/lidar/status',
            lambda message: setattr(cls, 'lidar_status', message), 10)
        cls.mode_client = cls.node.create_client(SetFaultMode, f'{TEST_PREFIX}/set_fault_mode')
        cls.wait_until(lambda: cls.mode_client.service_is_ready(), 'fake LiDAR service', timeout=5.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def wait_until(cls, predicate, description, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.03)
            if predicate():
                return
        raise AssertionError(f'Timed out waiting for {description}')

    @classmethod
    def set_mode(cls, mode):
        request = SetFaultMode.Request()
        request.mode = mode
        future = cls.mode_client.call_async(request)
        cls.wait_until(future.done, f'set fault mode {mode}')
        response = future.result()
        if not response.success:
            raise AssertionError(response.message)

    def test_fault_modes_propagate_through_real_safety_nodes(self):
        # NORMAL: each actual monitor reports healthy and Supervisor is SAFE.
        self.set_mode(SetFaultMode.Request.NORMAL)
        self.wait_until(
            lambda: self.system_status is not None and self.system_status.level == SafetyStatus.SAFE,
            'normal SAFE system state')
        self.assertTrue(self.system_status.data_valid)
        self.assertIsNotNone(self.watchdog_status)
        self.assertEqual(self.watchdog_status.level, SafetyStatus.SAFE)

        # STOP_PUBLISH: raw LiDAR silence is detected by the watchdog and wins as FAULT.
        self.set_mode(SetFaultMode.Request.STOP_PUBLISH)
        self.wait_until(
            lambda: self.watchdog_status is not None and
            self.watchdog_status.level == SafetyStatus.ERROR and
            'lidar=TIMEOUT' in self.watchdog_status.reason,
            'watchdog LiDAR timeout')
        self.assertGreater(self.watchdog_status.latency_event_id, 0)
        self.assertGreater(
            self.watchdog_status.detection_time_steady_ns,
            self.watchdog_status.fault_time_steady_ns)
        self.wait_until(
            lambda: self.system_status is not None and self.system_status.level == SafetyStatus.ERROR,
            'supervisor FAULT after stopped sensor')

        # Recovery policy is automatic: fresh status from every source returns SAFE.
        self.set_mode(SetFaultMode.Request.NORMAL)
        self.wait_until(
            lambda: self.system_status is not None and self.system_status.level == SafetyStatus.SAFE,
            'SAFE recovery after stopped sensor')

        # SLOW_PUBLISH uses a period longer than lidar_timeout_ms, so it is a deterministic timeout.
        self.set_mode(SetFaultMode.Request.SLOW_PUBLISH)
        self.wait_until(
            lambda: self.watchdog_status is not None and
            self.watchdog_status.level == SafetyStatus.ERROR and
            'lidar=TIMEOUT' in self.watchdog_status.reason,
            'watchdog LiDAR timeout from slow publishing')
        self.set_mode(SetFaultMode.Request.NORMAL)
        self.wait_until(
            lambda: self.system_status is not None and self.system_status.level == SafetyStatus.SAFE,
            'SAFE recovery after slow publishing')

        # STALE_TIMESTAMP still reaches the watchdog, but Supervisor rejects the inherited stale
        # LiDAR safety-status header rather than treating receipt time as sufficient freshness.
        self.set_mode(SetFaultMode.Request.STALE_TIMESTAMP)
        self.wait_until(
            lambda: self.watchdog_status is not None and self.watchdog_status.level == SafetyStatus.SAFE,
            'watchdog remains healthy for stale-but-received scans')
        self.wait_until(
            lambda: self.system_status is not None and
            self.system_status.level == SafetyStatus.ERROR and
            'lidar_safety safety status timeout' in self.system_status.reason,
            'supervisor FAULT from stale timestamp')
        self.set_mode(SetFaultMode.Request.NORMAL)
        self.wait_until(
            lambda: self.system_status is not None and self.system_status.level == SafetyStatus.SAFE,
            'SAFE recovery after stale timestamp')

        # INVALID_DATA is recognized by the existing LiDAR validation; raw-message liveness stays SAFE.
        self.set_mode(SetFaultMode.Request.INVALID_DATA)
        self.wait_until(
            lambda: self.lidar_status is not None and not self.lidar_status.data_valid and
            self.lidar_status.level == SafetyStatus.ERROR,
            'invalid LiDAR safety status')
        self.wait_until(
            lambda: self.system_status is not None and self.system_status.level == SafetyStatus.ERROR,
            'supervisor FAULT from invalid LiDAR data')
        self.set_mode(SetFaultMode.Request.NORMAL)
        self.wait_until(
            lambda: self.system_status is not None and self.system_status.level == SafetyStatus.SAFE,
            'SAFE recovery after invalid data')


@launch_testing.post_shutdown_test()
class TestFaultInjectionPipelineShutdown(unittest.TestCase):
    def test_nodes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
