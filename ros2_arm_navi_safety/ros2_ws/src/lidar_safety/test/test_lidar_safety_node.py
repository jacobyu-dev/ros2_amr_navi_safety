import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


@pytest.mark.launch_test
def generate_test_description():
    safety_node = launch_ros.actions.Node(
        package='lidar_safety',
        executable='lidar_safety_node',
        name='lidar_safety_node',
        parameters=[{
            'scan_topic': '/scan',
            'stop_distance_m': 0.7,
            'front_angle_deg': 30.0,
        }],
        output='screen',
    )
    return launch.LaunchDescription([safety_node, launch_testing.actions.ReadyToTest()])


class TestLidarSafetyNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('lidar_safety_node_test_client')
        cls.scan_publisher = cls.node.create_publisher(LaserScan, '/scan', 10)
        cls.obstacle_detected = None
        cls.min_distance = None
        cls.node.create_subscription(
            Bool, '/safety/lidar/obstacle_detected',
            lambda message: setattr(cls, 'obstacle_detected', message.data), 10)
        cls.node.create_subscription(
            Float32, '/safety/lidar/min_distance',
            lambda message: setattr(cls, 'min_distance', message.data), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def make_scan(front_distance):
        message = LaserScan()
        message.angle_min = -math.pi / 2.0
        message.angle_max = math.pi / 2.0
        message.angle_increment = math.pi / 6.0
        message.range_min = 0.05
        message.range_max = 10.0
        message.ranges = [5.0, 5.0, front_distance, front_distance, front_distance, 5.0, 5.0]
        return message

    def publish_until_result(self, front_distance, expected_obstacle):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.scan_publisher.publish(self.make_scan(front_distance))
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (self.obstacle_detected == expected_obstacle and
                    self.min_distance is not None and
                    math.isclose(self.min_distance, front_distance, abs_tol=1e-4)):
                return
        self.fail('Timed out waiting for the expected LiDAR safety output')

    def test_normal_scan_publishes_clear(self):
        self.publish_until_result(2.0, False)

    def test_obstacle_scan_publishes_detected(self):
        self.publish_until_result(0.4, True)

    def test_scan_returns_to_clear(self):
        self.publish_until_result(2.0, False)


@launch_testing.post_shutdown_test()
class TestLidarSafetyNodeShutdown(unittest.TestCase):
    def test_node_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
