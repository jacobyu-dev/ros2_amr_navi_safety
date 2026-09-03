#!/usr/bin/env python3
"""Publish a deterministic normal -> obstacle -> normal LaserScan sequence."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class TestScanPublisher(Node):
    def __init__(self):
        super().__init__('lidar_safety_test_scan_publisher')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.tick = 0
        self.timer = self.create_timer(0.2, self.publish_scan)
        self.get_logger().info('Publishing /scan: normal (2.0 m) -> obstacle (0.4 m) -> normal')

    def publish_scan(self):
        cycle_tick = self.tick % 30
        front_distance = 0.4 if 10 <= cycle_tick < 20 else 2.0
        scan = LaserScan()
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.pi / 6.0
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [5.0, 5.0, front_distance, front_distance, front_distance, 5.0, 5.0]
        self.publisher.publish(scan)
        self.tick += 1


def main():
    rclpy.init()
    publisher = TestScanPublisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
