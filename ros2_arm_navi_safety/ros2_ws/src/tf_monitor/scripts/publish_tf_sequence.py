#!/usr/bin/env python3
"""Publish dynamic TF as normal (0-3 s), stopped (3-5 s), then normal (5-8 s)."""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster


class SyntheticTfSequence(Node):
    def __init__(self):
        super().__init__('synthetic_tf_sequence_publisher')
        self.broadcaster = TransformBroadcaster(self)
        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)
        self.start_time_ns = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(0.1, self.publish_if_active)
        self.get_logger().info('TF sequence: publish 0-3 s, stop 3-5 s, publish 5-8 s')

    def publish_if_active(self):
        elapsed_s = (self.get_clock().now().nanoseconds - self.start_time_ns) / 1_000_000_000.0
        clock = Clock()
        clock.clock = self.get_clock().now().to_msg()
        self.clock_publisher.publish(clock)
        if elapsed_s >= 8.0:
            self.get_logger().info('TF sequence complete')
            rclpy.shutdown()
            return
        if 3.0 <= elapsed_s < 5.0:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_frame'
        transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = SyntheticTfSequence()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
