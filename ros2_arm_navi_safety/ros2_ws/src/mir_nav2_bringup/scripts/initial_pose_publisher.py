#!/usr/bin/env python3
"""Publish one world-aligned AMCL initial pose after AMCL is listening."""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('world_initial_pose_publisher')
        self.x = self.declare_parameter('x', 0.0).value
        self.y = self.declare_parameter('y', 0.0).value
        self.yaw = self.declare_parameter('yaw', 0.0).value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.attempts = 0
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)
        self.timer = self.create_timer(0.5, self.publish_when_ready)

    def publish_when_ready(self):
        self.attempts += 1
        if self.publisher.get_subscription_count() == 0 and self.attempts < 40:
            return
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.pose.pose.position.x = float(self.x)
        message.pose.pose.position.y = float(self.y)
        message.pose.pose.orientation.z = math.sin(float(self.yaw) / 2.0)
        message.pose.pose.orientation.w = math.cos(float(self.yaw) / 2.0)
        message.pose.covariance[0] = 0.05
        message.pose.covariance[7] = 0.05
        message.pose.covariance[35] = 0.03
        self.publisher.publish(message)
        self.get_logger().info(
            f'Published initial pose in {self.frame_id}: x={self.x}, y={self.y}, yaw={self.yaw}')
        self.timer.cancel()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
