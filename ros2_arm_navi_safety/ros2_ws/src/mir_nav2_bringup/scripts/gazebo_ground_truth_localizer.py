#!/usr/bin/env python3
"""Keep map->odom aligned with the MiR model's physical Gazebo pose."""

import math

from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def _yaw(orientation):
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y)
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(sin_yaw, cos_yaw)


def compute_map_to_odom(ground_truth_pose, odom_pose):
    """Return the planar transform which maps odometry onto physical pose."""
    yaw = _yaw(ground_truth_pose.orientation) - _yaw(odom_pose.orientation)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    odom_x = odom_pose.position.x
    odom_y = odom_pose.position.y
    x = ground_truth_pose.position.x - (cos_yaw * odom_x - sin_yaw * odom_y)
    y = ground_truth_pose.position.y - (sin_yaw * odom_x + cos_yaw * odom_y)
    return x, y, yaw


class GazeboGroundTruthLocalizer(Node):
    def __init__(self):
        super().__init__('gazebo_ground_truth_localizer')
        self.declare_parameter('ground_truth_topic', '/simulation/dynamic_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('correction_warning_distance', 0.25)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._warning_distance = float(
            self.get_parameter('correction_warning_distance').value)
        self._ground_truth_pose = None
        self._odom = None
        self._large_correction_reported = False
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            TFMessage, str(self.get_parameter('ground_truth_topic').value),
            self._ground_truth_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._odom_callback, qos_profile_sensor_data)
        self.get_logger().info(
            'Gazebo ground-truth localization enabled: physical MiR pose -> map')

    def _ground_truth_callback(self, msg):
        # SceneBroadcaster lists the root dynamic model first, followed by its
        # links. In these worlds MiR is the only dynamic root model.
        if not msg.transforms:
            return
        transform = msg.transforms[0].transform
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation = transform.rotation
        self._ground_truth_pose = pose
        self._publish_correction()

    def _odom_callback(self, msg):
        self._odom = msg
        self._publish_correction()

    def _publish_correction(self):
        if self._ground_truth_pose is None or self._odom is None:
            return
        x, y, yaw = compute_map_to_odom(
            self._ground_truth_pose, self._odom.pose.pose)
        transform = TransformStamped()
        transform.header.stamp = self._odom.header.stamp
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self._broadcaster.sendTransform(transform)

        distance = math.hypot(x, y)
        if (distance >= self._warning_distance and
                not self._large_correction_reported):
            self.get_logger().warning(
                'Physical displacement detected; correcting map->odom by '
                f'{distance:.3f} m and {math.degrees(yaw):.2f} deg')
            self._large_correction_reported = True
        elif (distance < self._warning_distance * 0.5 and
              self._large_correction_reported):
            self._large_correction_reported = False


def main():
    rclpy.init()
    node = GazeboGroundTruthLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
