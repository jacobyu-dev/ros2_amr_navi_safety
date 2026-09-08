#!/usr/bin/env python3
"""Collapse a thin multi-row Gazebo GPU LiDAR scan into a ROS 2D LaserScan."""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import Pose
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import LaserScan


class Scan2DProjector(Node):
    def __init__(self):
        super().__init__('scan_2d_projector')
        self.declare_parameter('render_dropout_hold_sec', 0.25)
        self.declare_parameter('set_pose_service', '/world/default/set_pose')
        self.declare_parameter('world_start_x', 0.0)
        self.declare_parameter('world_start_y', 0.0)
        self.declare_parameter('worker_count', 0)
        self.declare_parameter('worker_radius', 0.30)
        self._hold_sec = float(self.get_parameter('render_dropout_hold_sec').value)
        self._world_start_x = float(self.get_parameter('world_start_x').value)
        self._world_start_y = float(self.get_parameter('world_start_y').value)
        self._worker_radius = float(self.get_parameter('worker_radius').value)
        worker_count = int(self.get_parameter('worker_count').value)
        self._last_good_ranges = None
        self._last_good_intensities = None
        self._last_good_at = 0.0
        self._warned_shape = False
        self._publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan/rendered', self._scan_callback, qos_profile_sensor_data)
        self._latest_odometry = None
        self._pose_request = None
        self.create_subscription(
            Odometry, '/odom', self._odometry_callback, qos_profile_sensor_data)
        self._worker_poses = {}
        self._worker_subscriptions = [
            self.create_subscription(
                Pose,
                f'/simulation/worker_pose_{index}',
                lambda pose, worker_index=index: self._worker_pose_callback(
                    worker_index, pose),
                qos_profile_sensor_data)
            for index in range(1, worker_count + 1)
        ]
        service_name = str(self.get_parameter('set_pose_service').value)
        self._pose_client = self.create_client(SetEntityPose, service_name)
        self.create_timer(0.05, self._update_proxy_pose)

    def _scan_callback(self, msg):
        horizontal_count = self._horizontal_count(msg)
        if horizontal_count <= 0 or len(msg.ranges) % horizontal_count:
            if not self._warned_shape:
                self.get_logger().error(
                    f'Cannot project scan: {len(msg.ranges)} ranges for '
                    f'{horizontal_count} horizontal samples')
                self._warned_shape = True
            return

        layers = len(msg.ranges) // horizontal_count
        ranges = []
        intensity_values = []
        has_matching_intensities = len(msg.intensities) == len(msg.ranges)
        for horizontal_index in range(horizontal_count):
            candidates = [
                (msg.ranges[layer * horizontal_count + horizontal_index], layer)
                for layer in range(layers)
            ]
            valid = [
                candidate for candidate in candidates
                if math.isfinite(candidate[0])
                and msg.range_min <= candidate[0] <= msg.range_max
            ]
            if valid:
                distance, selected_layer = min(valid, key=lambda candidate: candidate[0])
                ranges.append(distance)
                if has_matching_intensities:
                    index = selected_layer * horizontal_count + horizontal_index
                    intensity_values.append(msg.intensities[index])
            else:
                ranges.append(math.inf)
                if has_matching_intensities:
                    intensity_values.append(0.0)

        self._add_worker_returns(msg, ranges)

        now = time.monotonic()
        if any(math.isfinite(distance) for distance in ranges):
            self._last_good_ranges = ranges
            self._last_good_intensities = intensity_values
            self._last_good_at = now
        elif self._last_good_ranges is not None and now - self._last_good_at <= self._hold_sec:
            # Some virtualized Ogre2 drivers intermittently return an entirely
            # empty render frame. A short hold prevents a false-clear safety
            # window without leaving a persistent ghost obstacle.
            ranges = self._last_good_ranges
            intensity_values = self._last_good_intensities

        output = LaserScan()
        output.header = msg.header
        output.angle_min = msg.angle_min
        output.angle_max = msg.angle_max
        output.angle_increment = msg.angle_increment
        output.time_increment = msg.time_increment
        output.scan_time = msg.scan_time
        output.range_min = msg.range_min
        output.range_max = msg.range_max
        output.ranges = ranges
        output.intensities = intensity_values
        self._publisher.publish(output)

    def _odometry_callback(self, msg):
        self._latest_odometry = msg

    def _worker_pose_callback(self, worker_index, pose):
        self._worker_poses[worker_index] = pose

    def _add_worker_returns(self, msg, ranges):
        """Merge collision-radius returns for workers into the rendered scan.

        Gazebo's physics pose remains the source of truth. This only compensates
        for virtualized Ogre2 drivers which render static geometry correctly but
        can omit moving visuals from GPU LiDAR frames.
        """
        if self._latest_odometry is None or not self._worker_poses:
            return

        robot_pose = self._latest_odometry.pose.pose
        robot_yaw = self._yaw(robot_pose.orientation)
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        robot_x = robot_pose.position.x + self._world_start_x
        robot_y = robot_pose.position.y + self._world_start_y
        laser_x = robot_x + 0.509646 * cos_yaw
        laser_y = robot_y + 0.509646 * sin_yaw
        radius_squared = self._worker_radius * self._worker_radius

        for worker_pose in self._worker_poses.values():
            world_dx = worker_pose.position.x - laser_x
            world_dy = worker_pose.position.y - laser_y
            centre_x = cos_yaw * world_dx + sin_yaw * world_dy
            centre_y = -sin_yaw * world_dx + cos_yaw * world_dy
            distance_squared = centre_x * centre_x + centre_y * centre_y

            if distance_squared <= radius_squared:
                for index in range(len(ranges)):
                    ranges[index] = min(ranges[index], msg.range_min)
                continue

            centre_distance = math.sqrt(distance_squared)
            centre_angle = math.atan2(centre_y, centre_x)
            half_angle = math.asin(min(1.0, self._worker_radius / centre_distance))
            first = max(0, int(math.floor(
                (centre_angle - half_angle - msg.angle_min) / msg.angle_increment)))
            last = min(len(ranges) - 1, int(math.ceil(
                (centre_angle + half_angle - msg.angle_min) / msg.angle_increment)))
            if first > last:
                continue

            for index in range(first, last + 1):
                ray_angle = msg.angle_min + index * msg.angle_increment
                projection = (
                    centre_x * math.cos(ray_angle)
                    + centre_y * math.sin(ray_angle))
                discriminant = projection * projection - (
                    distance_squared - radius_squared)
                if projection <= 0.0 or discriminant < 0.0:
                    continue
                distance = max(msg.range_min, projection - math.sqrt(discriminant))
                if distance <= msg.range_max:
                    ranges[index] = min(ranges[index], distance)

    def _update_proxy_pose(self):
        if self._latest_odometry is None or not self._pose_client.service_is_ready():
            return
        if self._pose_request is not None and not self._pose_request.done():
            return
        request = SetEntityPose.Request()
        request.entity.name = 'mir_lidar_proxy'
        request.entity.type = Entity.MODEL
        request.pose = deepcopy(self._latest_odometry.pose.pose)
        request.pose.position.x += self._world_start_x
        request.pose.position.y += self._world_start_y
        request.pose.position.z = 0.0
        self._pose_request = self._pose_client.call_async(request)

    @staticmethod
    def _horizontal_count(msg):
        if msg.angle_increment <= 0.0 or msg.angle_max < msg.angle_min:
            return 0
        return int(round((msg.angle_max - msg.angle_min) / msg.angle_increment)) + 1

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def main():
    rclpy.init()
    node = Scan2DProjector()
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
