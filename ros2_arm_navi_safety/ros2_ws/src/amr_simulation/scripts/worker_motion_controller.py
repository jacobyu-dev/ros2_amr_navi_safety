#!/usr/bin/env python3
"""Move collision-aware warehouse workers continuously along their lane."""

import json
import math

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_msgs.msg import TFMessage


class WorkerMotionController(Node):
    def __init__(self):
        super().__init__('worker_motion_controller')
        self.declare_parameter('set_pose_service', '/world/default/set_pose')
        self.declare_parameter('paths_json', '[]')
        self.declare_parameter('speed', 0.35)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ground_truth_topic', '/simulation/dynamic_pose')
        self.declare_parameter('robot_clearance', 0.90)
        self._speed = max(0.05, float(self.get_parameter('speed').value))
        self._robot_clearance = max(
            0.0, float(self.get_parameter('robot_clearance').value))
        self._paths = self._load_paths(str(self.get_parameter('paths_json').value))
        self._fractions = [0.0] * len(self._paths)
        self._directions = [1.0] * len(self._paths)
        self._poses = [self._pose_at(path, 0.0, True) for path in self._paths]
        self._yielding = [False] * len(self._paths)
        self._last_update_ns = None
        self._robot_position = None
        self._publishers = [
            self.create_publisher(Pose, f'/simulation/worker_pose_{index}',
                                  qos_profile_sensor_data)
            for index in range(1, len(self._paths) + 1)
        ]
        service = str(self.get_parameter('set_pose_service').value)
        self._client = self.create_client(SetEntityPose, service)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            TFMessage, str(self.get_parameter('ground_truth_topic').value),
            self._ground_truth_callback, qos_profile_sensor_data)
        self._pending = []
        self.create_timer(0.1, self._update_workers)

    def _load_paths(self, paths_json):
        try:
            paths = json.loads(paths_json)
        except json.JSONDecodeError as error:
            raise RuntimeError(f'Invalid worker paths_json: {error}') from error
        required = {'name', 'start_x', 'start_y', 'end_x', 'end_y'}
        if not isinstance(paths, list) or any(
                not isinstance(path, dict) or not required <= path.keys()
                for path in paths):
            raise RuntimeError('Each worker path needs name, start_x, start_y, end_x, end_y')
        return paths

    def _odom_callback(self, msg):
        # Fallback until SceneBroadcaster delivers the physical model pose.
        self._robot_position = (
            float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def _ground_truth_callback(self, msg):
        if not msg.transforms:
            return
        translation = msg.transforms[0].transform.translation
        self._robot_position = (float(translation.x), float(translation.y))

    @staticmethod
    def _pose_at(path, fraction, forward):
        start_x, start_y = float(path['start_x']), float(path['start_y'])
        end_x, end_y = float(path['end_x']), float(path['end_y'])
        pose = Pose()
        pose.position.x = start_x + (end_x - start_x) * fraction
        pose.position.y = start_y + (end_y - start_y) * fraction
        pose.position.z = 0.02
        yaw = math.atan2(end_y - start_y, end_x - start_x)
        if not forward:
            yaw += math.pi
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _candidate_progress(self, index, distance, dt):
        fraction = self._fractions[index]
        direction = self._directions[index]
        fraction += direction * self._speed * dt / max(distance, 1.0e-6)
        if fraction >= 1.0:
            fraction = 2.0 - fraction
            direction = -1.0
        elif fraction <= 0.0:
            fraction = -fraction
            direction = 1.0
        return min(1.0, max(0.0, fraction)), direction

    def _would_enter_robot_clearance(self, pose):
        if self._robot_position is None or self._robot_clearance <= 0.0:
            return False
        return math.hypot(
            pose.position.x - self._robot_position[0],
            pose.position.y - self._robot_position[1]) < self._robot_clearance

    def _update_workers(self):
        if not self._client.service_is_ready():
            return
        self._pending = [future for future in self._pending if not future.done()]
        if len(self._pending) >= len(self._paths):
            return
        now_ns = self.get_clock().now().nanoseconds
        if self._last_update_ns is None:
            self._last_update_ns = now_ns
        dt = min(max((now_ns - self._last_update_ns) / 1_000_000_000.0, 0.0), 0.2)
        self._last_update_ns = now_ns
        for index, path in enumerate(self._paths):
            start_x, start_y = float(path['start_x']), float(path['start_y'])
            end_x, end_y = float(path['end_x']), float(path['end_y'])
            distance = math.hypot(end_x - start_x, end_y - start_y)
            fraction, direction = self._candidate_progress(index, distance, dt)
            pose = self._pose_at(path, fraction, direction > 0.0)
            yielding = self._would_enter_robot_clearance(pose)
            if yielding:
                pose = self._poses[index]
            else:
                self._fractions[index] = fraction
                self._directions[index] = direction
                self._poses[index] = pose
            if yielding != self._yielding[index]:
                if yielding:
                    self.get_logger().info(
                        f'{path["name"]} yielding to MiR inside '
                        f'{self._robot_clearance:.2f} m clearance')
                else:
                    self.get_logger().info(f'{path["name"]} resumed its path')
                self._yielding[index] = yielding
            self._publishers[index].publish(pose)

            request = SetEntityPose.Request()
            request.entity.name = str(path['name'])
            request.entity.type = Entity.MODEL
            request.pose = pose
            self._pending.append(self._client.call_async(request))


def main():
    rclpy.init()
    node = WorkerMotionController()
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
