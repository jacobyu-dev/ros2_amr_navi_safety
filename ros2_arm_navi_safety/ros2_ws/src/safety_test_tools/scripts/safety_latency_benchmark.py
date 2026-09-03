#!/usr/bin/env python3
"""Repeat a real fault-injection scenario and let Supervisor CSV collect samples."""

import time

import rclpy
from arm_navi_safety_interfaces.msg import SafetyStatus
from arm_navi_safety_interfaces.srv import SetFaultMode
from rclpy.node import Node


class SafetyLatencyBenchmark(Node):
    """Callback-driven runner for the already-launched Phase 9 pipeline."""

    def __init__(self):
        super().__init__('safety_latency_benchmark')
        self.iterations = self.declare_parameter('iterations', 100).value
        self.fault_mode = self.declare_parameter('fault_mode', 1).value
        self.timeout_sec = self.declare_parameter('timeout_sec', 5.0).value
        service = self.declare_parameter(
            'set_fault_mode_service', '/fake_lidar/set_fault_mode').value
        status_topic = self.declare_parameter('system_status_topic', '/safety/status').value
        if self.iterations <= 0 or self.timeout_sec <= 0.0:
            raise ValueError('iterations and timeout_sec must be positive')
        self.system_level = SafetyStatus.UNKNOWN
        self.create_subscription(SafetyStatus, status_topic, self._status_callback, 10)
        self.client = self.create_client(SetFaultMode, service)

    def _status_callback(self, message):
        self.system_level = message.level

    def wait_until(self, predicate, description):
        deadline = time.monotonic() + self.timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f'timed out waiting for {description}')

    def set_mode(self, mode):
        request = SetFaultMode.Request()
        request.mode = mode
        future = self.client.call_async(request)
        self.wait_until(future.done, f'fault mode {mode} response')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)

    def run(self):
        self.wait_until(self.client.service_is_ready, 'fake LiDAR service')
        for index in range(1, self.iterations + 1):
            self.wait_until(
                lambda: self.system_level == SafetyStatus.SAFE,
                f'SAFE before iteration {index}')
            self.set_mode(self.fault_mode)
            self.wait_until(
                lambda: self.system_level == SafetyStatus.ERROR,
                f'FAULT for iteration {index}')
            self.set_mode(SetFaultMode.Request.NORMAL)
            self.wait_until(
                lambda: self.system_level == SafetyStatus.SAFE,
                f'SAFE recovery for iteration {index}')
            self.get_logger().info(
                f'completed latency benchmark iteration {index}/{self.iterations}')


def main():
    rclpy.init()
    node = SafetyLatencyBenchmark()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
