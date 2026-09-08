#!/usr/bin/env python3
"""Print one LiDAR minimum per second while a worker crosses the scan field."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class ScanCheck(Node):
    def __init__(self):
        super().__init__('scan_worker_check')
        self.minimum = None
        self.finite_count = 0
        self.total_count = 0
        self.messages = 0
        self.samples = 0
        self.started = time.monotonic()
        self.create_subscription(
            LaserScan, '/scan', self.callback, qos_profile_sensor_data)
        self.create_timer(1.0, self.report)
    def callback(self, msg):
        self.messages += 1
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        self.finite_count = len(valid)
        self.total_count = len(msg.ranges)
        if valid:
            self.minimum = (msg.header.frame_id, min(valid))
        else:
            self.minimum = None
    def report(self):
        if self.minimum:
            frame, minimum = self.minimum
            print(
                f'frame={frame} min_range={minimum:.3f} m '
                f'finite={self.finite_count}/{self.total_count}',
                flush=True)
        elif self.messages:
            print('scan received; no finite return in current field of view', flush=True)
        else:
            print('no /scan message received', flush=True)
        self.samples += 1
        if self.samples >= 8:
            rclpy.shutdown()

def main():
    rclpy.init(); node = ScanCheck()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__ == '__main__': main()
