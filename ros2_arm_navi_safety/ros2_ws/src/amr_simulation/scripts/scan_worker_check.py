#!/usr/bin/env python3
"""Print one LiDAR minimum per second while a worker crosses the scan field."""
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanCheck(Node):
    def __init__(self):
        super().__init__('scan_worker_check')
        self.minimum = None
        self.messages = 0
        self.samples = 0
        self.started = time.monotonic()
        self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.create_timer(1.0, self.report)
    def callback(self, msg):
        self.messages += 1
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if valid:
            self.minimum = (msg.header.frame_id, min(valid))
    def report(self):
        if self.minimum:
            frame, minimum = self.minimum
            print(f'frame={frame} min_range={minimum:.3f} m', flush=True)
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
