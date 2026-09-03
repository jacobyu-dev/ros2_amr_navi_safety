# safety_test_tools — Phase 8 Fake Sensor Fault Injection

`safety_test_tools` is test infrastructure only.  It provides a deterministic
`fake_lidar_node` that publishes the same `sensor_msgs/msg/LaserScan` input as
the real LiDAR path; production safety packages do not depend on it.

## Architecture

```text
fake_lidar_node (/scan)
       |------------------> lidar_safety (/safety/lidar/status)
       |------------------> sensor_watchdog (/safety/watchdog/status)
static TF --> tf_monitor (/safety/tf/status)
all three SafetyStatus inputs --> safety_supervisor (/safety/status)
```

The Phase 8 launch test runs all five safety-related nodes plus a static TF
publisher.  It changes faults through the real ROS service and observes the
real watchdog, LiDAR safety, and supervisor topics.

## Fault modes

| Mode | Fake sensor behavior | Expected system reaction |
| --- | --- | --- |
| `NORMAL` | Valid, clear scans at `normal_publish_period_ms` (100 ms by default). | Watchdog and supervisor are `SAFE`. |
| `STOP_PUBLISH` | Publishes no scans. | Watchdog reports `lidar=TIMEOUT`; supervisor becomes `ERROR`/FAULT. |
| `SLOW_PUBLISH` | Publishes every `slow_publish_period_ms` (2000 ms by default). | Each gap exceeds watchdog timeout, producing `ERROR`/FAULT. |
| `INVALID_DATA` | Publishes all-NaN ranges. | Existing LiDAR validation publishes invalid `ERROR`; supervisor becomes FAULT while raw-scan watchdog remains live. |
| `STALE_TIMESTAMP` | Publishes scans normally but backdates `header.stamp`. | Watchdog stays live; supervisor rejects the stale inherited LiDAR SafetyStatus timestamp and becomes FAULT. |

The supervisor's existing state machine is automatically recoverable: once all
inputs return to fresh, valid `SAFE` reports, it returns from `FAULT` to `SAFE`.
Phase 8 does not add a reset service or alter that policy.

## Build and automated tests

From the ROS workspace:

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
colcon build --symlink-install --packages-up-to safety_test_tools
source install/setup.bash
colcon test --packages-select safety_test_tools
colcon test-result --verbose
```

`test_fake_lidar` is the unit test for mode conversion and generated message
contents. `test_fault_injection_pipeline.py` is the launch integration test:
it verifies NORMAL, STOP, SLOW, stale timestamp, invalid data, and recovery
with bounded callback-driven waits.

## Manual demo

Terminal 1 starts the full pipeline:

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source install/setup.bash
ros2 launch safety_test_tools fault_injection_demo.launch.py
```

Terminal 2 can observe the aggregate result:

```bash
source /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws/install/setup.bash
ros2 topic echo /safety/status
```

Change fault modes without recompiling. `SetFaultMode` values are `NORMAL=0`,
`STOP_PUBLISH=1`, `SLOW_PUBLISH=2`, `INVALID_DATA=3`, and
`STALE_TIMESTAMP=4`:

```bash
ros2 service call /fake_lidar/set_fault_mode arm_navi_safety_interfaces/srv/SetFaultMode "{mode: 1}"
ros2 service call /fake_lidar/set_fault_mode arm_navi_safety_interfaces/srv/SetFaultMode "{mode: 0}"
```

The first command causes watchdog timeout and supervisor `ERROR`; the second
returns the sensor to normal publishing and demonstrates automatic recovery.
Try modes `2`, `3`, and `4` in the same form to inspect the other fault paths.

## Replay and remaining limits

No real LiDAR hardware, simulator, or rosbag is required for Phase 8. A future
replay test can run `ros2 bag play --clock` with a normal bag and place a
fault-injecting proxy before `/scan` to drop, delay, or backdate samples.
Hardware timing, DDS loss under real load, and bag replay are intentionally not
validated by this deterministic test package.

## Phase 9 latency events

On every non-normal mode request, `fake_lidar_node` publishes a correlated
`SafetyEvent` on `/safety/fault_injection/events`. It carries a monotonically
increasing `event_id`, a fault type, and T0 from `std::chrono::steady_clock`.
The watchdog attaches that correlation to its timeout `SafetyStatus`; invalid
scan handling attaches it in `lidar_safety`. Use the Phase 9-enabled demo
launch to create `safety_latency_results.csv` and see per-event and running
P95/P99/max summaries in the supervisor log.
