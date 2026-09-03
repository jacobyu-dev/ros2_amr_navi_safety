# sensor_watchdog — Phase 7

`sensor_watchdog_node` verifies that safety-critical ROS 2 updates are still arriving. A clear LiDAR scan is never trusted after `/scan` stops. It reuses `arm_navi_safety_interfaces/msg/SafetyStatus` and publishes `/safety/watchdog/status` with `source=sensor_watchdog`.

## Architecture and monitored sources

```text
/scan -------------------> LiDAR timestamp ----+
                                                 |
/safety/tf/status -------> TF timestamp --------+--> Sensor Watchdog
                                                        |
                                                        v
                                         /safety/watchdog/status
                                                        |
/safety/lidar/status ----------------------------------+
/safety/tf/status -------------------------------------+--> Safety Supervisor --> /safety/status
```

| Source | Topic observed | Expected rate | Timeout | Critical | Rationale |
| --- | --- | ---: | ---: | --- | --- |
| LiDAR | `/scan` | typical 10 Hz | 500 ms | Yes | Loss of raw scans must not leave the last clear obstacle result trusted. |
| TF monitor | `/safety/tf/status` | 10 Hz (`check_period_ms=100`) | 500 ms | Yes | Detects a TF-monitor node/output failure, distinct from its normal TF-error report. |

The 500 ms default allows roughly five missed 10 Hz updates. `/safety/lidar/status` is not redundantly watched: raw `/scan` detects LiDAR producer loss, while Supervisor independently validates the LiDAR safety decision.

## Policy, parameters, and supervisor integration

| Condition | Watchdog state | Published status |
| --- | --- | --- |
| Before first input during grace | `WAITING` | `UNKNOWN`, invalid |
| Every critical source fresh | `HEALTHY` | `SAFE`, valid |
| First input absent after grace or `age >= timeout` | `TIMEOUT` | `ERROR`, invalid |
| New fresh input after timeout | `HEALTHY` | `SAFE`, valid |

`startup_grace_period_ms` defaults to 2000. The `age == timeout` boundary is intentionally stale. Reasons expose every source, for example `lidar=HEALTHY; tf_monitor=TIMEOUT`. Supervisor consumes this as its required `sensor_watchdog` source and treats `WAITING`/`TIMEOUT` fail-safely. The watchdog recovers immediately; Supervisor retains its existing, recoverable state-machine policy when all inputs become fresh and valid.

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `scan_topic` | `/scan` | LiDAR input. |
| `tf_status_topic` | `/safety/tf/status` | TF-monitor liveness input. |
| `watchdog_status_topic` | `/safety/watchdog/status` | Aggregate output. |
| `lidar_timeout_ms`, `tf_timeout_ms` | 500 | Maximum input age. |
| `watchdog_check_period_ms` | 100 | Timer period. |
| `startup_grace_period_ms` | 2000 | Initial wait for first update. |
| `executor_threads` | 4 | MultiThreadedExecutor workers. |

The check timer is wall-based for regular scheduling, but every age comparison uses `this->now()` from the ROS clock. With `/use_sim_time`, paused rosbag time does not fabricate a timeout; when replay time advances, missing data is evaluated in replay time. Backwards ROS-time jumps are clamped to zero elapsed age.

## Thread safety

The LiDAR and TF subscriptions are reentrant. `WatchdogMonitor` protects each compound timestamp/received/state entry with one RAII mutex, then returns a private snapshot. Logging and publishing happen after release. The timer is mutually exclusive, serializing transition-log state; independent counters are atomic. Logs occur only on `WAITING`, `HEALTHY`, or `TIMEOUT` transitions.

## Build, tests, fault injection, and manual replay substitute

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select sensor_watchdog safety_supervisor
colcon test-result --verbose
```

The pure C++ GTest covers normal updates, timeout, exact boundary, never-received startup, recovery, and multiple-source aggregation. The launch test performs ROS pub/sub validation for startup timeout, LiDAR-only stop, TF-status-only stop, all-input stop, and recovery; these are the Phase 7 fault-injection tests.

For a manual demo, source the workspace in every terminal, then start:

```bash
# Terminal 1
ros2 run lidar_safety lidar_safety_node
# Terminal 2
ros2 launch tf_monitor tf_monitor.launch.py parent_frame:=base_link child_frame:=laser_frame
# Terminal 3
ros2 launch sensor_watchdog sensor_watchdog.launch.py
# Terminal 4
ros2 launch safety_supervisor safety_supervisor.launch.py
# Terminal 5
ros2 topic echo /safety/watchdog/status
ros2 topic echo /safety/status
# Terminal 6: 10 Hz synthetic scan stream
ros2 run lidar_safety publish_test_scan.py
```

With a matching TF broadcaster, observe `SAFE`; stop Terminal 6 for more than 500 ms to see a LiDAR timeout and supervisor `ERROR`, then restart it for recovery. Stop the TF monitor for the second fault, or stop both producers for the third. No real rosbag is included; this controlled publisher sequence is the replay substitute. With a real bag containing `/scan`, `/tf`, and `/tf_static`, run these four nodes and use `ros2 bag play --clock <bag>`.
