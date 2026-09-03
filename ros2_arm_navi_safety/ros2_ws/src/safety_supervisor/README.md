# safety_supervisor — Phase 5

`safety_supervisor_node` combines the existing monitor decisions; it never
inspects `LaserScan`, PointCloud, or TF data itself. It reuses the Phase 4
`arm_navi_safety_interfaces/msg/SafetyStatus` message end-to-end.

## Architecture

```text
LiDAR Safety Node (/safety/lidar/status) --+
                                            |
Sensor Watchdog (/safety/watchdog/status) --+
                                            |
                                            v
                                   Safety Supervisor
                                            |
                                            v
                              /safety/status (SafetyStatus)
                                            ^
                                            |
TF Monitor Node (/safety/tf/status) -------+
```

The Phase 2 and Phase 3 nodes now publish those two input topics in addition
to their pre-existing `Bool` / diagnostics topics. The producer names are
`lidar_safety` and `tf_monitor`.

## State Machine and fail-safe policy

```text
INIT -> SAFE | WARNING | STOP | FAULT
SAFE <-> WARNING <-> STOP
SAFE | WARNING | STOP <-> FAULT
```

`INIT` is used while waiting for the initial status from every required source.
If a source has still not reported after `startup_timeout_sec`, the supervisor
changes to `FAULT`. Once every source is present, aggregation uses:

```text
FAULT (SafetyStatus.ERROR, invalid data, UNKNOWN, or stale) > STOP > WARNING > SAFE
```

`FAULT` is deliberately recoverable: fresh valid reports from every required
source cause the next evaluation to select the current aggregate state.

## Topics, parameters, and QoS

| Direction | Default topic | Type |
| --- | --- | --- |
| Subscribe | `/safety/lidar/status` | `arm_navi_safety_interfaces/msg/SafetyStatus` |
| Subscribe | `/safety/tf/status` | `arm_navi_safety_interfaces/msg/SafetyStatus` |
| Subscribe | `/safety/watchdog/status` | `arm_navi_safety_interfaces/msg/SafetyStatus` |
| Publish | `/safety/status` | `arm_navi_safety_interfaces/msg/SafetyStatus` |

All three status endpoints use reliable, keep-last-10 QoS: a fresh status is
more useful than a long backlog, while reliable transport avoids normal status
drops. The final message has `source=safety_supervisor`; it maps `INIT` to
`UNKNOWN` and `FAULT` to the existing interface's `ERROR` value. Its
`data_valid` is false for `INIT` and `FAULT`.

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `evaluation_rate_hz` | `20.0` | Timer-based evaluation and final-status publication rate. |
| `source_timeout_sec` | `0.5` | Maximum age of either receipt time or a non-zero input header stamp. |
| `startup_timeout_sec` | `5.0` | Initial grace period before a never-seen required source becomes `FAULT`. |
| `required_sources` | `lidar_safety`, `tf_monitor`, `sensor_watchdog` | Required `SafetyStatus.source` values. |
| `source_topics` | LiDAR and TF topics above | One topic for each required source, in matching order. |
| `system_status_topic` | `/safety/status` | Final state output topic. |

Add a source without changing supervisor code by appending its expected source
name and matching topic to `required_sources` and `source_topics`.

## Phase 6 — Multi-threaded Safety Execution

### Why MultiThreadedExecutor?

Each safety executable uses `rclcpp::executors::MultiThreadedExecutor`. The
`executor_threads` parameter defaults to `4` and must be positive. Separate
executables remain separate processes; this change does not merge node roles.

```text
           MultiThreadedExecutor
                  |
      +-----------+------------+
      |           |            |
 LiDAR callback  TF callback  Supervisor timer
      |           |            |
      +-----+-----+            |
            |                  |
     Shared Safety State        |
        atomic + mutex          |
            |                  |
            +--------+---------+
                     v
               Safety Snapshot
                     |
                     v
                State Machine
                     |
                     v
               SafetyStatus output
```

### Callback Groups

| Node | Callback | Group | Reason |
| --- | --- | --- | --- |
| `lidar_safety_node` | scan subscription | Reentrant | The decision core and configuration are immutable per callback. |
| `tf_monitor_node` | `/tf_static` subscription | Reentrant | Static-frame updates may arrive independently of monitoring. |
| `tf_monitor_node` | health timer | MutuallyExclusive | Serializes `last_status_` and avoids overlapping TF checks. |
| `safety_supervisor_node` | source status subscriptions | Reentrant | LiDAR and TF reports may update concurrently. |
| `safety_supervisor_node` | evaluation timer | MutuallyExclusive | A state transition is evaluated once at a time. |

### Shared State

| Data | Access | Protection |
| --- | --- | --- |
| callback counters | callback writers | `std::atomic<uint64_t>` |
| LiDAR transition-log value | concurrent scan callbacks | `std::mutex` |
| TF static frame pairs | TF subscription and timer | `std::mutex` |
| source level, validity, timestamps, reason | source callbacks and supervisor timer | one `std::mutex` in `SafetySourceStore` |
| state machine and TF last status | their mutually-exclusive timer group | callback-group serialization |
| parameters and callback groups | initialized once | immutable thereafter |

### Atomic Variables

The callback counters use `fetch_add`; their values are independent and never
need to be combined with a timestamp or message payload. Default atomic memory
ordering is intentionally used.

### Mutex Protected State

`SafetySourceStore` protects each complete source record: level, validity,
receipt/message timestamps, and reason. The supervisor timer copies the whole
map while holding the mutex, releases it, then evaluates and publishes from
that private snapshot. Publishing, logging, TF lookup, and state evaluation
never occur while this mutex is held.

### Thread Safety Design

This snapshot rule prevents evaluation from combining fields from different
updates. Source callbacks construct a local `SafetySourceState` first, then
replace it under one RAII `std::lock_guard`. The state machine does not access
shared source data directly.

### Deadlock Prevention

Each aggregate has one mutex. No callback nests these mutexes, and no mutex is
held across ROS publish, logging, TF lookup, service calls, or blocking work.

### Test Strategy

`test_thread_safety` verifies an atomic flag, 8 x 10,000 atomic increments,
consistent writer/reader source snapshots, and concurrent source inputs while
the state machine remains in defined enum states. The launch test starts the
supervisor with four executor threads and concurrently publishes 500 LiDAR and
500 TF messages before checking recovery to `SAFE`.

### Stress Test

Run the normal package tests to execute the short concurrent launch stress
test:

```bash
colcon test --packages-select safety_supervisor
colcon test-result --verbose
```

It checks that the process remains alive, workers complete, and the final
supervisor status is valid. It is deliberately short enough for CI.

### ThreadSanitizer

ThreadSanitizer is not enabled by default because ROS 2 middleware and system
dependencies must be built with compatible sanitizer settings. For a local
compatible toolchain, use a separate workspace build, for example:

```bash
colcon build --cmake-args -DCMAKE_CXX_FLAGS=-fsanitize=thread \
  -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread
```

## Build and automated tests

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select safety_supervisor
colcon test-result --verbose
```

The GTest suite covers INIT, all requested state transitions, recovery,
severity order, and unknown-input failure. The launch test uses fake
`SafetyStatus` publishers to verify `SAFE`, `WARNING`, `STOP`, `ERROR`, source
timeout, and post-timeout recovery through actual ROS topics.

## Run

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run lidar_safety lidar_safety_node
ros2 run tf_monitor tf_monitor_node
ros2 launch sensor_watchdog sensor_watchdog.launch.py
ros2 launch safety_supervisor safety_supervisor.launch.py
ros2 topic echo /safety/status
```

For manual tests without sensors, start only the supervisor and publish a
status from two terminals. Keep each publisher running (`-r 20`) so the
intentional source timeout does not fire.

```bash
ros2 topic pub -r 20 /safety/lidar/status arm_navi_safety_interfaces/msg/SafetyStatus \
  "{source: lidar_safety, level: 1, reason: 'front clear', data_valid: true}"
ros2 topic pub -r 20 /safety/tf/status arm_navi_safety_interfaces/msg/SafetyStatus \
  "{source: tf_monitor, level: 1, reason: 'transform available', data_valid: true}"
```

Change LiDAR `level` to `2` for `WARNING` or `3` for `STOP`; publish TF with
`level: 4, data_valid: false` to produce final `ERROR`. Stop either publisher
for more than `source_timeout_sec` to verify `ERROR`, then restart it with a
fresh SAFE message to verify recovery.

## Replay test

**NOT RUN — no combined LiDAR/TF rosbag dataset is available.** With a bag that
contains `/scan`, `/tf`, and `/tf_static`, run:

```bash
ros2 run lidar_safety lidar_safety_node
ros2 run tf_monitor tf_monitor_node
ros2 run safety_supervisor safety_supervisor_node
ros2 bag play <bag>
ros2 topic echo /safety/status
```

This phase intentionally does not latch faults, provide a manual reset service,
block `/cmd_vel`, control Nav2, or integrate a motor/E-stop controller.

## Phase 9 — Safety decision latency measurement

Phase 9 measures the complete correlated path from an injected fault to the
completed supervisor state-machine decision. It is opt-in: normal safety
operation keeps `enable_latency_measurement=false` and creates no CSV file.

```text
Fake LiDAR / Fault Injector
          | T0 (accepted fault-injection service request)
          v
     LiDAR input
          v
      Watchdog or LiDAR safety
          | T1 (fault policy/input validation first detects the event)
          v
  correlated SafetyStatus (event_id)
          v
   Safety Supervisor callback
          | T2 (callback entry)
          v
   Safety State Machine
          | T3 (update() has returned)
          v
      STOP / FAULT + statistics
```

`event_id`, `fault_type`, T0, and T1 are optional additions to the existing
interfaces, so ordinary producers remain compatible. `SafetyLatencyMonitor`
uses a mutex-protected pending-event map and completed-event set. Thus
reentrant source callbacks can record separate events concurrently, while the
serial evaluation callback completes each ID at most once.

All four timestamps are `std::chrono::steady_clock` nanoseconds. The injected
event is intentionally scoped to one ROS host: on supported Linux ROS 2
deployments `steady_clock` is backed by the host monotonic clock, which lets
the participating processes subtract its values without wall-clock changes or
NTP adjustments. This is not a distributed-clock protocol; cross-host runs
need a synchronized monotonic clock or a different trace mechanism.

| Metric | Definition |
| --- | --- |
| Detection | `T1 - T0` |
| Dispatch | `T2 - T1` |
| Decision processing | `T3 - T2` |
| End-to-end | `T3 - T0` |

For `STOP_PUBLISH` and `SLOW_PUBLISH`, T0 is the moment the fake sensor
accepts the fault-mode request and T1 is the watchdog timer's first LiDAR
timeout observation. For `INVALID_DATA`, T1 is the first LiDAR safety callback
that recognizes the invalid scan. The current project has no TF fault injector,
so a TF timeout still follows the normal fail-safe path but has no synthetic T0
and is not reported as a Phase 9 end-to-end sample.

Each completed event emits an individual log and a scenario summary containing
count, mean, median, P95, P99, and maximum. Percentiles use nearest-rank
calculation. If `enable_latency_csv=true`, `latency_output_path` is truncated
at node start and receives:

```csv
event_id,fault_type,detection_ms,dispatch_ms,decision_ms,total_ms,result_state
1,lidar_timeout,501.2,0.3,0.1,501.6,FAULT
```

The values are engineering observations, not certified functional-safety
limits. No industrial latency threshold is implied.

### Run a measured fault-injection scenario

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch safety_test_tools fault_injection_demo.launch.py
ros2 service call /fake_lidar/set_fault_mode \
  arm_navi_safety_interfaces/srv/SetFaultMode "{mode: 1}"
cat safety_latency_results.csv
```

Switch the mode back to `0` between iterations. Repeating the service call
creates a fresh event ID, so the CSV can be analyzed with spreadsheet tools or
`awk` without mixing fault instances. The unit suite includes exact duration,
statistics, empty/single-sample, ID-correlation, and concurrent insertion
tests; the Phase 8 launch test additionally checks correlated LiDAR timeout
timestamps through the live ROS pipeline.

With that demo still running, repeat a callback-driven timeout benchmark (the
launch config enables CSV output):

```bash
ros2 run safety_test_tools safety_latency_benchmark --ros-args \
  -p iterations:=100 -p fault_mode:=1
```
