# tf_monitor — Phase 3

`tf_monitor_node` checks whether a requested TF relation is usable for safety.
For example, a scan from `laser_frame` cannot be trusted in `base_link` when
their TF is absent or old.

```text
             /tf and /tf_static
                      |
                      v
             tf2_ros::Buffer
                      |
                      v
               TF Monitor Node
                 /          \
                v            v
     /tf_monitor/healthy   diagnostics
```

## Design

`TfHealthChecker` is pure C++. It validates timing configuration and maps an
observation to `OK`, `MISSING`, `TIMEOUT`, or `STALE`; `age <= threshold` is
healthy. `TfMonitorNode` owns its `Buffer` with `unique_ptr`, and its
`TransformListener` with `shared_ptr` for the node lifetime. The listener fills
the Buffer from `/tf` and `/tf_static`; a Timer performs a latest-time lookup.

With `allow_static_transform=true`, the Node observes `/tf_static` with
transient-local QoS. A directly configured static edge remains `OK` regardless
of timestamp age, because static TF is valid across all time. Composed static
paths should be monitored edge-by-edge.

TF2 requires its dedicated listener thread when lookup uses a non-zero timeout.
This is TF2 infrastructure, not an application `MultiThreadedExecutor`.

## Parameters and topics

| Parameter | Default | Meaning |
| --- | --- | --- |
| `parent_frame` | `base_link` | target/reference frame |
| `child_frame` | `laser_frame` | source/sensor frame |
| `check_period_ms` | `100` | timer period; must be positive |
| `lookup_timeout_ms` | `100` | TF lookup wait; non-negative |
| `stale_threshold_ms` | `500` | max dynamic TF age; non-negative |
| `allow_static_transform` | `true` | do not age a direct static TF |
| `output_prefix` | `/tf_monitor` | test-only alternative output prefix |

Default outputs are `/tf_monitor/healthy` (`std_msgs/Bool`),
`/tf_monitor/diagnostics` (`diagnostic_msgs/DiagnosticArray`), and
`/safety/tf/status` (`arm_navi_safety_interfaces/msg/SafetyStatus`). The Phase
5 adapter maps `OK` to valid `SAFE` and every TF failure to `ERROR` with
`data_valid=false`; its topic is configurable with `safety_status_topic`.
Diagnostics carry the frames, status, transform age, and stale threshold. Logs
occur only on status changes.

## Build, run, and test

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tf_monitor
source install/setup.bash
ros2 launch tf_monitor tf_monitor.launch.py parent_frame:=base_link child_frame:=laser_frame
colcon test --packages-select tf_monitor
colcon test-result --verbose
```

GTest covers fresh, stale, boundary, missing, timeout, static, and invalid
configuration. The launch test checks missing TF, fresh `OK`, stopped-TF
`STALE`, and a wrong child frame without a crash.

## Synthetic replay

No real AMR bag is included. The helper records `/tf` while publishing
`base_link -> laser_frame`: normal for 0–3 seconds, stopped for 3–5 seconds,
then normal for 5–8 seconds. It refuses to overwrite an existing bag.
It supplies a best-effort `/tf` QoS override so the rosbag recorder is
compatible with the TF broadcaster.

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p bags
cd bags
ros2 run tf_monitor record_synthetic_tf_bag.sh tf_monitor_synthetic_bag
```

Run the monitor with simulated time in one terminal:

```bash
ros2 launch tf_monitor tf_monitor.launch.py use_sim_time:=true
```

In another terminal, source Jazzy and the workspace, then run:

```bash
ros2 topic echo /tf_monitor/diagnostics
ros2 bag play --clock --disable-keyboard-controls /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws/bags/tf_monitor_synthetic_bag
```

Expected replay transition: `OK -> STALE -> OK`. This checks rosbag record/play
and monitoring behavior, not real AMR clock synchronization or a production TF
tree. A future real bag should include `/tf`, `/tf_static`, and `/scan` or
`/points`.
