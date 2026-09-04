# Gazebo Safety System Integration Test

## Purpose

This package is a system integration test, not another safety algorithm. It
connects the existing Phase 2-14 sensor, watchdog, supervisor, latency, mission,
and NavigateToPose interfaces to a real Gazebo Harmonic MiR100 simulation and
produces one deterministic PASS/FAIL result.

## Reused architecture

```text
Gazebo MiR100 -> /scan/raw -> Phase 8 fault relay -> /scan
                                      |                |
                                      v                v
                              Sensor Watchdog     LiDAR Safety
                                      \                /
                                       Safety Supervisor
                                              |
Nav2-compatible /navigation/cmd_vel -> Safety Velocity Gate -> /cmd_vel -> Gazebo
                                              ^
                                       Mission Manager
                                              ^
                                      Scenario Runner
```

The runner only injects events, invokes mission start/resume, observes existing
system outputs, and evaluates assertions. It never decides whether motion is
safe. Safety remains owned by `lidar_safety`, `sensor_watchdog`, and
`safety_supervisor`.

The repository does not contain `nav2_bringup`, a map, AMCL, or a Nav2 parameter
set, and Nav2 is not installed in the supplied environment. Therefore the
self-contained test defaults to `integration_test_navigator`, a deterministic test
implementation of the existing NavigateToPose action contract that drives the
real Gazebo robot. Set `use_test_navigator:=false` to connect an externally
configured Nav2 instance at `/navigate_to_pose`; its controller output must be
remapped to `/navigation/cmd_vel`.

## Scenario

The state machine is event-driven:

```text
INITIALIZING
 -> WAITING_FOR_NAVIGATION
 -> NORMAL_NAVIGATION
 -> WAITING_FOR_OBSTACLE_STOP
 -> WAITING_FOR_OBSTACLE_RECOVERY
 -> WAITING_FOR_OBSTACLE_RESUME
 -> WAITING_FOR_SENSOR_STOP
 -> WAITING_FOR_SENSOR_RECOVERY
 -> WAITING_FOR_SENSOR_RESUME
 -> FINAL_NAVIGATION
 -> PASSED | FAILED
```

Each active state has a wall-clock timeout, but time never causes a successful
transition. Transitions require ROS action/status, sensor, safety, mission,
velocity, and Gazebo service observations.

The default goal is `(x=5.0, y=0.0, yaw=0.0)` in `odom` and is configured in
`config/integration_scenario.yaml`. The runner waits for accepted NavigateToPose
navigation and measured motion before placing the local primitive
`integration_test_obstacle` 0.95 m ahead of the current robot pose. Its front face is
inside the 0.7 m LiDAR stop zone without spawning it in collision with the
robot. Removal moves the same entity to `(100, 100)` through Gazebo's
`SetEntityPose` service; no Fuel asset or network access is used.

For sensor failure, Gazebo publishes `/scan/raw`. The Phase 8-compatible relay
normally forwards it to `/scan`, then applies the existing `SetFaultMode`
`STOP_PUBLISH` request. Thus the actual scan stream stops and the existing
500 ms watchdog detects a real timeout. `NORMAL` restores forwarding.

## Safety stop and recovery

Navigation commands never reach Gazebo directly:

```text
/navigation/cmd_vel -> safety_velocity_gate -> /cmd_vel -> MiR100 DiffDrive
```

The gate starts closed, publishes zero immediately for STOP/ERROR/UNKNOWN or
invalid safety status, and also closes on stale safety or command input. A node
test verifies that a continuously non-zero navigation command becomes zero
during a safety fault.

The Mission Manager cancels the active NavigateToPose goal on safety stop while
retaining the current waypoint. Recovery does not resume on one clear frame.
The runner requires five consecutive clear obstacle reports, five healthy
watchdog reports, and five SAFE/WARNING aggregate safety reports before calling
the existing `/mission/resume` service. Sensor recovery likewise requires five
consecutive watchdog and aggregate-safety samples. Mission Manager then resends
the retained waypoint.

Nav2 costmap avoidance and the safety stop are intentionally different. A
distant obstacle belongs to planning/control. This test injects an obstacle in
the LiDAR emergency zone specifically to prove supervisor STOP and final
velocity gating.

## Build and run

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch system_integration_test integrated_safety_scenario.launch.py \
  headless:=true record_bag:=false
```

The automated wrapper launches, waits for the transient-local result topic,
prints the log path, and returns a shell failure code on FAIL or timeout. It
uses an isolated ROS domain by default so repeated simulations cannot exchange
stale discovery or topic data. Set `INTEGRATION_TEST_DOMAIN_ID` to select a
specific domain:

```bash
./src/system_integration_test/scripts/run_integration_scenario.sh
```

Enable the focused rosbag recording with either:

```bash
ros2 launch system_integration_test integrated_safety_scenario.launch.py record_bag:=true
RECORD_BAG=true ./src/system_integration_test/scripts/run_integration_scenario.sh
```

## Expected result

A PASS requires initialization, accepted navigation and measured movement,
obstacle detection, aggregate safety stop, stationary `/odom` and `/cmd_vel`,
stable obstacle recovery and resumed movement, real scan suppression, watchdog
timeout, sensor-fault stop, stable sensor recovery, second resume, and
`MissionStatus::COMPLETED`. Any missing critical assertion or step timeout
publishes FAIL on `/integration_test/result`.

Expected logs progress through:

```text
[PASS] ROS / Gazebo initialized
[PASS] NavigateToPose accepted
[PASS] Normal navigation started
[PASS] Obstacle injected / detected / Safety stop / Robot stopped
[PASS] Obstacle removed / Safety recovered / Navigation resumed
[PASS] Sensor fault injected / Watchdog timeout / Safety fault / Robot stopped
[PASS] Sensor restored / Watchdog healthy / Safety recovered / Navigation resumed
[PASS] Goal reached
Result: PASS
```

Measured timestamps are reported for obstacle detection, supervisor decision,
zero command, stationary robot, obstacle recovery, watchdog timeout, sensor
decision, and sensor recovery. Unavailable or non-monotonic measurements are
printed as `N/A`; no latency is synthesized.

## Tests and debugging

```bash
colcon test
colcon test-result --verbose
```

The GTests cover normal state transitions, unexpected events, terminal PASS,
critical failure, and step timeout. Launch tests cover the runner state/result
topics and timeout behavior, plus fail-safe command gating. The wrapper is the
headless Gazebo E2E test.

For a failure, inspect:

```bash
ros2 topic echo /integration_test/scenario_state
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /navigation/cmd_vel
ros2 topic echo /cmd_vel
ros2 topic echo /safety/lidar/status
ros2 topic echo /safety/watchdog/status
ros2 topic echo /safety/status
ros2 topic echo /mission/status
ros2 action info /navigate_to_pose
```

On VMware without 3D acceleration, EGL warnings may appear. Acceptance is based
only on ROS topics, action/status transitions, Gazebo entity state, velocity,
and timestamps—not GUI FPS or real-time factor.
