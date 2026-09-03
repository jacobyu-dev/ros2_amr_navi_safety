# Phase 10 Mission Manager

`mission_manager_node` accepts parameter-configured waypoint missions through
standard `std_srvs/srv/Trigger` services.  This keeps terminal usage and tests
simple without introducing a fleet-management protocol:

* `/mission/start`, `/mission/cancel`, `/mission/pause`, `/mission/resume`
* `/mission/status` (`arm_navi_safety_interfaces/msg/MissionStatus`)
* `/safety/status` (`SafetyStatus`) is the sole safety input.

`SAFE` and `WARNING` permit motion. `STOP`, `ERROR`, `UNKNOWN`, or invalid
safety data pause an active mission, cancel the Nav2 goal, and retain the
current waypoint. A later safe status never restarts it; `/mission/resume` is
required.

## Synchronization

Safety input is reentrant, while user services are mutually exclusive. A single
short-lived mutex protects the compound mission state, active action handle,
goal sequence, and cancellation reason. ROS actions are sent/canceled only
after releasing the mutex. Goal sequence numbers discard delayed results from a
previous, safety-canceled goal after resume. This is safe under the existing
MultiThreadedExecutor model without holding a lock across ROS work.

## Nav2 development fallback

This repository does not have Nav2 installed. The local `nav2_msgs` package is
a development-only definition of the standard `NavigateToPose` action, so the
Mission Manager still uses `rclcpp_action::Client<nav2_msgs::action::NavigateToPose>`.
Remove that local fallback when building in a system that supplies Nav2's real
`nav2_msgs`; the action contract is the same. `fake_navigate_to_pose_server`
returns deterministic `success`, `abort`, or `hold` outcomes for tests.

## Phase 12 — Safety Supervisor + Mission Manager integration

### Purpose and architecture

Phase 12 keeps the Safety Supervisor and Mission Manager as independent ROS 2
nodes. The manager does not inspect sensors and the supervisor does not issue
navigation goals. `SafetyStatus` is the single integration boundary.

```text
LiDAR Safety ----+
TF Monitor ------+--> Safety Supervisor -- /safety/status --> Mission Manager
Sensor Watchdog --+                                         |
                                                           v
                                              NavigateToPose / Nav2
```

| SafetyStatus level | Mission policy |
| --- | --- |
| `SAFE` | Start is permitted; an existing mission keeps navigating. |
| `WARNING` | Start is permitted; the existing supervisor policy treats it as a warning only. |
| `STOP` | A running mission becomes `PAUSED`, its current Nav2 goal is canceled, and its waypoint is retained. |
| `ERROR`, `UNKNOWN`, or `data_valid=false` | Same fail-safe pause/cancel policy as `STOP`; `ERROR` is the project's highest-severity fault state. |

The existing interface has no `EMERGENCY` value. `SafetyStatus.ERROR` is
therefore the fault/emergency-equivalent state in this project. It blocks
ordinary start and resume commands until the supervisor reports a valid
`SAFE`/`WARNING` state.

### Priority, recovery, and concurrency

Safety has priority over all mission services:

```text
Mission start + concurrent STOP  --> PAUSED, never left NAVIGATING
STOP / ERROR + start or resume   --> rejected
STOP -> SAFE                     --> remains PAUSED
SAFE + explicit /mission/resume  --> returns to NAVIGATING
```

The short-lived `state_mutex_` protects safety permission, lifecycle state,
the active action handle, sequence number, and cancel reason as one compound
state. No ROS action operation is made while it is locked. A safety stop that
arrives before the action server accepts a goal is still safe: the later goal
acceptance callback sees the paused state and immediately cancels that goal.
Repeated STOP messages are idempotent: only the first transition requests a
Nav2 cancel.

The safety output now preserves the existing optional Phase 9 correlation ID
and adds `decision_time_steady_ns` when the Supervisor publishes its aggregate
decision. On the first Mission Manager stop transition, it logs
Safety-to-Mission and, when fault data is available, end-to-end latency. This
extends the existing measurement path without adding a separate benchmark
protocol.

### ROS interfaces

| Direction | Endpoint | Type |
| --- | --- | --- |
| Subscribe | `/safety/status` | `arm_navi_safety_interfaces/msg/SafetyStatus` |
| Publish | `/mission/status` | `arm_navi_safety_interfaces/msg/MissionStatus` |
| Service | `/mission/start`, `/mission/cancel`, `/mission/pause`, `/mission/resume` | `std_srvs/srv/Trigger` |
| Action client | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |

### Run and manually verify

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mission_manager safety_mission_integration.launch.py
```

In another terminal, inspect the integration and start a mission after the
supervisor becomes `SAFE`:

```bash
ros2 topic echo /safety/status
ros2 topic echo /mission/status
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

The launch starts the existing fake LiDAR pipeline. Trigger its existing fault
injector to verify the end-to-end stop path:

```bash
ros2 service call /fake_lidar/set_fault_mode \
  arm_navi_safety_interfaces/srv/SetFaultMode "{mode: 1}"
ros2 service call /fake_lidar/set_fault_mode \
  arm_navi_safety_interfaces/srv/SetFaultMode "{mode: 0}"
ros2 service call /mission/resume std_srvs/srv/Trigger '{}'
```

Mode `1` is the existing `STOP_PUBLISH` injection. Recovery to `SAFE` does not
move the robot; the final explicit resume is intentional. There is no suitable
combined sensor rosbag in this repository, so replay testing remains pending.

### Tests

`test_mission_state_machine` covers the pure lifecycle transitions. The
Phase 12 launch test starts a real `SafetySupervisorNode`, the manager, and
the existing fake NavigateToPose server. It verifies normal navigation,
Supervisor STOP propagation, one cancel for repeated STOP, start blocking,
explicit recovery, and `ERROR` fault blocking:

```bash
colcon test --packages-select mission_manager safety_supervisor
colcon test-result --verbose
```

`safety_test_tools/test/test_fault_injection_pipeline.py` remains the existing
sensor-timeout fault-injection coverage. Launching
`safety_mission_integration.launch.py` connects that same LiDAR → watchdog →
Supervisor path to the manager for manual end-to-end verification.

## Phase 13 — Nav2 `NavigateToPose` integration

Phase 13 retains the Phase 10–12 lifecycle policy and exposes a direct goal
input for Nav2. It deliberately does not implement a planner, controller,
SLAM, AMCL, or a simulation world: Nav2 owns those responsibilities.

```text
/mission/goal_pose (PoseStamped)              /safety/status (SafetyStatus)
                 |                                         |
                 v                                         v
          Mission Manager ------------------> safety gate / pause policy
                 |
                 | rclcpp_action::Client<NavigateToPose>
                 v
       /navigate_to_pose action server (Nav2 BT Navigator)
                 |
                 v
       Nav2 planner / controller -> cmd_vel
```

### Goal, feedback, result, and cancellation

`/mission/goal_pose` accepts `geometry_msgs/msg/PoseStamped`; its supplied
`frame_id`, position, and quaternion are forwarded unchanged to
`NavigateToPose`. The configurable `goal_pose_topic` defaults to that name.
The older parameter-configured waypoint mission and `/mission/start` service
remain available for compatibility.

The manager checks the action server without blocking an executor callback.
If it is unavailable, the request is rejected and the node remains alive.
Accepted goals use the existing `RUNNING` / `NAVIGATING` state. An action
rejection or `ABORTED` result maps to `FAILED`; `SUCCEEDED` advances the
waypoint and eventually maps to `COMPLETED`; cancellation retains the existing
Phase 12 policy (`PAUSED` for safety/manual pause, `CANCELED` for a user
cancel). The action feedback's `distance_remaining` and
`number_of_recoveries`, plus a manager-measured `navigation_time_sec`, are
published in the existing `/mission/status` message. A negative
`distance_remaining` means no feedback has yet arrived.

`STOP`, `ERROR`, `UNKNOWN`, or invalid SafetyStatus data prevents new goals
and cancels an active action exactly once. This project has no `EMERGENCY`
enumerator: `ERROR` is the emergency-equivalent level. Recovery to `SAFE` or
`WARNING` never resends a goal; `/mission/resume` is still explicit.

### Thread safety

The existing short-held state mutex protects action handles, goal sequence,
mission/safety state, cancellation reason, and feedback fields. Action send
and cancel operations happen after releasing it. Sequence numbers discard late
feedback/results from a canceled pre-resume goal, so action, safety, and
service callbacks can run under the existing `MultiThreadedExecutor` without
changing the lifecycle policy.

### Fake Nav2 integration and tests

`fake_navigate_to_pose_server` is a test-only compatible action server. Its
`result_sequence` parameter supports `success`, `abort`, and `hold`; setting
`reject_goals:=true` makes it reject incoming goals. The launch test covers
success, abort, held-goal cancellation, external PoseStamped input with
feedback, goal rejection, safety blocking, and explicit resume behavior.

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to mission_manager
source install/setup.bash
colcon test --packages-select mission_manager safety_supervisor
colcon test-result --verbose
```

### CLI verification

Terminal 1 — start the Fake NavigateToPose server and Mission Manager (the
demo intentionally disables the safety-clear prerequisite):

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch mission_manager mission_manager_fake_nav.launch.py
```

Terminal 2 — observe state and feedback:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic echo /mission/status
```

Terminal 3 — submit a direct Nav2 goal (the default fake server holds it, so
use `/mission/cancel` or `/mission/pause` to observe cancellation):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic pub --once /mission/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
```

Expected state is `RUNNING/NAVIGATING` after the goal and
`CANCELED/NAVIGATION_CANCELED` after the service call. For a complete safety
pipeline use `safety_mission_integration.launch.py`; inject the existing
LiDAR fault as documented above and observe `PAUSED/NAVIGATION_PAUSED` and a
single action cancellation.

### Real Nav2 verification and replay preparation

Start the site's existing Nav2 launch in Terminal 1, then verify the standard
server contract before starting this package's `mission_manager.launch.py` in
Terminal 2:

```bash
ros2 action list | grep navigate_to_pose
ros2 action info /navigate_to_pose
```

The action type must be `nav2_msgs/action/NavigateToPose`. Bring up the Safety
Supervisor (or explicitly supply a valid SAFE status), submit the
`/mission/goal_pose` command above, and observe feedback and the terminal
state on `/mission/status`. In an environment with Nav2's real `nav2_msgs`,
remove the repository's development-only local `nav2_msgs` fallback from the
overlay so the system package is used.

No combined sensor rosbag is stored in this repository. To prepare replay,
record `/scan`, `/tf`, `/tf_static`, and the safety source topics while
navigating, then replay them while the Supervisor, Manager, and a Fake/real
NavigateToPose server run. Verify that a replayed STOP produces one cancel and
that a subsequent SAFE does not automatically resume navigation.
