# Nav2 path validator

`nav2_path_validator` is an observer-only validation package. It does not plan,
control, send goals, or replace any Nav2 component. A successful
`NavigateToPose` result only says that the action reached its terminal success
state; this package independently checks that a usable global path existed,
that the robot followed it within measured error, and that the final pose is
within validation tolerances.

## Repository audit (2026-09-05)

The checked-in workspace does **not** contain a production Nav2 bringup or
parameter YAML, and the host only exposes the workspace's development-only
`nav2_msgs` package. No ROS nodes were running during the audit, so runtime
topic discovery returned an empty graph.

| Item | Verified current state |
|---|---|
| Planner plugin | Not present; there is no `planner_server` configuration |
| Controller plugin | Not present; there is no `controller_server` configuration |
| Goal checker | Not present |
| Global/local costmaps | Not present; no layer or `/scan` observation-source configuration exists |
| Map/localization | No map server, AMCL, or SLAM bringup is checked in |
| Phase 15 navigator | `integration_test_navigator`, explicitly a deterministic velocity-driving test double |
| Simulation data | `/odom`, `/tf`, `/tf_static`, `/scan`, `/cmd_vel` are provided/bridged |

Consequently this repository cannot honestly produce real planner/controller,
clear-path, obstacle-path, or Gazebo+Nav2 measurements yet. This validator is
ready to attach when a production Nav2 bringup is installed and supplied. It
never treats Phase 15's test navigator as evidence of Nav2 planning.

## Pipeline and observations

```text
requested goal (/mission/active_goal)
              |
              v
Nav2 planner --/plan--> global path
              |
              v
Nav2 controller --/received_global_plan, /cmd_vel--> robot
              |
              v
map -> base_link TF --> actual trajectory
              |
              v
metrics + PASS/FAIL + CSV + RViz
```

The path is where the planner intends to go. The trajectory is where the robot
actually went. Both are expressed in `validation_frame` (normally `map`). The
default pose source directly samples `map -> base_link`. The optional `odom`
source transforms each odometry pose into the validation frame before use; it
never compares `odom` and `map` coordinates directly.

Mission Manager now republishes the exact goal handed to the NavigateToPose
action on the transient-local `/mission/active_goal` topic. This is necessary
because action goal requests are services and are not otherwise observable as
a normal goal topic.

## Metrics and decisions

- Initial/latest global path length: sum of XY segment lengths.
- Actual travel distance: sum of accepted trajectory sample segments.
- Cross-track error: shortest distance from each trajectory sample to any
  segment of the global path active at that sample time. Mean, RMSE, and maximum
  are reported.
- Path start error: first initial-path pose to the first robot sample.
- Path-goal error: latest path endpoint to the requested goal.
- Goal position/yaw error: final robot pose to the requested goal, using the
  normalized shortest angular difference.
- Travel efficiency: actual distance divided by initial path length. It is an
  analysis value, not a hard PASS condition.
- Replan count: the new path is sampled along its arc and each sample is
  measured against segments of the previous path. A count is added only when
  the maximum directed deviation exceeds `replan_change_threshold`. This
  ignores a shortened prefix of the same route and repeated publications.
- Local-plan received: reported separately. Absence is informative but is not
  currently a required PASS condition because controller plugins expose
  different debug topics.

A dynamic obstacle need not cause a global replan. A local controller may
handle it while the global path remains stable, and a safety-zone intrusion may
instead stop the robot. The report records these facts without declaring
failure solely because `replan_count == 0`.

Default validation limits are:

```yaml
path_goal_tolerance: 0.25
max_tracking_rmse: 0.30
max_tracking_error: 0.70
goal_position_tolerance: 0.25
goal_yaw_tolerance_deg: 15.0
replan_change_threshold: 0.20
trajectory_sample_distance: 0.02
```

PASS requires an accepted and successful navigation, a goal, a valid path with
at least two poses in the validation frame, an endpoint near the requested
goal, at least two trajectory samples, tracking within both thresholds, and a
final pose within both goal tolerances. These are validator tolerances, separate
from Nav2's goal checker.

## Topic contract

The following defaults are configurable. `/plan` and `/received_global_plan` are
conventional Nav2 names and must be confirmed on the actual running graph; they
were not discovered from a running stack in this repository.

| Purpose | Default | Type |
|---|---|---|
| Global path input | `/plan` | `nav_msgs/msg/Path` |
| Controller tracking path | `/received_global_plan` | `nav_msgs/msg/Path` |
| Requested goal | `/mission/active_goal` | `geometry_msgs/msg/PoseStamped` |
| Odometry fallback | `/odom` | `nav_msgs/msg/Odometry` |
| Mission completion | `/mission/status` | `arm_navi_safety_interfaces/msg/MissionStatus` |
| Direct Nav2 completion | `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` |
| Actual trajectory output | `/nav2_path_validation/actual_trajectory` | `nav_msgs/msg/Path` |
| State output | `/nav2_path_validation/state` | `std_msgs/msg/String` |
| JSON result output | `/nav2_path_validation/result` | `std_msgs/msg/String` |

Before connecting a real stack, inspect it and override names rather than
assuming defaults:

```bash
ros2 node list
ros2 action list -t
ros2 topic list -t
ros2 topic info /plan --verbose
ros2 topic info /received_global_plan --verbose
ros2 topic echo /plan --once
ros2 param get /planner_server planner_plugins
ros2 param get /controller_server controller_plugins
ros2 param dump /global_costmap/global_costmap
ros2 param dump /local_costmap/local_costmap
```

Record the actual planner, controller, goal checker, costmap plugins, and LiDAR
observation source in this README when that configuration is added. Do not infer
an algorithm name from a plugin family.

## Build and unit/node tests

```bash
cd ~/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to nav2_path_validator mission_manager --symlink-install
source install/setup.bash
colcon test --packages-select nav2_path_validator mission_manager --event-handlers console_direct+
colcon test-result --verbose
```

The GTest suite covers path/trajectory length, point-to-segment distance,
nearest-segment cross-track error, mean/RMSE/maximum, goal position, angle
normalization, yaw error, empty and single-pose handling, repeated paths, route
prefix removal, and changed geometry. The launch test publishes a straight
mock path plus a close odometry trajectory without Gazebo and verifies the
machine-readable PASS result.

## Run with real Nav2

The repository now provides a production simulation/localization/Nav2 launch:

```bash
ros2 launch mir_nav2_bringup nav2_world.launch.py world:=warehouse_0
```

That launch starts this validator automatically. To observe another already
running Nav2 stack instead, launch only the validator:

```bash
ros2 launch nav2_path_validator nav2_path_validation.launch.py \
  scenario:=clear \
  global_path_topic:=/actual/global/path \
  local_path_topic:=/actual/local/path
```

For a direct Nav2 action workflow without Mission Manager, publish/copy the
requested PoseStamped on a goal topic visible to the validator and use:

```bash
ros2 launch nav2_path_validator nav2_path_validation.launch.py \
  goal_topic:=/goal_pose completion_source:=action_status
```

Set `rviz:=false` for headless operation and `record_bag:=true` to record the
selected navigation evidence. The RViz configuration shows map, global/local
costmaps, global/local paths, scan, robot model, TF, and the red actual
trajectory together. If real topic names differ, update the RViz displays too.

CSV output is written under `results_directory` after completion:

```text
nav2_path_validation_YYYYMMDD_HHMMSS.csv
nav2_path_validation_YYYYMMDD_HHMMSS_planned_path.csv
nav2_path_validation_YYYYMMDD_HHMMSS_actual_trajectory.csv
```

The manual finalizer exists for controlled tests and non-action data replay:

```bash
ros2 service call /nav2_path_validation/finalize_success std_srvs/srv/Trigger '{}'
```

Do not use that service as proof that a real NavigateToPose action succeeded.

## Clear path versus static obstacle

Run the same start and goal twice, once without the obstacle (`scenario:=clear`)
and once with it (`scenario:=obstacle`). Compare the two generated planned-path
files geometrically:

```bash
ros2 run nav2_path_validator compare_path_runs.py \
  results/clear_planned_path.csv results/obstacle_planned_path.csv \
  --threshold 0.20
```

The tool reports each path length and a symmetric segment-based geometry
deviation, exits successfully only when the deviation exceeds the threshold,
and does not compare CSV text or pose counts.

## Phase 15 boundary

The validator can be launched alongside Phase 15 as an observer once Phase 15
is run with `use_test_navigator:=false` and an external production Nav2 stack.
With the default test navigator there is no global/local path, planner,
controller, costmap, or replan to validate, so a path-validation result should
correctly fail rather than claim Nav2 coverage. Safety stop/recovery remains the
responsibility of Phase 15; the validator only records the resulting trajectory
and any path change.

## Known limitations

- Real Gazebo+Nav2 E2E execution requires the host packages
  `ros-jazzy-navigation2` and `ros-jazzy-nav2-bringup`.
- Cross-track error is planar and intentionally ignores Z for AMR navigation.
- Tracking samples are associated with the most recently received global path;
  this matches replanning operation but is not a time-synchronized rosbag
  offline estimator.
- RViz topic substitutions are static in the provided config; edit/remap the
  display topics when the running graph differs.
