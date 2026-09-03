# lidar_safety (Phase 2)

`lidar_safety_node` examines the front angular region of `/scan` and publishes:

- `/safety/lidar/obstacle_detected` (`std_msgs/Bool`)
- `/safety/lidar/min_distance` (`std_msgs/Float32`)
- `/safety/lidar/status` (`arm_navi_safety_interfaces/msg/SafetyStatus`)

The Phase 5 status adapter maps a valid clear ROI to `SAFE`, a valid obstacle
to `STOP`, and no valid ROI measurement to `ERROR` with `data_valid=false`.
It preserves the original publishers and does not change the LiDAR decision
core.

The default parameters are `scan_topic=/scan`, `stop_distance_m=0.7`, and
`front_angle_deg=30.0`, which selects angles from -30 to +30 degrees. A point at
exactly the stop distance is an obstacle (`distance <= stop_distance_m`).

## Safety policy

The Core ignores NaN, infinity, values below `range_min`, and values above
`range_max`. It calculates each beam angle as `angle_min + index *
angle_increment`, then checks that physical angle against both the scan angular
bounds and the front ROI. It never selects array indexes by position alone.

When there is no valid point in the front ROI, it publishes `min_distance=+inf`
and `obstacle_detected=false`. This prevents a missing measurement from being
mistaken for a zero-distance obstacle. The node logs only when obstacle state
changes, rather than for every scan callback.

`LidarSafetyCore` is pure C++ and contains this decision. `LidarSafetyNode`
adapts ROS `LaserScan` messages, reads parameters, and handles topic I/O. This
makes the safety rule testable without a ROS graph.

## Build and automated tests

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select lidar_safety
colcon test-result --verbose
```

The GTest suite checks the pure decision logic. The launch test runs the actual
node, publishes `LaserScan`, and verifies both output topics; it therefore
catches ROS publisher/subscriber and parameter-wiring failures that a unit test
cannot.

## Manual Node Test

Open four terminals. Source ROS Jazzy and the workspace in every terminal.

Terminal 1:

```bash
cd /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run lidar_safety lidar_safety_node
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws/install/setup.bash
ros2 topic echo /safety/lidar/obstacle_detected
```

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws/install/setup.bash
ros2 topic echo /safety/lidar/min_distance
```

Terminal 4:

```bash
source /opt/ros/jazzy/setup.bash
source /home/jacob-dev/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws/install/setup.bash
ros2 run lidar_safety publish_test_scan.py
```

The test publisher emits seven deterministic beams over -90 to +90 degrees:
2.0 m for two seconds, then 0.4 m for two seconds, then 2.0 m again. It exists
because a hand-written `ros2 topic pub` LaserScan command is fragile and hard to
read. Unlike a physical LiDAR, it has no real geometry, timing jitter, or sensor
noise; it is solely a reproducible Topic I/O test.

Expected values are normal: `false`, `2.0`; obstacle: `true`, `0.4`; then normal
again. The node terminal should show the matching obstacle-detected and
obstacle-cleared state-transition logs.

## RViz (optional)

With a real scan source or the test publisher running, start `rviz2`, set Fixed
Frame to the scan message's `frame_id` (the test publisher has none, so this is
primarily for a real sensor), and add a **LaserScan** display on `/scan`. Compare
the visible front beams with `ros2 topic echo` of the two safety outputs.

## Replay Test

No rosbag has been selected by this package. Replay remains pending until a bag
source is chosen; see the project-level Phase 2 report for the available options.
