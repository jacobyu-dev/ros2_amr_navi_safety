# MiR Nav2 bringup

This package runs the existing MiR Gazebo worlds with the real ROS 2 Jazzy
Nav2 stack, simulation-stable localization, the project safety pipeline, Mission Manager,
and the Nav2 path validator.

## One-time installation

```bash
sudo apt-get update
sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

The old source package `src/nav2_msgs` is a development-only action stub and
now contains `COLCON_IGNORE`. After installing Nav2, remove only its stale
generated artifacts before rebuilding:

```bash
cd ros2_arm_navi_safety/ros2_ws
rm -rf build/nav2_msgs install/nav2_msgs
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run mir_nav2_bringup check_nav2_prerequisites.sh
```

## Run

```bash
ros2 launch mir_nav2_bringup nav2_world.launch.py world:=warehouse_0
```

Supported base worlds are `empty`, `maze`, `corridor`, `bookstore`,
`warehouse`, and `warehouse_detailed`. Each also accepts `_0`, `_3`, and `_10`
dynamic-worker variants. Use `headless:=true rviz:=false` for CI.

The launch starts Gazebo, Map Server, all Nav2 navigation servers, the safety
sensors/supervisor/velocity gate, Mission Manager, the path validator, and
RViz. By default, `localization_mode:=simulation` continuously aligns wheel
odometry with the MiR model's physical Gazebo pose. This prevents moving
workers and the warehouse's repeated rack geometry from making AMCL jump to a
different aisle, and also follows any physical displacement caused by contact.

Use AMCL only when the purpose of the run is to test localization itself:

```bash
ros2 launch mir_nav2_bringup nav2_world.launch.py \
  world:=warehouse_3 localization_mode:=amcl
```

AMCL mode receives the matching world spawn pose automatically, but dynamic
workers can still expose real scan-localization ambiguity. Obstacle avoidance
and safety tests should keep the default simulation mode.

Nav2 commands follow this non-bypassable topic chain:

```text
Nav2 controller -> velocity smoother -> Nav2 collision monitor
  -> /navigation/cmd_vel -> safety_velocity_gate -> /safety/cmd_vel -> Gazebo
```

To start the configured example mission after the safety state becomes clear:

```bash
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

You can also use RViz's `2D Goal Pose`, which sends directly to Nav2. For a
full safety-aware mission test, prefer `/mission/start`.

The `nav2_world` launch automatically resumes a mission paused by a safety
obstacle after `/safety/status` remains motion-permitted for one second. This
delay lets the local and global costmaps clear the moving obstacle's old
position. Use `auto_resume:=false` when an explicit `/mission/resume` command
is required instead.

If Nav2 itself aborts because a moving obstacle temporarily leaves no valid
path, the launch retains the current waypoint and retries it up to five times
at two-second intervals. Set `navigation_retries:=0` to restore immediate
failure behavior.

Useful checks:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /plan --once
ros2 topic echo /received_global_plan --once
ros2 topic info /navigation/cmd_vel -v
ros2 topic info /cmd_vel -v
```

In `localization_mode:=amcl`, `ros2 lifecycle get /amcl` is also available.

## Maps

The committed PGM files are deterministically rasterized from each world's
static SDF box collision geometry. Dynamic workers are deliberately excluded.
Regenerate them after changing a world collision:

```bash
ros2 run mir_nav2_bringup generate_maps.py \
  --assets-root src/amr_simulation_assets \
  --output src/mir_nav2_bringup/maps
```
