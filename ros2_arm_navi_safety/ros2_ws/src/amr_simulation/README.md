# AMR simulation launch layer

Source the workspace and launch only the simulation layer:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_amr_navi_safety/ros2_arm_navi_safety/ros2_ws/install/setup.bash
ros2 launch amr_simulation simulation.launch.py world:=empty
```

Base worlds are `empty`, `maze`, `corridor`, `bookstore`, `warehouse`, and
`warehouse_detailed`. Every base world also has fixed-count `_0`, `_3`, and
`_10` variants.

- `warehouse` is a self-contained, offline six-rack layout with walls and box
  obstacles (the first attached-image style).
- `warehouse_detailed` is an offline detailed warehouse with pallet racks,
  cartons, a loading bay, and a building shell (the second attached-image
  style). It does not need Gazebo Fuel or internet access.

Fixed-count variants reuse their base world's SDF and include exactly 0, 3, or
10 collision-aware moving workers before Gazebo starts. Each world has its own
aisle-safe worker placement table. Workers continuously walk back and forth
between two waypoints; they are not the former one-way visual-only actors.
The suffix count takes precedence over the legacy `dynamic_obstacle` argument.

Use `dynamic_obstacle:=true` with `world:=warehouse` to spawn the scripted
worker. `headless:=true` runs the Gazebo server without the GUI. The launch
sets `GZ_SIM_RESOURCE_PATH` itself and starts only Gazebo, bridge, robot-state
publisher, and the 2D scan projector.

The Gazebo GUI defaults to the VM-compatible `ogre` renderer. Use
`gui_render_engine:=ogre2` only when the host GPU and graphics-memory budget
support Ogre 2. Headless runs enable Gazebo's offscreen renderer for the GPU
LiDAR.

## Safety LiDAR path

Gazebo Sim 8 / Ogre2 on some virtual GPUs has two relevant renderer defects:
one-row LiDAR targets can produce empty frames, and moving visuals can be absent
from the GPU sensor scene. The simulation launch therefore uses one dedicated
270-degree LiDAR proxy which follows `/odom`, renders two nearly coplanar rows,
and projects them to one 720-ray ROS `LaserScan`.

The worker motion controller repeatedly moves each solid collision model between
two world waypoints and publishes its pose. A worker yields before entering a
0.90 m clearance circle around the MiR's physical Gazebo pose, so the kinematic
worker cannot teleport into the chassis and push it away. The scan projector merges a
conservative 0.30 m worker radius into the rendered scan, so Nav2 costmaps and
Collision Monitor still see the worker if the virtual GPU omits its visual. A
fully empty renderer frame holds the last obstacle-bearing scan for at most
0.25 seconds, rejecting a single-frame dropout without delaying recovery after
the worker moves away.

The current VM-safe launch removes the MiR RGB-D render sensors because a second
render sensor makes this Gazebo/Ogre2 combination return empty LiDAR frames.
The camera bridge names remain reserved for compatibility, but camera messages
are intentionally unavailable in this safety-priority mode.

`scan_topic:=/scan/raw` remaps the projected 2D LiDAR output. Phase 15 uses it
to place a fault-injection relay before the public `/scan`. The
launch also bridges the selected world's Gazebo `SetEntityPose` service for
deterministic obstacle integration tests.

Fixed-count examples:

```bash
ros2 launch amr_simulation simulation.launch.py world:=warehouse_0
ros2 launch amr_simulation simulation.launch.py world:=warehouse_3
ros2 launch amr_simulation simulation.launch.py world:=warehouse_10
ros2 launch amr_simulation simulation.launch.py world:=maze_3
ros2 launch amr_simulation simulation.launch.py world:=corridor_10
ros2 launch amr_simulation simulation.launch.py world:=bookstore_3
ros2 launch amr_simulation simulation.launch.py world:=warehouse_detailed_10
```

The command interface is `/cmd_vel` (`geometry_msgs/msg/Twist`). Example:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.0}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.6}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

The ROS-side command input can be isolated with `cmd_vel_topic:=<topic>`. The
integrated Nav2 launch uses `/safety/cmd_vel`, so Gazebo accepts commands only
after the safety velocity gate.

For a finite-range sample each second while an actor is moving:

```bash
ros2 run amr_simulation scan_worker_check
```

There is intentionally no Nav2, SLAM, AMCL, mission manager, or safety node in
this package. To run the same worlds with real Nav2, AMCL, Mission Manager,
Safety Supervisor, and path validation, use:

```bash
ros2 launch mir_nav2_bringup nav2_world.launch.py world:=warehouse_0
```

See `mir_nav2_bringup/README.md` for installation, supported variants, and
validation commands.
