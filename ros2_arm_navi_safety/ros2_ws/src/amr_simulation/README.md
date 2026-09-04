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

Fixed-count variants reuse their base world's SDF and spawn exactly 0, 3, or
10 scripted workers. Each world has its own aisle-safe worker placement table.
The suffix count takes precedence over the legacy `dynamic_obstacle` argument.

Use `dynamic_obstacle:=true` with `world:=warehouse` to spawn the scripted
worker. `headless:=true` runs the Gazebo server without the GUI. The launch
sets `GZ_SIM_RESOURCE_PATH` itself and starts only Gazebo, bridge, robot-state
publisher, and entity spawners.

The Gazebo GUI defaults to the VM-compatible `ogre` renderer. Use
`gui_render_engine:=ogre2` only when the host GPU and graphics-memory budget
support Ogre 2. Headless runs do not start either GUI renderer.

`scan_topic:=/scan/raw` remaps the ROS side of the Gazebo LiDAR bridge. Phase
15 uses it to place a fault-injection relay before the public `/scan`. The
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

For a finite-range sample each second while an actor is moving:

```bash
ros2 run amr_simulation scan_worker_check
```

There is intentionally no Nav2, SLAM, AMCL, mission manager, or safety node in
this package.
