# AMR simulation assets

This package is the asset boundary for the ROS 2 Jazzy / Gazebo Harmonic testbed.
It contains no navigation, localization, mission, or safety code.

## World purpose

| World | Purpose |
| --- | --- |
| Empty | MiR hardware and sensor smoke test |
| MiR Maze | Basic obstacles and narrow spaces |
| Corridor | Narrow passages, turns, and blocking situations |
| Bookstore | Indoor shelf aisles and an open perimeter |
| Warehouse | Offline six-rack warehouse, walls, and box-obstacle tests |
| Any World 0 / 3 / 10 | Same base geometry with 0, 3, or 10 aisle-safe collision-aware workers |
| Warehouse Detailed | Offline warehouse with building, pallet racks, cartons, and loading bay |
| Warehouse + Worker | Dynamic-obstacle and future safety validation |

## Attribution and adaptation record

| Repository | License | Used material | Modified? | Source |
| --- | --- | --- | --- | --- |
| `Mostafasaad1/ros2_mir_nav2_pick_place` | MIT (repository); `mir_description/LICENSE.txt` is BSD-3-Clause | `mir_description`, `mir_control`, `mir_gazebo`, MiR Maze | Packages retained verbatim; Maze copied with Harmonic IMU-world-system addition | https://github.com/Mostafasaad1/ros2_mir_nav2_pick_place |
| `mertgulerx/autonomous-exploration-demo-benchmark` | Apache-2.0 | Corridor, Bookstore, Warehouse world definitions | Corridor copied verbatim. Bookstore, Warehouse, and Warehouse Detailed use local primitive-based adaptations. The unmodified Fuel-based warehouse remains in `upstream/` only as source material. | https://github.com/mertgulerx/autonomous-exploration-demo-benchmark |
| `Anastasios03git/autonomous-warehouse-amr` | **No LICENSE file found** | Actor trajectory and warehouse-layout reference | Only the path concept was used as a reference. The distributed worker is a new local primitive model; no upstream actor mesh or code is redistributed. The offline Warehouse was independently authored to recreate the visible six-rack layout. | https://github.com/Anastasios03git/autonomous-warehouse-amr |

License copies for the first two repositories are under `licenses/`. The worker
repository has no license file, so it is explicitly not represented as a locally
licensed asset.

## Resource policy

`amr_simulation` sets `GZ_SIM_RESOURCE_PATH` in its launch process; no shell
profile is modified. All distributed worlds and the collision-aware worker use
local geometry, so simulation startup does not need Gazebo Fuel or network
access.

## Dynamic worker model

The worker is a solid kinematic physics model with one continuous cylindrical
collision body, local human-shaped visuals, and inertia. The simulation motion
controller moves it repeatedly between two aisle-safe waypoints and publishes
the same pose to the Nav2 scan fallback. This replaces the former visual-only
`<actor>`, which allowed the robot and worker to pass through each other.

The worker's visual envelope covers its collision body at the robot LiDAR
height, so Gazebo's GPU LiDAR and the physics engine agree on its occupied
space. On virtual GPUs that omit moving visuals, `amr_simulation` projects this
same physics pose into `/scan` rather than treating the path as clear.
