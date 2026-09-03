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
| Any World 0 / 3 / 10 | Same base geometry with 0, 3, or 10 aisle-safe scripted workers |
| Warehouse Detailed | Offline warehouse with building, pallet racks, cartons, and loading bay |
| Warehouse + Worker | Dynamic-obstacle and future safety validation |

## Attribution and adaptation record

| Repository | License | Used material | Modified? | Source |
| --- | --- | --- | --- | --- |
| `Mostafasaad1/ros2_mir_nav2_pick_place` | MIT (repository); `mir_description/LICENSE.txt` is BSD-3-Clause | `mir_description`, `mir_control`, `mir_gazebo`, MiR Maze | Packages retained verbatim; Maze copied with Harmonic IMU-world-system addition | https://github.com/Mostafasaad1/ros2_mir_nav2_pick_place |
| `mertgulerx/autonomous-exploration-demo-benchmark` | Apache-2.0 | Corridor, Bookstore, Warehouse world definitions | Corridor copied verbatim. Bookstore, Warehouse, and Warehouse Detailed use local primitive-based adaptations. The unmodified Fuel-based warehouse remains in `upstream/` only as source material. | https://github.com/mertgulerx/autonomous-exploration-demo-benchmark |
| `Anastasios03git/autonomous-warehouse-amr` | **No LICENSE file found** | Actor trajectory and warehouse-layout reference | The worker trajectory was adapted. The offline Warehouse was independently authored to recreate the visible six-rack layout; upstream source is not redistributed. Its actor skin remains a runtime Fuel URL. Obtain a clear asset license before redistributing the worker. | https://github.com/Anastasios03git/autonomous-warehouse-amr |

License copies for the first two repositories are under `licenses/`. The worker
repository has no license file, so it is explicitly not represented as a locally
licensed asset.

## Resource policy

`amr_simulation` sets `GZ_SIM_RESOURCE_PATH` in its launch process; no shell
profile is modified. Bookstore, `warehouse`, and `warehouse_detailed` contain
only local geometry, so they do not need Fuel. The optional worker's walking
mesh is still an upstream Fuel URL and needs a network-accessible Fuel cache on
its first use.

## Known Gazebo actor limitation

Gazebo actors follow scripted trajectories; they are not physics bodies and have
no collision geometry. A GPU LiDAR can observe the rendered actor, but physical
collision validation requires a separately modeled collision proxy or a future
actor-control plugin. This package does not install the optional
`gazebo-ros-actor-plugin`.
