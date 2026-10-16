# Navigation demo

This demo plans an A* route through the current occupancy grid and publishes it
for visualization. It publishes no actuator command. The default setup targets
VRX's `wayfinding_task` world.

## Run

Build the new `boat_interfaces` and `boat` packages with your normal colcon
workflow. Start the world in one terminal:

```bash
make vrx-nav WORLD=wayfinding_task
```

After Gazebo has loaded, open another terminal:

```bash
make sh
ros2 launch boat navigation_demo.launch.py
```

The launch starts five small processes:

- `gazebo_obstacles` spawns visible, collidable spheres and moves them along
  trajectories in `boat/config/moving_obstacles.yaml`.
- `sim_truth` reads Gazebo poses and GNSS, rasterizes the selected world's
  collision geometry, and publishes the input topics below.
- `navigation_monitor` subscribes and prints a one-second input summary.
- `mission_manager` publishes the demo goal at `(-593, 241)` in the `map` frame.
- `astar_navigation` replans once a second and publishes `/navigation/planned_path`
  (`nav_msgs/msg/Path`) for RViz.

## Code layout

The Python code lives in `boat/boat/navigation/`, grouped by responsibility:

| Location | Responsibility |
| --- | --- |
| `mission_manager.py` | Publish one mission goal at a time and advance on task feedback |
| `navigation_monitor.py` | Log navigation inputs for diagnostics; no control output |
| `planning/astar_navigation.py` | Subscribe to the map, boat pose and goal; publish a path |
| `planning/grid_astar.py` | Eight-connected A* over free occupancy cells |
| `simulation/sim_truth.py` | Publish perfect boat state, GNSS, map, and tracks |
| `simulation/scenario_map.py` | Rasterize Gazebo collision geometry into the grid |
| `simulation/gazebo_obstacles.py` | Spawn and move physical Gazebo models |
| `simulation/obstacle_trajectory.py` | Parse and evaluate configured obstacle paths |

The ROS messages remain in `boat_interfaces/msg/`, the launch description in
`boat/launch/`, and the trajectory YAML in `boat/config/`. The shared
`scripts/apply_environment.py` helper copies a VRX world before launch. For the
navigation targets, its `--keep-running` option gives each VRX scoring plugin
an unlimited running duration and exit delay, and disables automatic shutdown.
Standard competition targets omit that option.

The mission manager publishes `/mission/goal` (`PoseStamped`) and
`/mission/state` (`String`), and advances when it receives
`/mission/task_done` (`Bool`). The demo does not publish task completion or move
the boat, so its single goal remains active. The demo launch gives this goal a
one-hour simulation-time timeout for inspection.

If the packages have not been built in the running container, use your normal
`make colcon` workflow first.

## ROS inputs

| Topic | Type | Meaning |
|---|---|---|
| `/navigation/gps_fix` | `sensor_msgs/msg/NavSatFix` | Gazebo GNSS latitude, longitude, altitude |
| `/navigation/gps_velocity` | `geometry_msgs/msg/Vector3Stamped` | Gazebo GNSS east, north, up velocity in m/s |
| `/navigation/boat_odometry` | `nav_msgs/msg/Odometry` | Exact Gazebo model pose; planar twist estimated from successive model poses |
| `/navigation/occupancy_map` | `nav_msgs/msg/OccupancyGrid` | Static collision map plus current moving-obstacle footprints |
| `/navigation/dynamic_objects` | `boat_interfaces/msg/TrackedObjectArray` | Current observed pose, ID, class, radius and estimated velocity |
| `/mission/goal` | `geometry_msgs/msg/PoseStamped` | Current demo target in the `map` frame |
| `/navigation/planned_path` | `nav_msgs/msg/Path` | Latest A* route from the boat to the goal |

The grid and tracks use Gazebo's local east-north-up `map` frame in metres.
Grid cells are `0` (free) or `100` (occupied). The default grid spans x
`[-650, -300)` and y `[-50, 350)` at 0.5 m per cell. Its bounds are ROS node
parameters on `sim_truth`; change them for another part of the venue. Cells
outside this bounded map have no representation.

The static grid is derived from the **generated world SDF and the installed or
cached model collision geometry**, including the Sydney shoreline collision
mesh, buoy cylinders and other props. Water and seabed planes are excluded.
Within the grid, areas with no modeled collision are treated as free; this is
not a depth or navigability guarantee. The map is for testing the navigation
interface against this VRX scenario, not a survey of real water.

The dynamic model positions in the grid and object list come from Gazebo's
world pose topic, so both reflect the physical models' current simulated poses.
Their velocity is estimated between successive published positions. The models
are fixed-height colliders moved through Gazebo's pose service; they do not have
boat dynamics. Edit `moving_obstacles.yaml` and restart the launch to change
their radius, speed, and XY waypoint path. An empty `obstacles_file` parameter
disables them.

The planner uses occupied cells from the latest grid, including each moving
obstacle's current footprint. Its `inflation_radius_m` ROS parameter defaults
to 3.0 m, keeping planned cell centres at least that far from occupied cell
centres. This is a starting value for the simulated WAM-V footprint; it is not a
verified safety margin for the real boat. To adjust it while running, use
`ros2 param set /astar_navigation inflation_radius_m 2.5`. The next plan uses
the new radius. A larger radius can leave no route if the boat already starts
too close to an occupied cell. The planner does not predict future obstacle
positions, smooth turns, or output speed and heading. It rejects unknown cells
and prevents diagonal moves through blocked corners. An empty path clears RViz
when the goal is outside the grid or no route exists at the selected radius.

You can inspect the exact messages with:

```bash
ros2 topic echo /navigation/dynamic_objects
ros2 topic echo /navigation/gps_velocity
ros2 topic info /navigation/occupancy_map
ros2 topic echo /navigation/planned_path --field poses --once
```

To view the map graphically, run `rviz2` in the container, set **Fixed Frame**
to `map`, and add a **Map** display for `/navigation/occupancy_map`. Add an
**Odometry** display for `/navigation/boat_odometry` to see the boat pose. Add a
**Path** display for `/navigation/planned_path` and a **Pose** display for
`/mission/goal`. The route should end near `(-593, 241)`. RViz is not included
in the default image; install `ros-jazzy-rviz2` in the container if needed.

`make vrx-nav` generates a development copy of the VRX
world with no scoring deadline or automatic shutdown. Gazebo keeps running after
waypoint completion until you stop the launch process (Ctrl+C). The standard
`make vrx` command retains VRX's normal task timer.
