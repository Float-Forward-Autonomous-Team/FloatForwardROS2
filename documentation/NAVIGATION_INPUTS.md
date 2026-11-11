# Navigation demo

This demo runs the Guidance & Navigation chain in simulation: a world model, a
global A* planner, path smoothing with line-of-sight guidance, and a PID
controller that **drives the boat** to the demo goal. The default setup targets
VRX's `wayfinding_task` world.

## Run

Build the `boat_interfaces` and `boat` packages with your normal colcon
workflow. Start the world in one terminal:

```bash
make vrx-nav WORLD=wayfinding_task
```

After Gazebo has loaded, open another terminal:

```bash
make sh
ros2 launch boat navigation_demo.launch.py
```

The launch starts nine small processes:

- `gazebo_obstacles` spawns visible, collidable spheres and moves them along
  trajectories in `boat/config/moving_obstacles.yaml`.
- `sim_truth` stands in for Localization and Perception: it reads Gazebo poses
  and GNSS and publishes them on the topics those modules will use.
- `world_model` builds the static map and splits Perception objects into
  stopped (static) and moving ones.
- `navigation_monitor` subscribes and prints a one-second input summary.
- `mission_manager` publishes the demo goal at `(-593, 241)` in the `map` frame
  and moves to the next task when the global planner reports it reached.
- `global_planner` publishes `/planning/global_path` (`nav_msgs/msg/Path`),
  replanning only when something relevant changes.
- `local_planner` is a placeholder: it forwards the global path to guidance and
  guidance's status to the global planner, unchanged.
- `local_spline_guidance` smooths the local path and publishes a lookahead
  target point and a target speed once a second; it stops the boat at the end
  of the path.
- `pid_controller` (Control) turns the target point and speed into engine angle
  and thrust commands for the single WAM-V engine.

## Data flow

```
Gazebo ──► sim_truth ──► /localization/odometry ───────────────────────┐
                    └──► /perception/objects ──► world_model            │
world SDF (prior) ─────────────────────────────► world_model            │
                                                   │                    ▼
                       /world_model/static_map ────┴──► global_planner, local_spline_guidance
                       /world_model/moving_objects  (local planner, later)
                       /world_model/landmarks       (task planner, later)

 commands go down                                status goes up
 ────────────────                                ──────────────
 mission_manager
   │ /mission/goal                               ▲ /planning/global_status
 global_planner
   │ /planning/global_path                       ▲ /planning/local_status
 local_planner (placeholder: forwards both ways)
   │ /planning/local_path                        ▲ /guidance/status
 local_spline_guidance
   │ /guidance/target_position, /guidance/target_velocity
 pid_controller ──► /wamv/thrusters/main/{pos,thrust}
```

Planners read only Localization, world model and mission topics. Replacing
`sim_truth` with real Localization and Perception therefore needs no planner
change, as long as the replacements publish the same messages.

## Code layout

The Python code lives in `boat/boat/navigation/`, grouped by responsibility:

| Location | Responsibility |
| --- | --- |
| `mission_manager.py` | Publish one mission goal at a time and advance on task feedback |
| `navigation_monitor.py` | Log navigation inputs for diagnostics; no control output |
| `world_model/world_model.py` | Prior map + Perception objects → static map, moving objects, landmarks |
| `world_model/layers.py` | Paint obstacles into grids; clearance (distance) field shared by planners |
| `world_model/objects.py` | Constant-velocity prediction; stopped-object promotion rule |
| `status.py` | Shared status message helpers: latched QoS, publish-on-change, goal matching |
| `planning/global_planner.py` | Subscribe to the static map, boat pose and goal; publish a path |
| `planning/local_planner.py` | Placeholder that forwards the path down and the status up |
| `planning/grid_astar.py` | Step costs from clearance; eight-connected A* |
| `planning/path_geometry.py` | Distance from a point to a path |
| `planning/path_smoothing.py` | Shortcut simplification, Catmull-Rom spline, lookahead target |
| `planning/local_spline_guidance.py` | Smooth the route; publish target point, speed and stop |
| `simulation/sim_truth.py` | Publish perfect boat state, GNSS and object tracks |
| `simulation/scenario_map.py` | Rasterize Gazebo collision geometry; list top-level models |
| `simulation/gazebo_obstacles.py` | Spawn and move physical Gazebo models |
| `simulation/obstacle_trajectory.py` | Parse and evaluate configured obstacle paths |

The PID controller lives outside `navigation/`, in `boat/boat/pid_controller.py`,
because it belongs to Control. The ROS messages remain in `boat_interfaces/msg/`,
the launch description in
`boat/launch/`, and the trajectory YAML in `boat/config/`. The shared
`scripts/apply_environment.py` helper copies a VRX world before launch. For the
navigation targets, its `--keep-running` option gives each VRX scoring plugin
an unlimited running duration and exit delay, and disables automatic shutdown.
Standard competition targets omit that option.

## Status chain

Each layer knows only the layer directly above and below it. Commands go down
(goal → global path → local path → target), and progress reports go up as
`boat_interfaces/msg/NavigationStatus`:

| State | Meaning |
|---|---|
| `IDLE` | Nothing to do yet: no goal or no path |
| `ACTIVE` | Working towards the goal |
| `SUCCEEDED` | The goal was reached |
| `BLOCKED` | Cannot make progress right now but still trying; `detail` says why |

Every report carries the goal it refers to (`goal`), so a layer ignores reports
about a goal it no longer works on. The status topics are latched and published
only when the state, goal or detail changes.

1. **Guidance** knows only its path. Within 1 m of the path's end it stops the
   boat and reports `SUCCEEDED` with the path's last point on `/guidance/status`.
2. **Local planner** (placeholder) forwards that report unchanged on
   `/planning/local_status`. The real local planner will report `SUCCEEDED`
   only when the end of its short path is also the end of the global path.
3. **Global planner** accepts `SUCCEEDED` only if it names the end of its
   current path, and that path was planned for the current mission goal. It
   then reports the mission goal reached on `/planning/global_status`, and
   keeps reporting it until the goal changes.
4. **Mission manager** advances to the next task when that report names the
   current task's goal. `BLOCKED` is only logged; the task timeout
   (`task_timeout_sec`) decides when to give up. When the last task is done
   the mission state becomes `COMPLETE`.

The mission manager also publishes `/mission/state` (`String`), for example
`RUNNING 1/1` or `COMPLETE`. The demo launch gives each task a one-hour
simulation-time timeout.

If the packages have not been built in the running container, use your normal
`make colcon` workflow first.

## Topics

| Topic | Type | Publisher | Meaning |
|---|---|---|---|
| `/localization/odometry` | `nav_msgs/msg/Odometry` | `sim_truth` | Exact Gazebo boat pose; planar body-frame velocity from successive poses |
| `/localization/gps_fix` | `sensor_msgs/msg/NavSatFix` | `sim_truth` | Gazebo GNSS latitude, longitude, altitude |
| `/localization/gps_velocity` | `geometry_msgs/msg/Vector3Stamped` | `sim_truth` | Gazebo GNSS east, north, up velocity in m/s |
| `/perception/objects` | `boat_interfaces/msg/TrackedObjectArray` | `sim_truth` | Every scripted obstacle: ID, class, pose, radius, estimated velocity |
| `/world_model/static_map` | `nav_msgs/msg/OccupancyGrid` | `world_model` | Prior map plus stopped objects; latched, published only on change |
| `/world_model/moving_objects` | `boat_interfaces/msg/TrackedObjectArray` | `world_model` | Objects that are not in the static map |
| `/world_model/landmarks` | `boat_interfaces/msg/TrackedObjectArray` | `world_model` | Top-level world models (buoys, posts, …); latched |
| `/mission/goal` | `geometry_msgs/msg/PoseStamped` | `mission_manager` | Current demo target in the `map` frame |
| `/mission/state` | `std_msgs/msg/String` | `mission_manager` | `RUNNING i/n`, `COMPLETE` or `ABORTED` |
| `/planning/global_path` | `nav_msgs/msg/Path` | `global_planner` | Latest route from the boat to the goal; empty when there is none |
| `/planning/global_status` | `boat_interfaces/msg/NavigationStatus` | `global_planner` | Progress towards the mission goal; latched |
| `/planning/local_path` | `nav_msgs/msg/Path` | `local_planner` | Path for guidance to follow (for now the global path) |
| `/planning/local_status` | `boat_interfaces/msg/NavigationStatus` | `local_planner` | Progress along the global path (for now guidance's status) |
| `/guidance/status` | `boat_interfaces/msg/NavigationStatus` | `local_spline_guidance` | Following, blocked, or at the end of the local path; latched |
| `/planning/smoothed_path` | `nav_msgs/msg/Path` | `local_spline_guidance` | Smoothed route with path headings, for visualization |
| `/guidance/target_position` | `geometry_msgs/msg/PoseArray` | `local_spline_guidance` | One lookahead target in `map`; empty means stop |
| `/guidance/target_velocity` | `std_msgs/msg/Float64` | `local_spline_guidance` | Target forward speed in m/s |
| `/wamv/thrusters/main/pos` | `std_msgs/msg/Float64` | `pid_controller` | Engine angle in radians (±45°) |
| `/wamv/thrusters/main/thrust` | `std_msgs/msg/Float64` | `pid_controller` | Engine thrust in newtons |

Everything uses Gazebo's local east-north-up `map` frame in metres.

## World model

### Static map

The static map has two layers. The **prior** layer is loaded once at startup.
In simulation it comes from `prior_world_sdf`: the **world SDF and the
installed or cached model collision geometry**, including the Sydney shoreline
collision mesh, buoy cylinders and other props. Water and seabed planes are
excluded. Within the grid, areas with no modeled collision are treated as free;
this is not a depth or navigability guarantee. With an empty `prior_world_sdf`
(for example on the real boat until a chart is available) the prior is all free.

The **detected static** layer holds Perception objects that have stopped. An
object is promoted when its speed stays at or below `stationary_speed_mps`
(default 0.2 m/s) for `stationary_time_s` (default 5 s) of consecutive
snapshots. It is drawn into the grid as a circle of its radius and removed from
`/world_model/moving_objects`. It is demoted when it reports a speed above the
threshold or moves farther than its radius from where it stopped. A promoted
object that disappears from the snapshots stays in the map, since a stopped
obstacle outside Perception's view is still there.

The map is published latched and only when a layer changes, so subscribers
started later still receive it. Cells are `0` (free) or `100` (occupied). The
default grid spans x `[-650, -300)` and y `[-50, 350)` at 0.5 m per cell; the
bounds are `world_model` parameters. Cells outside this bounded map have no
representation.

### Moving objects and prediction

Moving objects are never drawn into the grid. Consumers predict them with
`world_model.objects.predict`, a constant-velocity extrapolation from the
snapshot time, so every planner predicts the same way.

### Landmarks

Each top-level model of the prior world that has collision geometry is
published as a landmark: `id` is the model's name in the world, `classification`
is the model type (the last part of its URI), and `radius_m` is the largest
distance from the model origin to its footprint. The list includes the whole
venue (`sydney_regatta`, with a very large radius); consumers select landmarks
by classification.

## Global planner

The planner ignores moving objects; they belong to the local planner. It uses
the static map only.

When a map arrives, the planner computes a **clearance field**: each cell's
distance to the nearest occupied or unknown cell, or to the map edge
(`world_model.layers.clearance_cells`). The step cost of entering a cell is the
step length times a multiplier:

- infinite when the clearance is below `inflation_radius_m` (default 3.0 m),
  so the route keeps at least that distance from obstacles and the map edge;
- rising linearly from 1 at `safe_distance_m` (default 10 m) to
  `1 + proximity_weight` (default weight 2.0) at zero clearance, so routes keep
  to the middle of channels when there is room.

The inflation radius is a starting value for the simulated WAM-V footprint; it
is not a verified safety margin for the real boat.

The planner checks its triggers once a second and plans only when one holds:

| Trigger | Meaning |
|---|---|
| `no previous plan` | First plan, or inputs became valid again |
| `new goal` | The goal moved more than 0.5 m |
| `map updated` | A new static map arrived (for example a stopped object) |
| `parameters changed` | A cost parameter was changed at runtime |
| `boat off path` | The boat is farther than `replan_deviation_m` (default 10 m) from the path |

Each plan is logged with its trigger and search time. The path is withdrawn
(an empty path is published) when the map, pose and goal frames differ, the
boat or goal is outside the map, or the boat or goal is inside the inflated
region. When no route exists, the planner publishes an empty path and tries
again only after a trigger.

## Guidance and control

Once a second, `local_spline_guidance` takes the latest local path, removes
points that a straight, collision-checked shortcut can skip, and draws a
Catmull-Rom spline through the rest. Its collision check uses the global
planner's rule (`passable_cells` in `world_model/layers.py`): every sample must
be at least `inflation_radius_m` from the nearest obstacle in the clearance
field. If the spline fails the check it falls back to the straight shortcuts,
and those fall back to the planner's own path, which always passes. The demo
launch sets `inflation_radius_m` (default 3 m) for both nodes from one launch
argument; set it with `inflation_radius_m:=<metres>` on the launch command. It then picks the point
`lookahead_distance_m` (default 5 m) ahead of the boat's projection on the
smoothed route and publishes it with `target_speed_mps` (default 1 m/s),
reduced linearly within `slowdown_distance_m` (5 m) of the goal. Within
`goal_tolerance_m` (1 m) of the goal it publishes an empty target and speed 0.
Any missing or invalid input also produces an empty target, which stops the
boat.

`pid_controller` steers the engine towards the target point (heading PID,
engine angle limited to ±45°) and sets thrust from the speed error (speed PID,
−1000 to 2300 N). With a target in the `map` frame it uses
`/localization/odometry`; an empty target sets angle and thrust to zero.

The smoothed path stays outside the planner's inflated region, but the boat
does not follow it exactly: line-of-sight steering cuts corners and waves push
the boat sideways. If the boat itself enters the inflated region, the planner
cannot start a search there; it withdraws the path, guidance stops the boat,
and planning resumes only once the boat drifts clear.

Change parameters while running, for example:

```bash
ros2 param set /global_planner inflation_radius_m 2.5
ros2 param set /global_planner proximity_weight 0.0
```

The planner itself does not smooth turns; `local_spline_guidance` does. It rejects
unknown cells and prevents diagonal moves through blocked corners.

## Scripted obstacles

The scripted obstacles are physical Gazebo models moved through Gazebo's pose
service, so `sim_truth` sees them in the same pose stream as the boat. They are
fixed-height colliders without boat dynamics. Their velocity is estimated
between successive published positions. Edit `moving_obstacles.yaml` and
restart the launch to change their radius, speed, and XY waypoint path. An
empty `obstacles_file` launch argument disables them.

The default file has two obstacles. `moving_obstacle_1` loops round a square
and stays a moving object. `stopping_obstacle_1` drives onto the demo route,
stops after about 30 s, and is added to the static map 5 s later. The global
planner then logs `map updated` and routes around it.

## Inspect

```bash
ros2 topic echo /perception/objects
ros2 topic echo /world_model/moving_objects
ros2 topic echo /world_model/landmarks --once
ros2 topic info /world_model/static_map
ros2 topic echo /planning/global_path --field poses --once
```

To view the map graphically, run `rviz2` in the container, set **Fixed Frame**
to `map`, and add a **Map** display for `/world_model/static_map` with
durability **Transient Local**. Add an **Odometry** display for
`/localization/odometry` to see the boat pose. Add a **Path** display for
`/planning/global_path` and a **Pose** display for `/mission/goal`. The route
should end near `(-593, 241)`. RViz is not included in the default image;
install `ros-jazzy-rviz2` in the container if needed.

`make vrx-nav` generates a development copy of the VRX
world with no scoring deadline or automatic shutdown. Gazebo keeps running after
waypoint completion until you stop the launch process (Ctrl+C). The standard
`make vrx` command retains VRX's normal task timer.
