# Simulation guide

How the VRX boat simulation in this repo works: the vessel model, the simulated
sensors, the ROS 2 interfaces, the coordinate frames, the environment, how it is
launched, and how the same nodes can run against simulated and real sensors.

For setup and the `make` commands, see the [README](README.md).

> **Status of this document.** Everything here was read from the VRX `v3.1.2`
> source and this repo's config, not observed on a running simulation. Confirm
> topic and frame names once with `ros2 topic list` and
> `ros2 run tf2_tools view_frames` and correct this file if they differ.

## Contents

1. [Overview](#1-overview)
2. [Vessel model and dynamics](#2-vessel-model-and-dynamics)
3. [Single-engine configuration and steering](#3-single-engine-configuration-and-steering)
4. [Camera, LiDAR, GNSS and IMU simulation](#4-camera-lidar-gnss-and-imu-simulation)
5. [ROS 2 topics and sensor interfaces](#5-ros-2-topics-and-sensor-interfaces)
6. [TF / coordinate frame structure](#6-tf--coordinate-frame-structure)
7. [Waves, wind and environmental disturbances](#7-waves-wind-and-environmental-disturbances)
8. [How simulations and scenarios are launched](#8-how-simulations-and-scenarios-are-launched)
9. [Same interfaces for simulated and real sensors](#9-same-interfaces-for-simulated-and-real-sensors)
10. [Recreating our real boat in the simulation](#10-recreating-our-real-boat-in-the-simulation)
11. [Known limitations](#11-known-limitations)

## 1. Overview

The simulation is [VRX](https://github.com/osrf/vrx) (Virtual RobotX) running on
Gazebo Harmonic, connected to ROS 2 Jazzy through `ros_gz_bridge`. VRX is cloned
into the Docker image at build time (pinned to `v3.1.2`, under `/ws/src/vrx`) and
is not patched. The only boat-specific file in this repo is
`boat/config/wamv_single_thruster.yaml`.

`make vrx` does two things inside the container:

1. **Generates the boat.** VRX's `generate_wamv` reads the thruster YAML and
   writes `/ws/wamv.urdf`.
2. **Launches the world.** `competition.launch.py` starts Gazebo with the chosen
   world, spawns that URDF, and starts the ROS bridges and TF nodes.

```
wamv_single_thruster.yaml ──generate_wamv──> /ws/wamv.urdf
                                                  │
world .sdf ──> Gazebo (physics, sensors) <──spawn─┘
                     │
               ros_gz_bridge ──> ROS 2 topics under /wamv, /tf, /clock
```

## 2. Vessel model and dynamics

The boat is a WAM-V catamaran simulated as one rigid body, `wamv/base_link`
(180 kg hull, plus batteries, sensors and the engine). Physics runs on the DART
engine with a 4 ms step. Four plugins produce the forces:

| Force | Plugin | What it does |
|---|---|---|
| Buoyancy and waves | `vrx::Surface` (one per hull) | Upward force at two points per hull, from how deep each point sits below the local water surface |
| Water drag | `vrx::SimpleHydrodynamics` | Linear plus quadratic damping in all six axes |
| Thrust | `gz::sim::systems::Thruster` | Applies the commanded force along the propeller axis |
| Engine swivel | `gz::sim::systems::JointPositionController` | Rotates the engine about its vertical axis |

Key parameters (all stock VRX values):

| Parameter | Value |
|---|---|
| Hull mass | 180 kg |
| Hull inertia (ixx, iyy, izz) | 120, 393, 446 kg·m² |
| Hull length / radius | 4.9 m / 0.213 m |
| Hull spacing | y = ±1.03 m |
| Surge drag `xU`, `xUU` | 100, 150 |
| Sway drag `yV`, `yVV` | 100, 100 |
| Yaw drag `nR`, `nRR` | 800, 800 |
| Added mass | 0 |
| Max thrust per engine | 2353 N forward, 1000 N reverse |

## 3. Single-engine configuration and steering

The stock WAM-V has `left` and `right` engines at y = ±1.027 m. Ours has one
engine named `main` at the same x and z but on the centreline:

```yaml
engine:
  - prefix: "main"
    position: "-2.373776 0.0 0.318237"
    orientation: "0.0 0.0 0.0"
```

For each YAML entry the generator emits one engine, one thrust plugin and one
swivel controller. The generator prints a "NOT compliant" error for the
centreline position; that only concerns official VRX competition entries.

**Consequences of one engine:**

- **Top speed is about 3.6 m/s.** At steady speed thrust equals drag:
  2353 = 100·v + 150·v² gives v ≈ 3.6 m/s. The stock twin-engine boat reaches
  about 5.3 m/s by the same sum.
- **No differential thrust.** Steering is done by swivelling the engine, which
  vectors the thrust. The propeller sits about 2.65 m behind `base_link`.
- **Underactuated.** Sideways motion and heading cannot be controlled
  independently, so station keeping is less precise than with two engines.

**Steering commands** (both `std_msgs/msg/Float64`):

```bash
# throttle in newtons: -1000 to +2353
ros2 topic pub -r 10 /wamv/thrusters/main/thrust std_msgs/msg/Float64 "{data: 1000.0}"

# engine angle in radians: -pi to +pi, 0 is straight
ros2 topic pub -r 10 /wamv/thrusters/main/pos std_msgs/msg/Float64 "{data: 0.4}"
```

`pos` is a target angle, not a rotation rate. From the geometry, a positive
angle turns the boat to starboard (right) and a negative angle to port; confirm
the sign on first run. VRX's joystick teleop assumes `left`/`right` engines and
does not work with this layout.

## 4. Camera, LiDAR, GNSS and IMU simulation

The sensors are declared in two files under `boat/config/`:

- `wamv_sensors.yaml`: which sensors the boat carries and where they are
  mounted (VRX's component format).
- `boat_params.yaml`: what each sensor type measures (rate, resolution, field
  of view, range, noise). `scripts/apply_boat_params.py` writes these into the
  generated URDF.

The defaults are VRX's example cameras, LiDAR, GNSS and IMU:

| Sensor | Count and position | Spec | Noise |
|---|---|---|---|
| Camera | 3, forward-facing, pitched down 15° (`front_left_camera`, `front_right_camera`, `far_left_camera`) | 1280×720 RGB, 80° horizontal FOV, 30 Hz | Gaussian per pixel, σ = 0.007 |
| LiDAR | 1 (`lidar_wamv`), 1.8 m high, pitched 8° | 16 beams, ±15° vertical, 360° horizontal, 1875 samples per ring, 10 Hz, 0.1–130 m | Gaussian, σ = 0.01 m |
| GNSS | 1 (`gps_wamv`), at the stern | 20 Hz | None (perfect fix) |
| IMU | 1 (`imu_wamv`) | 100 Hz, orientation in ENU | Gaussian noise plus bias on gyro and accelerometer |

VRX's example set also has an acoustic pinger receiver and a ball shooter; ours
leaves them out.

GNSS position noise can be switched on with `gps.horizontal_noise_stddev` and
`gps.vertical_noise_stddev` in `boat_params.yaml`.

## 5. ROS 2 topics and sensor interfaces

All boat topics are bridged from Gazebo under the `/wamv` namespace.

**Sensors (simulation → ROS):**

| Topic | Type |
|---|---|
| `/wamv/sensors/cameras/<name>_sensor/image_raw` | `sensor_msgs/msg/Image` |
| `/wamv/sensors/cameras/<name>_sensor/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/wamv/sensors/cameras/<name>_sensor/optical/image_raw` | `sensor_msgs/msg/Image` (optical frame) |
| `/wamv/sensors/lidars/lidar_wamv_sensor/points` | `sensor_msgs/msg/PointCloud2` |
| `/wamv/sensors/lidars/lidar_wamv_sensor/scan` | `sensor_msgs/msg/LaserScan` |
| `/wamv/sensors/gps/gps/fix` | `sensor_msgs/msg/NavSatFix` |
| `/wamv/sensors/imu/imu/data` | `sensor_msgs/msg/Imu` |
| `/wamv/joint_states` | `sensor_msgs/msg/JointState` |
| `/clock` | `rosgraph_msgs/msg/Clock` |

**Actuators (ROS → simulation):**

| Topic | Type | Unit |
|---|---|---|
| `/wamv/thrusters/main/thrust` | `std_msgs/msg/Float64` | newtons |
| `/wamv/thrusters/main/pos` | `std_msgs/msg/Float64` | radians |

**Task and environment:**

| Topic | Content |
|---|---|
| `/vrx/task/info` | Task state, timing and score |
| `/vrx/stationkeeping/goal`, `/vrx/stationkeeping/pose_error` | Task-specific goal and error (each task world has its own) |
| `/vrx/debug/wind/speed`, `/vrx/debug/wind/direction` | Current wind (hidden when `competition_mode:=True`) |

There is no ground-truth odometry topic. That needs VRX's `wamv_p3d` component,
which the example sensor set does not include.

## 6. TF / coordinate frame structure

- **World frame:** ENU, origin at latitude −33.724223, longitude 150.679736
  (Sydney International Regatta Centre).
- **Body frame:** `base_link`, x forward, y to port, z up.
- **Boat-internal frames:** `robot_state_publisher` publishes the URDF tree
  (base, engine, propeller, sensor links) using `/wamv/joint_states`.
- **Sensor frames:** the `pose_tf_broadcaster` node republishes Gazebo's sensor
  poses to `/tf`. Each camera also gets an `_optical` frame (z forward, x right,
  y down) from `optical_frame_publisher`.
- **Double prefix:** link names already start with `wamv/` and the publisher
  adds `wamv/` again, so expect frame names like `wamv/wamv/base_link`.
- **No global transform:** nothing publishes `map` or `odom` to `base_link`.
  The navigation stack must produce it from GNSS and IMU, for example with
  `robot_localization`.

## 7. Waves, wind and environmental disturbances

| Disturbance | Model | Default |
|---|---|---|
| Waves | Pierson-Moskowitz spectrum, 3 component waves | On: period 5 s, gain 0.3, direction 0 |
| Wind | Force proportional to relative wind speed squared, plus a yaw torque; optional gusts | Off: mean speed 0, gust gain 0 |
| Current | Not modelled | — |

Waves only change the water height under each hull point. The boat heaves,
pitches and rolls, but waves do not push it sideways.

Wind and waves are set in `boat/config/environment.yaml`:

| Key | Meaning |
|---|---|
| `wind.speed` | Mean wind speed in m/s (0 = no wind) |
| `wind.direction` | Where the wind blows towards, in degrees: 0 = east, 90 = north |
| `wind.gust_strength` | How much the speed varies around the mean, in m/s |
| `wind.gust_time` | How quickly gusts build and fade, in seconds |
| `waves.gain` | Wave height multiplier (0 = flat water) |
| `waves.period` | Time between wave crests, in seconds |
| `waves.direction` | Where the waves travel towards, in degrees |

`make vrx` runs `scripts/apply_environment.py`, which copies the chosen world to
`/ws/worlds/` with these values written in, and launches the copy. Keep several
environment files (calm, windy, rough) and pick one with
`make vrx ENVIRONMENT=<path>`.

## 8. How simulations and scenarios are launched

```bash
make vrx                              # default world: stationkeeping_task
make vrx WORLD=wayfinding_task        # pick a scenario
make vrx HEADLESS=True                # no GUI
make vrx THRUSTER_YAML=<path>         # different engine layout
make vrx SENSOR_YAML=<path>           # different sensor layout
make vrx BOAT_PARAMS=<path>           # different hull, drag, thrust and sensor values
make vrx ENVIRONMENT=<path>           # different wind and waves
```

The target runs `generate_wamv.launch.py`, then `scripts/apply_boat_params.py`
and `scripts/apply_environment.py`, then
`ros2 launch vrx_gz competition.launch.py`. That launch:

1. starts Gazebo with the world;
2. spawns the boat at a fixed start pose;
3. starts `ros_gz_bridge`, `pose_tf_broadcaster`, `optical_frame_publisher`
   and `robot_state_publisher` under the `/wamv` namespace.

Other launch arguments not exposed by the Makefile:

| Argument | Meaning |
|---|---|
| `paused` | Start the simulation paused |
| `sim_mode` | `full` (default), `sim` (no bridges) or `bridge` (bridges only) |
| `competition_mode` | Hide debug topics |
| `extra_gz_args` | Extra arguments for `gz sim` |

**Scenarios.** Each scenario is a world file with a scoring plugin:
`stationkeeping_task`, `wayfinding_task`, `navigation_task`, `follow_path_task`,
`perception_task`, `scan_dock_deliver_task`, `acoustic_perception_task`,
`acoustic_tracking_task`, `gymkhana_task`, `wildlife_task`, and the full
`sydney_regatta` venue. A task runs through *initial*, *ready* and *running*
phases, reported on `/vrx/task/info`.

The `boat` package does not yet have its own VRX launch file; `sim.launch.py`
is only the shapes demo that checks the bridge.

## 9. Same interfaces for simulated and real sensors

Here "GCN" is taken to mean guidance, control and navigation. Nothing in this
section is implemented yet; this is the proposed design.

**Principle:** perception and GCN nodes depend only on a fixed set of topic
names, message types and frame names. What publishes them (Gazebo or hardware)
is decided by the launch file.

| Piece | Simulation | Real boat |
|---|---|---|
| Sensor topics (e.g. `/boat/camera/front/image_raw`, `/boat/lidar/points`, `/boat/gnss/fix`, `/boat/imu/data`) | VRX topics remapped to these names | Hardware drivers publishing on these names |
| Actuator commands (throttle, steering angle) | Adapter node → `/wamv/thrusters/main/thrust` and `/pos` | Adapter node → motor controller |
| Frames | One shared URDF with the real sensor positions | Same URDF |
| Clock | `use_sim_time:=true` | `use_sim_time:=false` |
| Bring-up | `sim` launch file: VRX plus remaps | `real` launch file: drivers |

The message types already match: the simulation publishes the standard
`sensor_msgs` types that real camera, LiDAR, GNSS and IMU drivers use.

Differences to keep in mind:

- The simulated GNSS has no noise, so add noise before trusting a localisation
  filter tuned in simulation.
- Real drivers often publish with best-effort QoS while the bridge publishes
  reliable. Subscribe with sensor-data QoS so nodes work with both.
- Frame names in message headers must match between the two modes.

## 10. Recreating our real boat in the simulation

Pasting the boat's weight is not enough; the simulator needs about five things:

| What | How to get it | Key in `boat/config/boat_params.yaml` |
|---|---|---|
| Mass | Weigh the hull | `hull.mass` |
| Rotational inertia | CAD, or approximate as a box | `hull.inertia` |
| Hull shape | Hull length, radius and spacing | `hull.length`, `hull.radius`, `hull.spacing` |
| Water drag | Water tests (below) | `drag.xU`, `xUU`, `yV`, `yVV`, `nR`, `nRR` |
| Motor thrust | Bollard pull: tie the boat to a dock through a scale, full throttle | `thruster.max_thrust` |
| Engine position | Tape measure from the boat's centre | `position` in `boat/config/wamv_single_thruster.yaml` |

`make vrx` generates the stock WAM-V URDF and then runs
`scripts/apply_boat_params.py`, which writes these values over the stock ones
in `/ws/wamv.urdf`. To change a value, edit the YAML and run `make vrx` again.
Check what the simulator will use with `grep -n "mass value" /ws/wamv.urdf`.

**Drag tests:**

- *Top speed at full throttle.* Thrust equals drag at steady speed, which gives
  one point on the drag curve.
- *Coast-down.* Run at speed, cut the throttle, log GPS speed over time. The
  decay gives the linear and quadratic surge terms.
- *Timed full-lock turn.* Gives a rough value for the yaw terms.

Then tune until the same manoeuvres give the same speeds and turn rates in
simulation.

**Catches:**

- The buoyancy model assumes two cylindrical hulls. A monohull needs the hull
  points rearranged, and roll stability will only be approximate.
- Only the physics values change. The visuals and collision shapes stay those
  of the WAM-V.
- Values not in `boat_params.yaml` (for example the reverse thrust limit) still
  come from VRX.

## 11. Known limitations

- Hull, drag and thrust values are stock WAM-V, not our boat's.
- The engine swivels to the commanded angle almost instantly; a real steering
  servo is much slower.
- Reverse thrust is capped at 1000 N against 2353 N forward.
- No rudder or hull-lift effect: there is no steering authority without thrust.
- The thrust plugin does not check whether the propeller is submerged.
- The engine sits on the centreline where the WAM-V model has no structure
  (cosmetic only).
- GNSS is noiseless unless noise is set in `boat_params.yaml`; there is no water
  current; wind is off until set in `environment.yaml`.
- Rendering is on the CPU inside the container, so camera and LiDAR are slow.
