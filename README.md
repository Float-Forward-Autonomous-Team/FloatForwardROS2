# Float Forward ROS 2

ROS 2 (Jazzy) FF project. The whole dev environment runs in Docker — nobody
needs ROS installed on their own machine, just Docker and `make`.

## Prerequisites, must do before anything else

- **Docker Desktop** — running, before anything else. ([mac](https://www.docker.com/products/docker-desktop/) / [windows](https://www.docker.com/products/docker-desktop/) / [linux](https://docs.docker.com/desktop/setup/install/linux/))
- **`make`**
  - macOS: already installed
  - Linux: `sudo apt install make`
  - Windows: `winget install ezwinports.make`
- **`pre-commit`** — `pip install pre-commit` (or `brew install pre-commit` / `pipx install pre-commit`), then `pre-commit install` once inside the repo to wire it into git

## Quick start

```bash
git clone https://github.com/Float-Forward-Autonomous-Team/FloatForwardROS2.git
cd FloatForwardROS2

make build     # build the docker image (first time, or after editing Dockerfile)
make colcon    # start the container + rosdep install + colcon build
make sh        # open a terminal inside the container
```

Inside the shell (ROS + the built package are already sourced automatically):

```bash
ros2 run boat heartbeat          # or: ros2 launch boat boat.launch.py
```

You should see `alive 0`, `alive 1`, … printed once a second.

## Simulation (Gazebo)

[Gazebo Harmonic](https://gazebosim.org/) is the physics/sensor simulator we
use. It's a separate program from ROS 2, with its own pub/sub transport
(Gazebo Transport) and its own message types (`gz.msgs.*`). `ros_gz` is the
glue package family (`ros_gz_sim`, `ros_gz_bridge`, …) that lets Gazebo and
ROS 2 talk to each other.

Gazebo's GUI runs *inside* the container
against a virtual display and streams to your browser over
[noVNC](https://novnc.com/).

### VRX (boat simulation)

[VRX](https://github.com/osrf/vrx) is OSRF's Virtual RobotX simulator: it adds a
WAM-V catamaran model (thrusters, buoyancy/wave physics, a sensor suite) and a set
of ready-made task worlds. It's cloned into the Docker image at `/ws/src/vrx`
(pinned to `v3.1.2`) and builds alongside `boat` via the normal colcon step.

**Run it:**

```bash
make build     # clones vrx into the image — only needed once, or after editing Dockerfile
make colcon    # builds vrx_gz / vrx_ros / vrx_urdf alongside boat
make vrx       # starts the container and launches a VRX world
```

1. Open `http://localhost:6080/vnc.html?resize=scale` in a browser and click **Connect**
   (no password). If it's blank, reload once (the display server can take a moment
   to start). The WAM-V should appear on the water.
2. In a `make sh` shell, `ros2 topic list` shows the boat's `/wamv/...` topics.

Pick the world with `WORLD=<name>` (default: `stationkeeping_task`, a single light
task world — good for day-to-day dev):

```bash
make vrx WORLD=wayfinding_task
```

Other task worlds: `navigation_task`, `follow_path_task`, `scan_dock_deliver_task`,
`perception_task`, `acoustic_perception_task`, `acoustic_tracking_task`,
`gymkhana_task`, `wildlife_task`.

Things to know:
- Our boat has one engine, `main` (the stock WAM-V has `left` and `right`), placed by
  `boat/config/wamv_single_thruster.yaml`. Its topics are `/wamv/thrusters/main/thrust`
  and `/wamv/thrusters/main/pos`.
- One engine gives half the stock boat's total thrust (2353 N instead of 4707 N), so
  top speed is about 3.6 m/s instead of 5.3 m/s, and it accelerates more slowly.
- One engine can't steer with differential thrust: steer by rotating the engine
  via `.../thrusters/main/pos` (it takes a target angle in radians).
- VRX's joystick teleop (`usv_joy_teleop.py`) assumes `left`/`right` and won't work.
- Gazebo closes itself when the task's timer runs out: about 5 min 20 s of sim time
  for `stationkeeping_task`. Run `make vrx` again to restart.

See [SIMULATION.md](documentation/SIMULATION.md) for the full guide: vessel dynamics, sensors,
topics, TF frames, waves and wind, launching, and running the same nodes against
simulated and real sensors.
[ROS_CHEATSHEET.md](documentation/ROS_CHEATSHEET.md) is the quick reference: every topic, message
type and node.

### Foxglove (viewing the boat's sensors)

[Foxglove](https://foxglove.dev/) shows live ROS topics: camera images, the lidar point
cloud, the GPS position and plots. It runs in your browser, with nothing to install, and
connects to `foxglove_bridge`, which runs inside the container and streams the topics over a
WebSocket on port 8765.

**First time only.** The bridge is installed in the Docker image, so rebuild the image and
the workspace:

```bash
make build     # installs foxglove_bridge and foxglove_msgs
make down      # the next `make` command then starts a container from the new image
make colcon    # registers the new nodes
```

If `make colcon` complains about `--merge-install`, your build volumes come from an older
build. Run `make clean`, then `make colcon` again.

**Every time.** Use two terminals:

```bash
make vrx WORLD=sydney_regatta   # terminal 1: the simulator; wait until the boat has spawned
make race                       # terminal 2: foxglove_bridge plus the nodes the dashboard uses
```

`sydney_regatta` has no task, so the boat stays pinned to its start platform until `make race`
releases it. Start `make race` soon after the boat appears, because the release only happens
during the driver's first few seconds.

**Warning:** `make race` also runs `circle_driver`, which frees the boat and drives it in
circles. If you want to look at the sensors with the boat left alone, run `make foxglove` in
terminal 2 instead. It starts only the bridge. Add `make obstacles` in a third terminal to get
the ground-truth markers.

**Connect Foxglove:**

1. Open [app.foxglove.dev](https://app.foxglove.dev) in your browser and sign in if it asks you
   to. If your browser won't connect, use the
   [desktop app](https://foxglove.dev/download) instead; the steps are the same.
2. Choose **Open connection**, then **Foxglove WebSocket**, and enter `ws://localhost:8765`.
3. Import the team layout from the **Layouts** menu: **Import from file**, then pick
   `foxglove/dashboard_layout.json` from this repo. You only need to do this once.

The layout has four tabs:

| Tab           | What it shows                                                         |
|---------------|-----------------------------------------------------------------------|
| `Race`        | Front left camera, 3D view, GPS map, engine thrust and angle          |
| `Camera`      | Front left and front right cameras side by side                       |
| `LiDAR`       | Point cloud with obstacle markers, far left camera                    |
| `Boat status` | GPS map, VRX task info, plots of thrust, IMU and wind                 |

**For perception work.** Use these topics:

| Topic                                                                  | Type                         |
|------------------------------------------------------------------------|------------------------------|
| `/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw`     | `sensor_msgs/msg/Image`      |
| `/wamv/sensors/cameras/front_right_camera_sensor/optical/image_raw`    | `sensor_msgs/msg/Image`      |
| `/wamv/sensors/cameras/far_left_camera_sensor/optical/image_raw`       | `sensor_msgs/msg/Image`      |
| `/wamv/sensors/lidars/lidar_wamv_sensor/points`                        | `sensor_msgs/msg/PointCloud2`|
| `/obstacles/markers` (simulation only: Gazebo's true obstacle positions) | `visualization_msgs/msg/MarkerArray` |

`/obstacles/markers` is ground truth, so you can compare your detections against it. It only
exists in simulation. Any topic your own node publishes shows up in Foxglove automatically,
so you can add it to a panel without changing the bridge.

**The other commands:**

```bash
make foxglove   # only foxglove_bridge
make obstacles  # only the ground-truth obstacle markers
make circles    # only the circle driver; change it with THRUST=<newtons> STEER=<radians>
make track      # only the GPS track, as GeoJSON on /gps_track for a Map panel
make race THRUST=800 STEER=0.2   # all of it, with a different circle
```

**If something doesn't work:**

- **`port is already allocated`:** another container is using port 6080 or 8765, often a second
  copy of this repo. Find it with `docker ps` and stop it first.
- **Foxglove can't connect:** make sure `make race` or `make foxglove` is still running. Only
  one bridge can use the port, so don't run both at once.
- **`Package 'foxglove_bridge' not found`:** the container uses the old image. Run the
  first-time steps above.
- **`foxglove_msgs` or `obstacle_truth` errors after switching branches:** run `make colcon`.

## Repo layout

```
FloatForwardROS2/
├── README.md
├── Dockerfile                 the environment: ROS 2 Jazzy + colcon + rosdep + Gazebo + noVNC
├── docker/                    supervisord.conf + entrypoint.sh for the in-container GUI stack
├── compose.yml
├── Makefile
├── pyproject.toml
├── .pre-commit-config.yaml
├── .gitignore  .dockerignore
├── .github/workflows/test-pipeline.yml   CI: pre-commit + docker build + colcon build
├── documentation/
│   ├── SIMULATION.md          how the VRX simulation works
│   └── ROS_CHEATSHEET.md      every topic, message type and node
├── foxglove/                  dashboard_layout.json, the team's Foxglove layout
├── scripts/                   helper scripts run by the Makefile (apply_boat_params.py, apply_environment.py)
└── boat/                      ROS 2 package (ament_python)
    ├── package.xml            dependencies + build type
    ├── setup.py  setup.cfg    entry points, data files
    ├── resource/boat          ament index marker (empty, required)
    ├── boat/
    │   ├── __init__.py
    │   └── heartbeat.py       example node -> `ros2 run boat heartbeat`
    ├── launch/boat.launch.py  example launch
    └── config/
        ├── params.yaml                  example parameters
        ├── wamv_single_thruster.yaml    engine position
        ├── wamv_sensors.yaml            which sensors, and where they are mounted
        ├── boat_params.yaml             hull, drag, thrust and sensor values
        └── environment.yaml             wind and waves
```

## Add a node

1. Copy `boat/boat/heartbeat.py` as a starting point for `boat/boat/<new_node_name>.py`.
2. Register it in `boat/setup.py`, under `entry_points -> console_scripts`:
   ```python
   "<name> = boat.<name>:main",
   ```
3. `make colcon`
4. `make sh`, then `ros2 run boat <name>`

## Build & test

```bash
make build     # docker compose build — first time, or after editing Dockerfile
make colcon    # rosdep install + colcon build
make sh        # shell into the container
make down      # stop (build/install/log volumes kept)
make clean     # stop AND wipe those volumes — full rebuild
```

What to do after a change:

| You changed                       | What to do                                    |
|------------------------------------|------------------------------------------------|
| Python node / launch file / YAML   | nothing — `ros2 run` again                     |
| new node, entry_point, or package  | `make colcon`                                  |
| `package.xml` dependency           | `rosdep install --from-paths src --ignore-src -r -y`, then rebuild |
| `Dockerfile`                       | `make build`                                   |

`package.xml` lists the dependencies. The dependencies are resolved by [`rosdep`](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Rosdep.html) (rosdep is similar to `apt` or `pip` but for ROS packages). If you add a dependency, you must run `rosdep install` before rebuilding.

If a deleted node still runs or a renamed package still shows up, the build is
stale: `rm -rf build install log && colcon build --symlink-install --merge-install`.
Always pass `--merge-install`: VRX needs it to find the WAM-V meshes, and colcon
refuses to mix it with an install built without it (fix that with `make clean`).

## CI

Every pull request runs the same two things: `pre-commit run --all-files`, and
`make build` + `make colcon` — so a green PR means it builds and passes lint.
