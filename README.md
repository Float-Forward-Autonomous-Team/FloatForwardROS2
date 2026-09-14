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
git clone <this repo>
cd float_forward

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
ROS 2 talk to each other. For ROS 2 Jazzy, Gazebo Harmonic + the bridge
install as a single apt package, `ros-jazzy-ros-gz` — already baked into
this repo's Docker image, nothing extra to install yourself.

Since not everyone on the team is on the same OS (and there's no GPU
passthrough into Docker anyway), Gazebo's GUI runs *inside* the container
against a virtual display and streams to your browser over
[noVNC](https://novnc.com/) — no host setup beyond Docker and a browser.

**Core concepts** (if you're new to Gazebo):

- **World** — an SDF file describing the environment: ground plane,
  lighting, physics settings, and what's placed in it. No boat model exists
  yet, so we launch Gazebo's own stock `shapes.sdf` demo world (a box,
  sphere, and cylinder) just to prove the pipeline works end to end.
- **Model / entity** — a robot, obstacle, or object placed in a world, made
  of links/joints/collision/visual geometry. The boat will eventually be
  one of these (a URDF/SDF/xacro file that doesn't exist in this repo yet).
- **Plugin** — a shared library Gazebo loads (globally, or attached to a
  model/sensor) that adds behavior — e.g. applying thruster forces or
  simulating a GPS sensor — usually by publishing simulated state as
  Gazebo Transport topics.
- **The `ros_gz_bridge`** — a node (`parameter_bridge`) that maps one
  Gazebo topic+type to one ROS 2 topic+type, one line of config per topic.
  `boat/launch/sim.launch.py` bridges `/clock` as the minimal example.
- **Topics** — once bridged, `ros2 topic echo /clock` / `ros2 topic list`
  behave exactly like any other ROS topic, even though the data originates
  inside Gazebo.

**Run it:**

```bash
make build     # only needed once, or after editing Dockerfile
make sim       # starts the container; prints the noVNC URL
```

1. Open `http://localhost:6080/vnc.html` in a browser and click **Connect**
   (no password). You should see a plain desktop within a few seconds — if
   it's blank, reload once (the display server can take a moment to start).
2. `make sh` — open a shell in the container.
3. `ros2 launch boat sim.launch.py` — Gazebo's GUI should appear in the
   noVNC tab showing a box, sphere, and cylinder on a ground plane.
4. In a second `make sh` shell: `ros2 topic echo /clock` (incrementing sim
   time) and `ros2 topic list` (includes `/clock`) confirm the bridge.

**Gotchas:**

- Rendering is software-only (no GPU passthrough), so it's slower than
  native — especially on Apple Silicon.
- The noVNC port is published as loopback-only (`127.0.0.1:6080`) because
  the VNC server has no password. Don't widen that binding to share it
  over a network.
- `shapes.sdf` uses built-in geometry, so it needs no network access. A
  future real boat model that references [Fuel](https://app.gazebosim.org/)
  assets will need internet access on first launch (cached after that).
- Added `ros_gz_sim`/`ros_gz_bridge` as `package.xml` dependencies — like
  any dependency change, `make colcon` (which runs `rosdep install`) picks
  them up automatically.

## Repo layout

```
float_forward/
├── Dockerfile                 the environment: ROS 2 Jazzy + colcon + rosdep + Gazebo + noVNC
├── docker/                    supervisord.conf + entrypoint.sh for the in-container GUI stack
├── compose.yml
├── Makefile
├── pyproject.toml
├── .pre-commit-config.yaml
├── .github/workflows/ci.yml
└── boat/                      ROS 2 package (ament_python)
    ├── package.xml            dependencies + build type
    ├── setup.py  setup.cfg    entry points, data files
    ├── resource/boat          ament index marker (empty, required)
    ├── boat/
    │   ├── __init__.py
    │   └── heartbeat.py       example node -> `ros2 run boat heartbeat`
    ├── launch/boat.launch.py  example launch
    ├── launch/sim.launch.py   Gazebo demo world + ros_gz bridge -> `ros2 launch boat sim.launch.py`
    └── config/params.yaml     example parameters
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
| C++ source                         | `colcon build --packages-select boat`          |
| `package.xml` dependency           | `rosdep install --from-paths src --ignore-src -r -y`, then rebuild |
| `Dockerfile`                       | `make build`                                   |

`package.xml` list the dependencies. The dependencies are resolved by [`rosdep`](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Rosdep.html) (rosdep is similar to `apt` or `pip` but for ROS packages). If you add a dependency, you must run `rosdep install` before rebuilding.

If a deleted node still runs or a renamed package still shows up, the build is
stale: `rm -rf build install log && colcon build --symlink-install`.

## CI

Every pull request runs the same two things: `pre-commit run --all-files`, and
`make build` + `make colcon` — so a green PR means it builds and passes lint.
# FloatForwardROS2
