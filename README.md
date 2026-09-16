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

## Repo layout

```
float_forward/
├── Dockerfile                 the environment: ROS 2 Jazzy + colcon + rosdep
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
