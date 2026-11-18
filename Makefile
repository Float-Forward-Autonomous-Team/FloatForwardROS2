.PHONY: build up sh colcon vrx foxglove obstacles circles track race live publish tunnel record replay-data down clean

WORLD ?= stationkeeping_task
HEADLESS ?= False
THRUSTER_YAML ?= /ws/src/boat/boat/config/wamv_single_thruster.yaml
SENSOR_YAML ?= /ws/src/boat/boat/config/wamv_sensors.yaml
ENVIRONMENT ?= /ws/src/boat/boat/config/environment.yaml
BOAT_PARAMS ?= /ws/src/boat/boat/config/boat_params.yaml

# `docker compose exec ... bash -lc` runs a login but non-interactive shell,
# and Ubuntu's default ~/.bashrc returns immediately when non-interactive —
# skipping the `source /opt/ros/jazzy/setup.bash` line it normally runs for
# `make sh`. Source explicitly here instead of relying on .bashrc.
ROS_ENV = source /opt/ros/jazzy/setup.bash && { [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash; true; }

build:   ## build the docker image
	docker compose build

# --no-recreate: every target below runs `up` first, and a recreate would kill whatever
# is running in the container (e.g. `make race` killing `make vrx` with error 137).
# so after `make build`, run `make down` once to pick up the new image.
up:      ## start the container in the background (Gazebo GUI stack starts automatically)
	docker compose up -d --no-recreate

sh: up   ## open a shell in the running container
	docker compose exec ros bash

# --merge-install puts every package under one install/share/, as VRX expects:
# the WAM-V URDF's package://wamv_description meshes (hull, engine, propeller)
# only resolve through vrx_gazebo's install/share/ resource path. without --merge-install
# some boat's features don't render
colcon: up  ## rosdep install + colcon build inside the container
	docker compose exec ros bash -lc \
		"$(ROS_ENV) && cd /ws && sudo apt-get update && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install --merge-install"

# generate_wamv writes <yaml name>.xacro next to each yaml, so generate from copies
# in /ws instead of littering the bind-mounted repo. apply_boat_params then writes
# our hull/drag/thrust/sensor values (BOAT_PARAMS) over the stock WAM-V ones in the URDF.
# apply_environment does the same for wind and waves (ENVIRONMENT), into a copy of the world.
vrx: up  ## launch a VRX world with our single-engine WAM-V (needs `make colcon` first); override with WORLD=<name>/HEADLESS=True/THRUSTER_YAML=<path>/SENSOR_YAML=<path>/BOAT_PARAMS=<path>/ENVIRONMENT=<path>
	docker compose exec ros bash -lc "$(ROS_ENV) && \
		cp $(THRUSTER_YAML) /ws/thrusters.yaml && \
		cp $(SENSOR_YAML) /ws/sensors.yaml && \
		ros2 launch vrx_gazebo generate_wamv.launch.py thruster_yaml:=/ws/thrusters.yaml component_yaml:=/ws/sensors.yaml wamv_target:=/ws/wamv.urdf && \
		python3 /ws/src/boat/scripts/apply_boat_params.py $(BOAT_PARAMS) /ws/wamv.urdf && \
		python3 /ws/src/boat/scripts/apply_environment.py $(ENVIRONMENT) $(WORLD) /ws/worlds && \
		ros2 launch vrx_gz competition.launch.py world:=/ws/worlds/$(WORLD) headless:=$(HEADLESS) urdf:=/ws/wamv.urdf"

foxglove: up  ## start foxglove_bridge on ws://localhost:8765 (run alongside `make vrx`)
	docker compose exec ros bash -lc "$(ROS_ENV) && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"

# simulation only: Gazebo's true obstacle positions as markers on /obstacles/markers
obstacles: up  ## publish ground-truth obstacle markers for Foxglove (run alongside `make vrx`)
	docker compose exec ros bash -lc "$(ROS_ENV) && ros2 run boat obstacle_truth"

THRUST ?= 600.0
STEER ?= 0.3

# placeholder race stage: constant thrust (N) and engine angle (rad); also frees the boat from its start platform
circles: up  ## drive the boat in circles (run alongside `make vrx`); override with THRUST=<N>/STEER=<rad>
	docker compose exec ros bash -lc "$(ROS_ENV) && ros2 run boat circle_driver --ros-args -p thrust:=$(THRUST) -p steer:=$(STEER)"

# one terminal for the whole dashboard side: foxglove_bridge + circle_driver + gps_track + obstacle_truth.
# replaces running `make foxglove`, `make circles`, `make track` and `make obstacles` separately.
race: up  ## run a race stage with the Foxglove dashboard (start `make vrx` first); override with THRUST=<N>/STEER=<rad>
	docker compose exec ros bash -lc "$(ROS_ENV) && ros2 launch boat race.launch.py thrust:=$(THRUST) steer:=$(STEER)"

# runs on the Mac, not in the container: the public live page prototype, a static page that reads foxglove_bridge directly
live:    ## serve the public live page on http://localhost:8090 (AI tab needs `make race`)
	python3 -m http.server 8090 --bind 127.0.0.1 --directory live

# runs on the Mac: deploy live/ to Vercel as a static site. The first run asks you to log in
# and to name the project; the name becomes the address, <name>.vercel.app.
publish: ## publish the public live page to Vercel (production)
	@npx --yes vercel whoami >/dev/null 2>&1 || npx --yes vercel login
	npx --yes vercel deploy live --prod

# runs on the Mac: share the public bridge (port 8766, read-only) through a Cloudflare quick tunnel
tunnel:  ## make USV-01 live on the public page for anyone with the printed link (needs `make race`, `brew install cloudflared`)
	./scripts/public_tunnel.sh

RECORD_S ?= 75
# the public page's topics only, so the recording holds nothing the page could not show anyway
PUBLIC_TOPICS = /wamv/sensors/gps/gps/fix /gps_track /wamv/sensors/imu/imu/data /obstacles/markers \
	/detections /detections/inference_ms /public/cameras/front_left/compressed

record: up  ## record RECORD_S seconds (default 75) of what the public page shows, into recordings/ (needs `make race`)
	@# -T and </dev/null: with a terminal attached, ros2 bag record reads the keyboard for its
	@# pause key and gets stopped by the shell, because timeout runs it in the background.
	@# timeout's own exit code 124 is the normal end of the recording, not an error.
	docker compose exec -T ros bash -lc "$(ROS_ENV) && mkdir -p /ws/src/boat/recordings && \
		bag=/ws/src/boat/recordings/usv01_$$(date +%Y%m%d_%H%M%S) && \
		{ timeout -s INT $(RECORD_S) ros2 bag record -s mcap -o \$$bag --topics $(PUBLIC_TOPICS) < /dev/null || [ \$$? -eq 124 ]; } && \
		ros2 bag info \$$bag"

# the newest recording unless BAG=recordings/<name> is given
BAG ?= $(shell ls -d recordings/usv01_* 2>/dev/null | tail -1)
replay-data: up  ## turn a recording into the replay the public page loops (live/data/usv01); BAG=recordings/<name>
	docker compose exec ros bash -lc "$(ROS_ENV) && cd /ws/src/boat && python3 live/tools/build_usv_replay.py $(BAG) live/data/usv01"

track: up  ## publish the boat's GPS track as GeoJSON on /gps_track for Foxglove's Map panel
	docker compose exec ros bash -lc "$(ROS_ENV) && ros2 run boat gps_track"

down:    ## stop the container (build/install/log volumes kept)
	docker compose down

clean:   ## stop the container and wipe build/install/log volumes
	docker compose down -v
