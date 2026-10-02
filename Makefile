.PHONY: build up sh colcon vrx down clean

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

up:      ## start the container in the background (Gazebo GUI stack starts automatically)
	docker compose up -d

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

down:    ## stop the container (build/install/log volumes kept)
	docker compose down

clean:   ## stop the container and wipe build/install/log volumes
	docker compose down -v
