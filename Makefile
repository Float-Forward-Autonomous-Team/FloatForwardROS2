.PHONY: build up sh colcon sim vrx down clean

WORLD ?= stationkeeping_task
HEADLESS ?= False
THRUSTER_YAML ?= /ws/src/boat/boat/config/wamv_single_thruster.yaml

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

colcon: up  ## rosdep install + colcon build inside the container
	docker compose exec ros bash -lc \
		"$(ROS_ENV) && cd /ws && sudo apt-get update && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"

sim: up  ## start the container and print how to reach the Gazebo GUI
	@echo "Gazebo GUI stack is starting in the background."
	@echo "Open http://localhost:6080/vnc.html?resize=scale in a browser and click Connect (no password)."
	@echo "Then run: make sh   and inside the shell: ros2 launch boat sim.launch.py"

# generate_wamv writes <yaml name>.xacro next to the yaml, so generate from a copy
# in /ws instead of littering the bind-mounted repo.
vrx: up  ## launch a VRX world with our single-engine WAM-V (needs `make colcon` first); override with WORLD=<name>/HEADLESS=True/THRUSTER_YAML=<path>
	docker compose exec ros bash -lc "$(ROS_ENV) && \
		cp $(THRUSTER_YAML) /ws/thrusters.yaml && \
		ros2 launch vrx_gazebo generate_wamv.launch.py thruster_yaml:=/ws/thrusters.yaml wamv_target:=/ws/wamv.urdf && \
		ros2 launch vrx_gz competition.launch.py world:=$(WORLD) headless:=$(HEADLESS) urdf:=/ws/wamv.urdf"

down:    ## stop the container (build/install/log volumes kept)
	docker compose down

clean:   ## stop the container and wipe build/install/log volumes
	docker compose down -v
