.PHONY: build up sh colcon sim down clean

build:   ## build the docker image
	docker compose build

up:      ## start the container in the background (Gazebo GUI stack starts automatically)
	docker compose up -d

sh: up   ## open a shell in the running container
	docker compose exec ros bash

colcon: up  ## rosdep install + colcon build inside the container
	docker compose exec ros bash -lc \
		"cd /ws && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"

sim: up  ## start the container and print how to reach the Gazebo GUI
	@echo "Gazebo GUI stack is starting in the background."
	@echo "Open http://localhost:6080/vnc.html?resize=scale in a browser and click Connect (no password)."
	@echo "Then run: make sh   and inside the shell: ros2 launch boat sim.launch.py"

down:    ## stop the container (build/install/log volumes kept)
	docker compose down

clean:   ## stop the container and wipe build/install/log volumes
	docker compose down -v
