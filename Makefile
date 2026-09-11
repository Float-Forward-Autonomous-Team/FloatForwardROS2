.PHONY: build up sh colcon down clean

build:   ## build the docker image
	docker compose build

up:      ## start the container in the background
	docker compose up -d

sh: up   ## open a shell in the running container
	docker compose exec ros bash

colcon: up  ## rosdep install + colcon build inside the container
	docker compose exec ros bash -lc \
		"cd /ws && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"

down:    ## stop the container (build/install/log volumes kept)
	docker compose down

clean:   ## stop the container and wipe build/install/log volumes
	docker compose down -v
