FROM ros:jazzy-ros-base

# build tooling the ros-base image doesn't ship (colcon, rosdep)
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ws/src && chown -R ubuntu:ubuntu /ws

USER ubuntu
WORKDIR /ws

# source ros2 every time you start a new bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
 && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
