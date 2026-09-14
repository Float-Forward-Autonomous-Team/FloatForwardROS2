FROM ros:jazzy-ros-base

# build tooling the ros-base image doesn't ship (colcon, rosdep)
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic + the ROS <-> Gazebo bridge, resolved from the standard
# ROS 2 apt repo already configured on this base image (ros-jazzy-ros-gz
# pulls in ros_gz_sim, ros_gz_bridge, ros_gz_interfaces, gz-sim, etc.)
# mesa-utils gives glxinfo/glxgears for diagnosing software rendering.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      mesa-utils \
 && rm -rf /var/lib/apt/lists/*

# headless GUI stack: virtual framebuffer, window manager, VNC server,
# noVNC web client (served over a websocket by websockify), and a process
# supervisor to keep them all running in the background.
RUN apt-get update && apt-get install -y --no-install-recommends \
      xvfb \
      fluxbox \
      x11vnc \
      novnc \
      websockify \
      supervisor \
 && rm -rf /var/lib/apt/lists/*

COPY docker/supervisord.conf /etc/supervisor/supervisord.conf
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# pre-create the colcon output dirs so their ownership (ubuntu, not root)
# carries over when the named volumes in compose.yml mount here empty
RUN mkdir -p /ws/src /ws/build /ws/install /ws/log && chown -R ubuntu:ubuntu /ws

# the base image already ships rosdep's source list; only the per-user
# cache is missing, and `rosdep install` needs it before it can resolve
# any dependencies.
USER ubuntu
RUN rosdep update
USER root

# force software (CPU) OpenGL rendering: there's no GPU passthrough into
# the container, so Gazebo's Ogre2 renderer must use Mesa's llvmpipe
# rasterizer instead of trying (and failing) to find a real GPU.
ENV LIBGL_ALWAYS_SOFTWARE=1
# Xvfb's virtual display number; every later `gz sim` / `ros2 launch`
# picks this up automatically because it's an image-wide ENV.
ENV DISPLAY=:1

USER ubuntu
WORKDIR /ws

# source ros2 every time you start a new bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
 && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
