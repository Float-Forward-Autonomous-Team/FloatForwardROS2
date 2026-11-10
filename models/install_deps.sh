#!/usr/bin/env bash
# Installs what the perception nodes, Jana's sim buoy test and the LaRS scripts need, inside the
# team container. The team Docker image does not have these yet, and they are lost whenever the
# container is recreated (make down, make clean, make build). Then run, in a `make sh` terminal:
#
#     bash /ws/src/boat/models/install_deps.sh
#
# Takes a few minutes (torch is the big download). Safe to run again.
set -euo pipefail
# slow connections: wait up to 2 minutes for a stalled download (pip's default is 15 s) and retry more
export PIP_DEFAULT_TIMEOUT=120 PIP_RETRIES=10
# pip installs into ~/.local; with compose.override.yml that is a volume, which can start out owned by root
sudo mkdir -p "$HOME/.local" && sudo chown "$(id -un):$(id -gn)" "$HOME/.local"

sudo apt-get update
sudo apt-get install -y python3-pip python3-opencv python3-venv \
    ros-jazzy-vision-msgs ros-jazzy-rviz2 ros-jazzy-rqt-image-view

# on a slow connection a download sometimes breaks off (timeout, or "do not match the hashes"):
# try each pip step up to 3 times, with pip's download cache cleared in between
pip_retry() {
    for attempt in 1 2 3; do
        pip install --break-system-packages "$@" && return 0
        [ "$attempt" -lt 3 ] || break
        echo ">>> pip failed (attempt $attempt of 3); clearing pip's download cache and trying again"
        pip cache purge > /dev/null 2>&1 || true
        sleep 5
    done
    echo ">>> pip failed 3 times: check the internet connection, then run this script again"
    return 1
}

pip_retry torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip_retry onnxruntime==1.31.1 ultralytics  # 1.31.1 works with numpy 1.26

# ultralytics pulls in numpy 2 and pip's OpenCV; ROS needs numpy 1.x and the apt OpenCV
pip_retry "numpy<2"
pip uninstall --break-system-packages -y opencv-python opencv-python-headless || true
# a pip scipy needs numpy 2 (Jana's guide upgrades it for that); with numpy 1.x the apt scipy is the matching one
if pip list --user 2>/dev/null | grep -qi '^scipy '; then
    pip uninstall --break-system-packages -y scipy
fi

# VRX's perception_task ends after 30 s of simulated time (minutes on a CPU) and Gazebo closes:
# give it 10 hours instead. Part of the container, so it is lost with it like the packages above.
WORLD_FILE=/ws/src/vrx/vrx_gz/worlds/perception_task.sdf
if [ -w "$WORLD_FILE" ] && grep -q 'duration>30<' "$WORLD_FILE"; then
    sed -i 's#duration>30<#duration>36000<#' "$WORLD_FILE"
    echo "VRX perception_task timer set to 10 hours"
fi

python3 -c "import numpy, scipy, cv2, onnxruntime, ultralytics, rclpy, vision_msgs.msg; \
print('all imports work: numpy', numpy.__version__, '(want 1.26.x), scipy', scipy.__version__, '(want 1.11.x), cv2', cv2.__version__, '(want 4.6.x)')"
