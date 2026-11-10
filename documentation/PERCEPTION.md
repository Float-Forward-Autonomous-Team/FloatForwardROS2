# Perception

Three ROS 2 nodes in the `boat` package turn the boat's LiDAR and front camera into obstacles, a water/sky/obstacle segmentation and object detections. Two more nodes build on them: one shows the segmentation and the detections together in one picture, one turns the water mask into the distance to the nearest obstacle in each direction. Input topics are parameters, so the same nodes run against VRX now and against the real sensors later.

Status (November 2026): working in VRX on branch `perception/first-nodes`. The segmentation node publishes its mask for other nodes (see "For fusion"). eWaSR is fine-tuned on LaRS and measured there (see "Measuring and fine-tuning on LaRS"); it has not been measured on our own camera yet.

| Node | Reads | Produces | Model |
|---|---|---|---|
| `lidar_obstacles` | LiDAR point cloud | one box per obstacle, with distance | none, plain geometry |
| `water_segmentation` | camera image | class of every pixel (`/perception/water_mask`) and an overlay | eWaSR ResNet-18, fine-tuned on LaRS (ONNX) |
| `yolo_detector` | camera image | image with detection boxes | YOLO26-s, stock COCO weights |
| `combined_overlay` | water mask and detector image | one picture with both | none, it only combines the two |
| `water_scan` | water mask, camera_info, IMU | distance to the nearest obstacle per direction (`LaserScan`) | none, plain geometry |

## Quick start

Needs the model files described under "Models" in `models/`.

```bash
git checkout perception/first-nodes
make sh                                  # then, once per new container:
bash /ws/src/boat/models/install_deps.sh
exit
make colcon                              # registers the nodes, once after checkout
make vrx WORLD=perception_task           # leave this terminal running
make sh                                  # in a second terminal
ros2 launch boat perception.launch.py
```

All three nodes print to that one terminal; each line starts with the node name.

`perception.launch.py` arguments:

| Argument | Default | Used by |
|---|---|---|
| `image_topic` | `/wamv/sensors/cameras/front_left_camera_sensor/image_raw` | `water_segmentation`, `yolo_detector` |
| `cloud_topic` | `/wamv/sensors/lidars/lidar_wamv_sensor/points` | `lidar_obstacles` |
| `ewasr_model` | `/ws/src/boat/models/ewasr_resnet18_lars.onnx` | `water_segmentation` |
| `yolo_model` | `/ws/src/boat/models/yolo26s.pt` | `yolo_detector`; Jana's buoy model is `buoys_best.pt` |
| `confidence` | `0.5` | `yolo_detector`; detections below this score are dropped |
| `water_overlay` | `true` | `water_segmentation`; `false` publishes only the mask |
| `own_boat` | the WAM-V's hulls in the front_left_camera | `water_segmentation`; where our own boat is in the image (see "Our own boat"); `none` switches it off |
| `combined_overlay` | `true` | `false` leaves out the `combined_overlay` node |
| `water_scan` | `true` | `false` leaves out the `water_scan` node |

Examples:

```bash
ros2 launch boat perception.launch.py image_topic:=/wamv/sensors/cameras/front_right_camera_sensor/image_raw own_boat:=none
ros2 launch boat perception.launch.py ewasr_model:=/ws/src/boat/models/ewasr_resnet18.onnx   # the original model
ros2 launch boat perception.launch.py yolo_model:=/ws/src/boat/models/buoys_best.pt confidence:=0.1
```

The own-boat outline and `water_scan`'s camera position are set for the front_left_camera. With another camera, add `own_boat:=none` as in the first example (or measure that camera's outline, see "Our own boat").

**With Jana's MEBC-style buoys** (`sim_models/`, from branch `perception/sim-buoys`; `buoys_best.pt` from Jana, in `models/`): start `make vrx WORLD=perception_task`, then in a `make sh` terminal place her five buoys in a row 10 m ahead and start the nodes with her model:

```bash
python3 /ws/src/boat/sim_models/spawn_test_course.py --line --at 10 --spacing 3.5
ros2 launch boat perception.launch.py yolo_model:=/ws/src/boat/models/buoys_best.pt confidence:=0.1
```

`/perception/combined_overlay` then shows her buoy classes on top of the segmentation. The script places the row 10 m ahead of the boat's origin with 3.5 m between buoys, so `/perception/water_scan` should show the middle buoy at about 10 m and the outer ones at about 12 m (`ros2 topic echo /perception/water_scan --once`, or RViz).

To run one node with its own parameters:

```bash
ros2 run boat lidar_obstacles --ros-args -p max_range:=30.0
```

Decimal parameters need a decimal point on the command line (`0.0`, not `0`), otherwise ROS rejects the type.

## Nodes

### `lidar_obstacles`

Drops LiDAR points that are too close (our own hull), too far, or below `min_z`, puts the rest on a 2D grid and joins touching occupied cells into one object. For each object it reports centre, size, distance and bearing.

| | Topic | Type |
|---|---|---|
| Subscribes | `cloud_topic` | `sensor_msgs/msg/PointCloud2`, sensor-data QoS (best effort) |
| Publishes | `/perception/obstacle_markers` | `visualization_msgs/msg/MarkerArray` |

| Parameter | Default | Meaning |
|---|---|---|
| `cloud_topic` | `/wamv/sensors/lidars/lidar_wamv_sensor/points` | input point cloud |
| `min_range` | `3.0` | m; closer points are ignored (own hull) |
| `max_range` | `60.0` | m; farther points are ignored |
| `min_z` | `-1.5` | m in the LiDAR frame; lower points are treated as water |
| `cell` | `0.5` | m; grid cell size for grouping points |
| `min_points` | `3` | clusters with fewer points are dropped as noise |

Output: markers are in the LiDAR's own frame (header copied from the cloud; in VRX `wamv/wamv/base_link/lidar_wamv_sensor`), x forward, y left. Each scan sends a `DELETEALL`, then per object one `CUBE` (namespace `obstacles`, size = bounding box, at least 0.3 m) and one text label with the distance (namespace `labels`). Marker ids are positions in that scan's list, not persistent object ids.

Log, one line per scan with the nearest six objects: `36 obstacles: 9 m at -175 deg, 9 m at -121 deg, ...`. Bearing 0° is straight ahead, positive is to port (left).

Limitations:
- No classes: a quay and a buoy look the same, and the shore and buildings come out as a few very large clusters.
- `min_z` is applied in the LiDAR frame, which is pitched 8° down in our VRX sensor config, so it is not a true height-above-water cut. In VRX this caused no false obstacles on open water; on the real boat the cut should be done in a level frame using TF and the IMU.

### `water_segmentation`

Runs eWaSR on the camera image and labels every pixel as obstacle, water or sky, and marks the part of the image where our own boat is.

| | Topic | Type |
|---|---|---|
| Subscribes | `image_topic` | `sensor_msgs/msg/Image` (`rgb8` or `bgr8`), sensor-data QoS |
| Publishes | `/perception/water_mask` | `sensor_msgs/msg/Image` (`mono8`): per pixel 0 obstacle, 1 water, 2 sky, 3 our own boat; same size and header as the input |
| Publishes | `/perception/water_overlay` | `sensor_msgs/msg/Image` (`rgb8`), same size and header as the input; only if `publish_overlay` |

| Parameter | Default | Meaning |
|---|---|---|
| `image_topic` | `/wamv/sensors/cameras/front_left_camera_sensor/image_raw` | input camera |
| `model_path` | `/ws/src/boat/models/ewasr_resnet18_lars.onnx` | eWaSR model file; the node stops with a message if it is missing |
| `seconds_between_frames` | `1.0` | frames arriving sooner are skipped; `0.0` processes every frame |
| `publish_overlay` | `true` | `false` skips drawing the overlay (saves time on the boat) |
| `own_boat_polygons` | `""` (the WAM-V's hulls in the launch file) | outline of our own boat in the image, see below; `""` or `none`: no own boat in view |
| `report_every` | `20` | frames per timing summary |

The overlay is the camera image blended 50/50 with the classes: orange obstacle, blue water, grey sky, dark grey our own boat. Log per frame, for example: `water 60 %, sky 16 %, obstacles 15 %, own boat 10 % | eWaSR inference: 411.0 ms, total: 455.0 ms`.

**Our own boat.** The camera is fixed to the boat, so the parts of our boat it sees are always at the same pixels. In VRX the front_left_camera sees the WAM-V's two hulls in the bottom corners, and eWaSR calls them obstacle. `own_boat_polygons` outlines them, and every pixel inside the outline is published as 3 (our own boat) whatever the model says there, so `water_scan`, and anything else that reads the mask, does not take our hulls for an obstacle. Outside the outline nothing changes, so a buoy next to a hull is still an obstacle.

The outline is written as points, in fractions of the image width and height (`0,0` top left, `1,1` bottom right): `x,y x,y x,y ...`, one polygon per hull, separated by `;`. The launch file sets the outline for VRX's front_left_camera (`VRX_FRONT_LEFT_OWN_BOAT` in `perception.launch.py`), traced around the hulls on two `/perception/combined_overlay` screenshots, with a margin of about 20 pixels for the speckles the model puts next to them. It covers 10 % of the image and all of what the model marked at the hulls in both screenshots.

For another camera, or the real boat, measure it again: in `rqt_image_view` showing `/perception/water_overlay`, tick the box left of the `/perception/water_overlay_mouse_left` field, and click around the hull. Each click is published in image pixels (`ros2 topic echo /perception/water_overlay_mouse_left`); divide x by the image width and y by the height, and pass the points as `own_boat:="x,y x,y ..."`, or put them in the launch file.

Limitations:
- We use the variant without IMU input (`ewasr_resnet18`), so the model gets no horizon information. In VRX the near water, rendered as a mirror of the sky, comes out as sky. The planned fix is the IMU variant (`ewasr_resnet18_imu`, same release) with a horizon mask computed from the IMU.
- Trained on real footage: validate it on real video, not in VRX.

### `yolo_detector`

Runs YOLO26-s on the camera image and draws a box with class and confidence for each detection.

| | Topic | Type |
|---|---|---|
| Subscribes | `image_topic` | `sensor_msgs/msg/Image` (`rgb8` or `bgr8`), sensor-data QoS |
| Publishes | `/perception/yolo_overlay` | `sensor_msgs/msg/Image` (`rgb8`), same size and header as the input |

| Parameter | Default | Meaning |
|---|---|---|
| `image_topic` | `/wamv/sensors/cameras/front_left_camera_sensor/image_raw` | input camera |
| `model_path` | `/ws/src/boat/models/yolo26s.pt` | YOLO weights |
| `confidence` | `0.3` (`0.5` in the launch file) | detections below this score are dropped |
| `seconds_between_frames` | `0.5` | frames arriving sooner are skipped; `0.0` processes every frame |

Log per frame, for example: `2 detections: airplane 0.46 at x=640 y=380, airplane 0.63 at x=820 y=385  (412 ms)`; x and y are the box centre in pixels, and the time is the model call on its own.

Limitations:
- Stock COCO weights (80 everyday classes, no buoys): buoys are boxed correctly but labelled `airplane`. Needs fine-tuning on our classes.
- Our own hulls at the bottom of the image are detected at 0.3 to 0.45 confidence, which is why the launch file uses 0.5.
- Publishes only the annotated image, not the detections themselves (see "For fusion").

### `combined_overlay`

Shows segmentation and detections together: paints the water mask onto the detector's image with its boxes. It changes neither model and neither node, it only combines what the two already publish. For people watching and for the dashboard, not for other nodes.

| | Topic | Type |
|---|---|---|
| Subscribes | `mask_topic` | `sensor_msgs/msg/Image` (`mono8`) |
| Subscribes | `boxes_topic` | `sensor_msgs/msg/Image` (`rgb8` or `bgr8`) |
| Publishes | `/perception/combined_overlay` | `sensor_msgs/msg/Image` (`rgb8`), same size and header as the detector image |

| Parameter | Default | Meaning |
|---|---|---|
| `mask_topic` | `/perception/water_mask` | segmentation result |
| `boxes_topic` | `/perception/yolo_overlay` | detector image with boxes |
| `max_age` | `2.0` | s; a mask further away in time than this is not painted |
| `alpha` | `0.4` | strength of the colours: water blue, obstacle orange, our own boat dark grey, sky left as it is |

The two nodes pick their frames on their own, so each detector image gets the mask closest in time. The label in the top-left corner says which: `same frame` (on the boat, where both process every frame), `mask 0.4 s older` (in VRX on a CPU), or `detections only` when no mask is close enough.

### `water_scan`

Turns the water mask into the distance to the nearest obstacle in each direction, published like a LiDAR scan, so the planner gets a second obstacle source that does not depend on the LiDAR. For each column of the mask it takes the first pixel above the bottom of the image that is not water, where an obstacle meets the water. Our own boat (3 in the mask) is looked past like water, and so is "sky" below the horizon, which is a reflection on the water that the model mistook for sky. The camera's lens data, its height above the water and its tilt, plus the boat's roll and pitch from the IMU, turn that pixel into a point on the water. A column that is water up to the sky is free.

| | Topic | Type |
|---|---|---|
| Subscribes | `mask_topic` | `sensor_msgs/msg/Image` (`mono8`), from `water_segmentation` |
| Subscribes | `camera_info_topic` | `sensor_msgs/msg/CameraInfo`, the camera's lens data |
| Subscribes | `imu_topic` | `sensor_msgs/msg/Imu`, the boat's roll and pitch |
| Publishes | `/perception/water_scan` | `sensor_msgs/msg/LaserScan` in `frame_id`, with the camera image's timestamp |

| Parameter | Default | Meaning |
|---|---|---|
| `mask_topic` | `/perception/water_mask` | segmentation result |
| `camera_info_topic` | `/wamv/sensors/cameras/front_left_camera_sensor/camera_info` | lens data of the camera the mask comes from |
| `imu_topic` | `/wamv/sensors/imu/imu/data` | boat orientation |
| `use_imu` | `true` | `false` assumes the boat is level |
| `camera_height` | `1.6` | m above the water (VRX: 1.5 m above the boat's centre, which floats about 0.1 m above the water) |
| `camera_pitch_deg` | `15.0` | camera tilted down; `0` when the IMU sits inside the camera, as in the ZED 2i |
| `camera_roll_deg`, `camera_yaw_deg` | `0.0` | camera mount; yaw positive to port |
| `camera_x`, `camera_y` | `0.75`, `0.1` | m; camera position forward and to port of the boat's origin, so distances are from the origin |
| `frame_id` | `wamv/wamv/base_link` | frame of the published scan |
| `angle_increment_deg` | `0.5` | angle between two directions of the scan |
| `max_range` | `60.0` | m; obstacles farther away count as free |
| `ignore_bottom` | `0.0` (`0.28` in the launch file) | share of the image height at the bottom to treat as water, for model speckles next to our hulls |
| `ignore_left`, `ignore_right` | `0.5`, `0.5` (`0.30`, `0.26` in the launch file) | share of the image width from the left and right edge where `ignore_bottom` applies; `0.5` and `0.5` is the whole width |
| `column_step` | `2` | look at every n-th column |

Each value in `ranges` is the distance in metres to the nearest obstacle in that direction, `inf` where the camera sees free water up to the horizon (or the obstacle is beyond `max_range`), and `nan` where no part of the image looks in that direction. Angles are positive to port, as in any ROS scan. Log per frame, for example: `5 obstacles, left to right: 12.0 m at +34 deg, 10.5 m at +18 deg, 10.0 m at +0 deg, 10.5 m at -17 deg, 12.0 m at -33 deg, 49 % of directions free | roll +0.7, pitch +1.7 deg`. Neighbouring directions with a distance count as one object, reported at its nearest point: the outer columns of an upright object see only its upper part (the camera looks down), which reads farther away.

First run in VRX with Jana's five buoys (10 m ahead of the boat's origin, 3.5 m apart): the mooring buoy in the middle came out at 11.7 m instead of 10.0 m, because the segmentation calls the lower part of its drum water, so the obstacle seems to start higher up. The LiDAR saw the same buoys at the right distances. The geometry itself is within 1 % in the tests; this error comes from the mask. If more than 30 % of the bottom row is obstacle on the first frame, the node suggests outlining our own boat (`own_boat_polygons` in `water_segmentation`), because that is usually our own hulls.

Our own hulls: `water_segmentation` marks them as our own boat, which `water_scan` looks past. On top of that, the launch file skips the bottom corners where the hulls are (`ignore_bottom`, `ignore_left`, `ignore_right`), because the model now and then marks a speckle of water next to a hull as obstacle, which would read as an obstacle 2 to 3 m away. Only the corners: the middle of the image still finds an object close ahead between the hulls at the right distance (tested: a box 2.6 m ahead comes out at 2.60 m; skipping the whole width would put it at 3.18 m). On a VRX frame with Jana's buoys (taken from a `combined_overlay` screenshot), the model alone puts the nearest obstacle at 2.9 m (the left hull); with the hulls marked, the nearest is a buoy at 12.3 m.

"Sky" below the horizon: the model sometimes calls a patch of water sky, for example right above the hulls. Until 10 November 2026 such a patch ended the column, so `water_scan` reported that direction as free and hid what was behind it (tested: a box 2.6 m ahead behind such a patch read as free water). Now the node works out where the horizon is in each column from the camera tilt and the IMU, and looks past sky below it.

Tested against a ray-cast scene with flat water and a wall at 10, 20 and 40 m: under 1 % error in every direction the camera sees. With the boat rolled 3° and pitched 2°, the error stays under 1 % with the IMU and is 50 % without it. With the hulls marked as our own boat and no corner skip, the box 2.6 m ahead comes out at 2.60 m and the wall within 1 %. Sky patches on the water, with the boat level and with it rolled 5° and pitched 3°, change no distance.

Limitations:
- Flat-water geometry: accuracy falls with distance (at 1.6 m height and an 80° wide camera, one pixel at 30 m is about 0.6 m), and with errors in `camera_height` and `camera_pitch_deg`. Measure both on the real mount.
- Only as good as the water mask: an obstacle called water is missed, a reflection called obstacle becomes a false obstacle.
- The scan is computed in a level frame that turns with the boat, but published in `frame_id`, which pitches and rolls with the boat; at a few degrees of tilt the difference is small.
- Covers only the camera's field of view, about 80° ahead in VRX. It cannot see the water hidden behind our own hulls, so directions there are judged from what is visible above them.

## For fusion

What exists today:

| Topic | Type | Content |
|---|---|---|
| `/perception/water_mask` | `sensor_msgs/msg/Image` (`mono8`) | per pixel 0 obstacle, 1 water, 2 sky, 3 our own boat, at the camera image's resolution |
| `/perception/water_scan` | `sensor_msgs/msg/LaserScan` | distance to the nearest obstacle per direction, from the water mask (see `water_scan`) |

The mask carries the header of the camera image it came from (timestamp and `frame_id`), so a listener can match it to the LiDAR scan from the same moment and use the camera's `camera_info` to turn pixels into directions. Check it while the nodes run:

```bash
ros2 topic hz /perception/water_mask
ros2 topic echo /perception/water_scan --once
```

Proposed, to agree with the detector side and whoever writes fusion:

| Topic | Type | Content |
|---|---|---|
| `/perception/detections` | `vision_msgs/msg/Detection2DArray` | box, class and score per detection, with the camera image's header (`ros-jazzy-vision-msgs` is already in `install_deps.sh`) |
| `/perception/lidar/clusters` | to decide | per cluster: centre, size, number of points |

Still missing:
- Everything is per frame: no tracking, no persistent ids.
- Camera results are in image pixels, LiDAR results in the LiDAR frame. Nothing is in the map frame yet.

## Models

The model files are not in git (`models/*.onnx` and `models/*.pt` are git-ignored; eWaSR is 230 MB). They go in `models/` in the repo folder, which is `/ws/src/boat/models/` inside the container.

**YOLO26-s** (20 MB), inside the container:

```bash
cd /ws/src/boat/models
python3 -c "from ultralytics import YOLO; YOLO('yolo26s.pt')"
```

**eWaSR ResNet-18** (230 MB): the authors' pretrained `ewasr_resnet18` weights (github.com/tersekmatija/eWaSR, release 0.1.0; the variant without IMU input, trained on MaSTr1325), converted to ONNX by `models/export_ewasr.py`. Once, inside the container:

```bash
sudo apt-get update && sudo apt-get install -y python3-venv
cd /ws/src/boat/models
python3 -m venv /ws/export_env
source /ws/export_env/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install timm onnx
git clone https://github.com/tersekmatija/eWaSR
curl -L -o ewasr_resnet18.pth https://github.com/tersekmatija/eWaSR/releases/download/0.1.0/ewasr_resnet18.pth
python export_ewasr.py
deactivate
rm -rf eWaSR ewasr_resnet18.pth
```

It ends with `saved ewasr_resnet18.onnx`; the warnings before that are normal.

**eWaSR ResNet-18 fine-tuned on LaRS** (`ewasr_resnet18_lars.onnx`, 230 MB, the default since November 2026): made from the model above by `models/finetune_ewasr.py` (see "Measuring and fine-tuning on LaRS"). Get the file from Antonio (team drive) and put it in `models/`. Same input and output as below. To go back to the original: `ewasr_model:=/ws/src/boat/models/ewasr_resnet18.onnx`.

eWaSR input and output:
- input `image`: 1×3×384×512 float32, RGB, scaled to 0–1, then ImageNet mean (0.485, 0.456, 0.406) and std (0.229, 0.224, 0.225)
- output `logits`: 3 channels, 0 obstacle, 1 water, 2 sky; take the argmax per pixel and resize to the image size

Licences: eWaSR Apache-2.0; Ultralytics and the YOLO26 weights AGPL-3.0.

## Dependencies

Not in the team Docker image yet. They are lost whenever the container is recreated (`make down`, `make clean`, `make build`), so install them again after that. In a `make sh` terminal:

```bash
bash /ws/src/boat/models/install_deps.sh
```

It installs the packages for the perception nodes, Jana's sim buoy test, the LaRS scripts and `vision_msgs` (for a future detections topic), sets the VRX `perception_task` timer to 10 hours (see "Known issues"), and ends by checking that everything imports. Downloads that break off are retried up to 3 times. What it does, for reference:

```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-opencv python3-venv ros-jazzy-vision-msgs ros-jazzy-rviz2 ros-jazzy-rqt-image-view
pip install --break-system-packages torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install --break-system-packages onnxruntime==1.31.1 ultralytics
pip install --break-system-packages "numpy<2"
pip uninstall --break-system-packages -y opencv-python opencv-python-headless
pip uninstall --break-system-packages -y scipy      # only if pip installed one; the apt scipy stays
```

The last two lines matter: `ultralytics` pulls in numpy 2 and pip's OpenCV, while the system packages ROS uses need numpy 1.x, so we keep numpy 1.x and the apt OpenCV. Check:

```bash
python3 -c "import numpy, cv2, onnxruntime, ultralytics, rclpy; print(numpy.__version__, cv2.__version__)"
```

Expected: `1.26.4 4.6.0`.

## Viewing the output

The windows open inside the noVNC tab (`http://localhost:6080/vnc.html?resize=scale`). Start each from its own `make sh` terminal:

- `rviz2`: set Fixed Frame to `wamv/wamv/base_link/lidar_wamv_sensor`. Add, By topic: the LiDAR `points` (PointCloud2; set Reliability Policy to Best Effort and Size to 0.15) and `/perception/obstacle_markers` (MarkerArray).
- The water scan in `rviz2`: set Fixed Frame to `wamv/wamv/base_link`, then Add, By topic, `/perception/water_scan` (LaserScan; Size 0.2). Free directions show no point.
- `ros2 run rqt_image_view rqt_image_view`: pick `/perception/combined_overlay` (segmentation and detections together), `/perception/water_overlay` or `/perception/yolo_overlay`; press the refresh button if a topic is missing.

## Timing

`water_segmentation` logs per frame the model call on its own (`inference`) and the whole frame (`total`: conversion, preprocessing, model, drawing and publishing). Every `report_every` frames it adds a summary, for example:

```
eWaSR timing over 20 frames: inference mean 411.0 ms, median 398.0 ms, p95 640.2 ms, max 802.1 ms; total per frame mean 455.0 ms, so this node can do at most 2.2 frames/s
```

The first frame is left out of the summary because it includes one-off start-up cost. `yolo_detector` logs only the model time per frame, at the end of its detections line.

Numbers taken while Gazebo runs are pessimistic, because Gazebo renders on the same CPU. For clean numbers, stop the simulator (Ctrl+C in its terminal, not `make down`) and run inside the container:

```bash
cd /ws/src/boat/models
python3 benchmark.py        # options: --runs 100, --image frame.jpg, --only ewasr|yolo
```

It needs neither ROS nor Gazebo, so the same script can be run on each candidate onboard computer.

## Measuring and fine-tuning on LaRS

[LaRS](https://lojzezust.github.io/lars-dataset/) is a public maritime dataset from lakes, rivers and seas. Every pixel is labelled water, sky or obstacle, every boat and buoy has a box, and every frame is tagged with its conditions (still water, heavy reflections, extra bright, ...). It is for research use only (CC BY-NC 4.0): each person registers and downloads their own copy, and nobody passes the files on.

Setup: unzip `lars_v1.0.0_images.zip` and `lars_v1.0.0_annotations.zip` into `datasets/LaRS` in the repo folder. Git ignores `datasets/`; an empty file `datasets/COLCON_IGNORE` keeps colcon out of it.

**Measure** in the team container, on the CPU:

```bash
cd /ws/src/boat/models
python3 evaluate_lars.py --lars /ws/src/boat/datasets/LaRS       # eWaSR: water, sky, obstacle per condition
python3 evaluate_yolo_lars.py --lars /ws/src/boat/datasets/LaRS  # YOLO: boats and buoys found, by size and condition
```

Both take `--split train` for more frames and `--model A --model B` to compare two models.

To see what changed between two eWaSR models, `compare_frames.py` lists every frame of one condition with its numbers (missed and false obstacles, whole objects found) and saves side-by-side pictures in `datasets/compare/`: the image, the true labels, the old and the new model, with missed obstacles in magenta and false obstacles in yellow:

```bash
python3 compare_frames.py --lars /ws/src/boat/datasets/LaRS --condition glitter
```

**Fine-tune** in the separate GPU image (`models/training/Dockerfile`), on a PC with an NVIDIA card. From the repo folder in PowerShell:

```bash
docker build -t ff-train models/training
docker run --rm -it --gpus all --shm-size 4g -v "${PWD}:/work" ff-train python finetune_ewasr.py --lars /work/datasets/LaRS
docker run --rm -it --gpus all --shm-size 4g -v "${PWD}:/work" ff-train python finetune_yolo.py --lars /work/datasets/LaRS
```

Each script splits the LaRS training frames by video: four fifths to train on, one fifth held out, saved in `datasets/LaRS_split/`. It trains with settings fixed in advance, writes a new model file next to the current one (`ewasr_resnet18_lars.onnx`, `yolo26s_lars.pt`), and compares old and new on the held-out and the validation frames. Switch a node to the new model only if no condition is more than 1 point worse: `-p model_path:=/ws/src/boat/models/<file>`.

**Results of the eWaSR fine-tune (November 2026)**, original model → `ewasr_resnet18_lars.onnx`, in %. "False obstacles" is water called obstacle, "missed obstacles" is obstacle called water.

| Frames | mIoU | false obstacles | missed obstacles |
|---|---|---|---|
| held-out, all 529 | 89.5 → 96.2 | 5.0 → 1.1 | 5.2 → 2.4 |
| held-out, still water (28) | 71.9 → 92.2 | 27.8 → 2.8 | 12.8 → 6.6 |
| held-out, heavy reflections (13) | 57.4 → 92.9 | 43.3 → 3.1 | 7.7 → 2.3 |
| validation, all 198 | 84.7 → 95.2 | 6.6 → 1.4 | 6.7 → 3.5 |
| validation, still water (23) | 48.3 → 95.2 | 45.2 → 3.4 | 11.2 → 1.5 |
| validation, heavy reflections (10) | 49.1 → 94.2 | 58.0 → 3.4 | 2.7 → 2.2 |

mIoU went up in every condition. The only flags were missed obstacles on validation glitter (8.5 → 10.0, 16 frames) and dusk (21.6 → 23.5, 9 frames), while on the held-out frames the same conditions improved (glitter 4.8 → 3.0, dusk 6.0 → 2.7). A second run that also weighted glitter and dusk (`ewasr_resnet18_lars2`) made still water and heavy reflections worse on the held-out frames (false obstacles 2.8 → 5.4 and 3.1 → 7.2), so we kept the first. Both test sets are LaRS, so the gain on our own camera is still to be measured.

## Known issues

- **Windows:** the container exits at start with `'bash\r': No such file or directory`, because Git converted `docker/entrypoint.sh` to Windows line endings. In the repo folder: `git config core.autocrlf false`, delete `docker/entrypoint.sh`, `git checkout -- docker/entrypoint.sh`, then `make build`.
- **VRX task timer:** `perception_task` ends after 30 s of simulation time, a few minutes of real time on CPU, and Gazebo then exits. `install_deps.sh` sets it to 10 hours; by hand, inside the container: `cd /ws/src/vrx/vrx_gz/worlds` and `sed -i 's#duration>30<#duration>36000<#' perception_task.sdf`. Lost when the container is recreated.
- **Installs lost after a new container:** the pip packages live in the container. Optional, on your own laptop: put `compose.override.yml` (it mounts a volume on `/home/ubuntu/.local`) next to `compose.yml` and list it in `.git/info/exclude`; the packages then survive a new container, and `install_deps.sh` takes about a minute.
- **RViz shows no LiDAR points:** set the PointCloud2's Reliability Policy to Best Effort.
