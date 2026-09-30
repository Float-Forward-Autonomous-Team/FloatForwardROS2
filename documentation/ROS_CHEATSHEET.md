# ROS 2 cheat sheet

The topics, message types and nodes of the simulated boat. Background and
explanations are in [SIMULATION.md](SIMULATION.md).

> Names below were read from the VRX `v3.1.2` source, not from a running
> simulation. If `ros2 topic list` shows something different, fix this file.

## Sensor topics (subscribe)

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/wamv/sensors/cameras/front_left_camera_sensor/image_raw` | `sensor_msgs/msg/Image` | 30 Hz | 1280×720 RGB |
| `/wamv/sensors/cameras/front_left_camera_sensor/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | Intrinsics |
| `/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw` | `sensor_msgs/msg/Image` | 30 Hz | Same image, stamped in the optical frame |
| `/wamv/sensors/cameras/front_left_camera_sensor/optical/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | Same, optical frame |
| `/wamv/sensors/lidars/lidar_wamv_sensor/points` | `sensor_msgs/msg/PointCloud2` | 10 Hz | 16 beams × 1875 points |
| `/wamv/sensors/lidars/lidar_wamv_sensor/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | Same data as a scan |
| `/wamv/sensors/gps/gps/fix` | `sensor_msgs/msg/NavSatFix` | 20 Hz | Latitude, longitude, altitude |
| `/wamv/sensors/imu/imu/data` | `sensor_msgs/msg/Imu` | 100 Hz | Orientation (ENU), angular velocity, linear acceleration |
| `/wamv/joint_states` | `sensor_msgs/msg/JointState` | — | Engine swivel angle and propeller spin |

The other two cameras use the same four topics with `front_right_camera_sensor`
and `far_left_camera_sensor`. Use the `optical/` topics for computer vision
(OpenCV axis convention: z forward, x right, y down).

## Actuator topics (publish)

| Topic | Type | Unit and range | Effect |
|---|---|---|---|
| `/wamv/thrusters/main/thrust` | `std_msgs/msg/Float64` | newtons, −1000 to +2353 | Throttle; the last value is held |
| `/wamv/thrusters/main/pos` | `std_msgs/msg/Float64` | radians, −π to +π, 0 = straight | Engine angle; positive should turn the boat to starboard |

## Simulation, task and environment topics

| Topic | Type | Content |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Transforms |
| `/wamv/pose`, `/wamv/pose_static` | `tf2_msgs/msg/TFMessage` | Raw sensor poses from Gazebo (already forwarded to `/tf`) |
| `/vrx/task/info` | `ros_gz_interfaces/msg/ParamVec` | Task name, state, timing, score |
| `/vrx/debug/wind/speed` | `std_msgs/msg/Float32` | Current wind speed, m/s |
| `/vrx/debug/wind/direction` | `std_msgs/msg/Float32` | Wind direction, degrees |

Task-specific topics, by world:

| World | Topics |
|---|---|
| `stationkeeping_task` | `/vrx/stationkeeping/goal` (`geometry_msgs/msg/PoseStamped`), `/vrx/stationkeeping/pose_error`, `/vrx/stationkeeping/mean_pose_error` (`std_msgs/msg/Float32`) |
| `wayfinding_task` | `/vrx/wayfinding/waypoints`, `/vrx/wayfinding/mean_error`, `/vrx/wayfinding/min_errors` |
| other task worlds | see `ros2 topic list` after launching that world |

## Nodes

| Node | Job |
|---|---|
| `ros_gz_bridge` (`parameter_bridge`) | Copies the topics above between Gazebo and ROS 2 |
| `robot_state_publisher` | Publishes the boat's link frames from the URDF |
| `pose_tf_broadcaster` | Forwards Gazebo sensor poses to `/tf` |
| `optical_frame_publisher` (one per camera) | Republishes images in the optical frame |
