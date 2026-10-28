"""One command to test the buoy detector on MEBC-style buoys in VRX.

Start the sim first and wait for "ScoringPlugin::OnRunning" in its log:
    make vrx WORLD=navigation_task HEADLESS=True
Then, in a `make sh` terminal:
    ros2 launch /ws/src/boat/sim_models/sim_buoy_test.launch.py

What it starts:
  1. a camera + LiDAR bridge for the world's real name (works around navigation_task
     publishing its sensors under "sydney_regatta", which VRX's own bridge misses)
  2. the five textured MEBC-style buoys in a row in front of the boat (old ones removed first)
  3. yolo_detector with our buoy model, as "buoy_detector", on /perception/buoy_overlay
  4. an image viewer on /perception/buoy_overlay

Options (name:=value):
  world:=sydney_regatta   internal world name (check: gz service -l | grep /create)
  bridge:=true            false for worlds whose own bridge works (e.g. perception_task)
  at:=10 spacing:=3.5     row distance and gap, metres
  model:=/ws/src/boat/models/buoys_best.pt   confidence:=0.1
  viewer:=true            false to skip the image window
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SPAWN = "/ws/src/boat/sim_models/spawn_test_course.py"


def setup(context):
    """Build the launch actions from the resolved launch arguments."""

    def get(name):
        return LaunchConfiguration(name).perform(context)

    world = get("world")
    sensor = f"/world/{world}/model/wamv/link/wamv/base_link/sensor"
    cam_gz = f"{sensor}/front_left_camera_sensor/image"
    lidar_gz = f"{sensor}/lidar_wamv_sensor/scan/points"
    actions = []

    if get("bridge").lower() == "true":
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="buoy_test_bridge",
                output="screen",
                arguments=[
                    f"{cam_gz}@sensor_msgs/msg/Image[gz.msgs.Image",
                    f"{lidar_gz}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                ],
                remappings=[
                    (cam_gz, "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
                    (lidar_gz, "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
                ],
            )
        )

    # remove old test buoys, then spawn the row (after a short wait so Gazebo is ready)
    actions.append(
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-c",
                        f"python3 {SPAWN} --remove && python3 {SPAWN} --line "
                        f"--at {get('at')} --spacing {get('spacing')}",
                    ],
                    output="screen",
                )
            ],
        )
    )

    actions.append(
        Node(
            package="boat",
            executable="yolo_detector",
            name="buoy_detector",
            output="screen",
            parameters=[{"model_path": get("model"), "confidence": float(get("confidence"))}],
            remappings=[("/perception/yolo_overlay", "/perception/buoy_overlay")],
        )
    )

    if get("viewer").lower() == "true":
        actions.append(
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                arguments=["/perception/buoy_overlay"],
                output="log",
            )
        )
    return actions


def generate_launch_description():
    """Declare the options and start everything through setup()."""
    defaults = {
        "world": "sydney_regatta",
        "bridge": "true",
        "at": "10",
        "spacing": "3.5",
        "model": "/ws/src/boat/models/buoys_best.pt",
        "confidence": "0.1",
        "viewer": "true",
    }
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=setup)]
    )
