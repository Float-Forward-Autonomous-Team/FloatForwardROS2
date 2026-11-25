"""Launch the test slalom next to `make vrx-race`, with Foxglove, GPS track and perception.

Everything runs on simulation time: the simulator is slower than real time on a laptop,
and the race time must not depend on that.

The perception nodes only show what they see; the boat still steers on GPS. A camera node
whose Python package or model file is missing is left out, with a message saying what to do.
"""

import os
from importlib.util import find_spec

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SIM_TIME = {"use_sim_time": True}
CAMERA = "/wamv/sensors/cameras/{}_camera_sensor/optical/{}"
MODELS = "/ws/src/boat/models/"
# The first file that exists is used: our own fine-tuned models, then the public ones.
YOLO_MODELS = [MODELS + "buoys_best.pt", MODELS + "yolo26s.pt"]
EWASR_MODELS = [MODELS + "ewasr_resnet18_lars.onnx", MODELS + "ewasr_resnet18.onnx"]


def first_file(choice, candidates):
    """Return the chosen model, or the first candidate that exists; None if there is none."""
    files = candidates if choice == "auto" else [choice]
    return next((f for f in files if os.path.exists(f)), None)


def missing(what, package, model):
    """Return why a camera node cannot start, or None if it can."""
    if find_spec(package) is None:
        return f"{what} is off: Python package '{package}' missing, run `make perception-deps`"
    if model is None:
        return f"{what} is off: no model file in models/, see the README"
    return None


def yolo(name, camera, model, overlay, config):
    """Return a detector node for one camera."""
    return Node(
        package="boat",
        executable="yolo_detector",
        name=name,
        parameters=[
            config,
            SIM_TIME,
            {
                "image_topic": CAMERA.format(camera, "image_raw"),
                "model_path": model,
                "confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
            },
        ],
        remappings=[("/perception/yolo_overlay", overlay)],
        output="screen",
    )


def perception(context):
    """Return the perception nodes that can run here, and a message for those that cannot."""

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    if arg("perception").lower() != "true":
        return []
    config = os.path.join(
        get_package_share_directory("boat"), "config", "perception_slalom.yaml"
    )
    actions = [
        Node(
            package="boat",
            executable="lidar_obstacles",
            name="lidar_obstacles",
            parameters=[config, SIM_TIME],
            output="screen",
        )
    ]

    yolo_model = first_file(arg("yolo_model"), YOLO_MODELS)
    problem = missing("YOLO detection", "ultralytics", yolo_model)
    if problem:
        actions.append(LogInfo(msg=problem))
    else:
        actions.append(LogInfo(msg=f"YOLO model: {yolo_model}"))
        overlay = "/perception/yolo_overlay"
        actions.append(yolo("yolo_detector", "front", yolo_model, overlay, config))
        if arg("rear_detection").lower() == "true":
            overlay = "/perception/rear/yolo_overlay"
            actions.append(yolo("yolo_detector_rear", "rear", yolo_model, overlay, config))
        actions.append(
            Node(
                package="boat",
                executable="combined_overlay",
                name="combined_overlay",
                parameters=[SIM_TIME],
                output="screen",
            )
        )

    ewasr_model = first_file(arg("ewasr_model"), EWASR_MODELS)
    problem = missing("water segmentation", "onnxruntime", ewasr_model)
    if problem:
        actions.append(LogInfo(msg=problem))
    else:
        actions.append(LogInfo(msg=f"segmentation model: {ewasr_model}"))
        actions.append(
            Node(
                package="boat",
                executable="water_segmentation",
                name="water_segmentation",
                parameters=[
                    config,
                    SIM_TIME,
                    {
                        "image_topic": CAMERA.format("front", "image_raw"),
                        "model_path": ewasr_model,
                    },
                ],
                output="screen",
            )
        )
        actions.append(
            Node(
                package="boat",
                executable="water_scan",
                name="water_scan",
                parameters=[
                    config,
                    SIM_TIME,
                    {"camera_info_topic": CAMERA.format("front", "camera_info")},
                ],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    """Build the launch description for the test slalom."""
    default = os.path.join(get_package_share_directory("boat"), "config", "race_slalom.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("course", default_value=default, description="race config file"),
            DeclareLaunchArgument(
                "foxglove", default_value="true", description="also start foxglove_bridge"
            ),
            DeclareLaunchArgument(
                "perception", default_value="true", description="also start the perception nodes"
            ),
            DeclareLaunchArgument(
                "rear_detection",
                default_value="false",
                description="true: a second YOLO on the rear camera (about 0.5 GB more memory)",
            ),
            DeclareLaunchArgument(
                "yolo_model", default_value="auto", description="detector weights (.pt)"
            ),
            DeclareLaunchArgument(
                "ewasr_model", default_value="auto", description="segmentation model (.onnx)"
            ),
            DeclareLaunchArgument(
                "confidence", default_value="0.3", description="lowest detection score shown"
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                condition=IfCondition(LaunchConfiguration("foxglove")),
                parameters=[SIM_TIME],
                output="screen",
            ),
            Node(
                package="boat",
                executable="slalom_race",
                name="slalom_race",
                parameters=[LaunchConfiguration("course"), SIM_TIME],
                output="screen",
            ),
            Node(
                package="boat",
                executable="gps_track",
                name="gps_track",
                parameters=[SIM_TIME],
                output="screen",
            ),
            OpaqueFunction(function=perception),
        ]
    )
