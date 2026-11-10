"""Starts the perception nodes against VRX (or any other source of the same topics): LiDAR obstacles,
water segmentation, detector, a combined view, and the water scan (distances from the water mask).

    ros2 launch boat perception.launch.py
    ros2 launch boat perception.launch.py image_topic:=/wamv/sensors/cameras/front_right_camera_sensor/image_raw
    ros2 launch boat perception.launch.py ewasr_model:=/ws/src/boat/models/ewasr_resnet18.onnx
    ros2 launch boat perception.launch.py yolo_model:=/ws/src/boat/models/buoys_best.pt confidence:=0.1

The segmentation node publishes /perception/water_mask (mono8: 0 obstacle, 1 water, 2 sky) for other nodes.
combined_overlay shows segmentation and detections together on /perception/combined_overlay
(combined_overlay:=false to leave it out). water_scan turns the water mask into the distance to the
nearest obstacle per direction, as a LaserScan on /perception/water_scan (water_scan:=false to leave it out).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    water_overlay = ParameterValue(LaunchConfiguration("water_overlay"), value_type=bool)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/wamv/sensors/cameras/front_left_camera_sensor/image_raw",
                description="Camera image topic for segmentation and detection",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/wamv/sensors/lidars/lidar_wamv_sensor/points",
                description="LiDAR point cloud topic for obstacle clustering",
            ),
            DeclareLaunchArgument(
                "ewasr_model",
                default_value="/ws/src/boat/models/ewasr_resnet18_lars.onnx",
                description="eWaSR model; ewasr_resnet18_lars.onnx is our LaRS fine-tune (Nov 2026)",
            ),
            DeclareLaunchArgument(
                "yolo_model",
                default_value="/ws/src/boat/models/yolo26s.pt",
                description="detector weights: yolo26s.pt (stock COCO) or Jana's buoys_best.pt (our buoys)",
            ),
            DeclareLaunchArgument(
                "confidence",
                default_value="0.5",
                description="detections below this score are dropped",
            ),
            DeclareLaunchArgument(
                "water_overlay",
                default_value="true",
                description="false: water_segmentation publishes only the mask, not the overlay image",
            ),
            DeclareLaunchArgument(
                "combined_overlay",
                default_value="true",
                description="false: do not start the node that shows segmentation and detections together",
            ),
            DeclareLaunchArgument(
                "water_scan",
                default_value="true",
                description="false: do not start the node that turns the water mask into distances",
            ),
            Node(
                package="boat",
                executable="lidar_obstacles",
                name="lidar_obstacles",
                output="screen",
                parameters=[{"cloud_topic": cloud_topic}],
            ),
            Node(
                package="boat",
                executable="water_segmentation",
                name="water_segmentation",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "model_path": LaunchConfiguration("ewasr_model"),
                        "publish_overlay": water_overlay,
                    }
                ],
            ),
            Node(
                package="boat",
                executable="yolo_detector",
                name="yolo_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "model_path": LaunchConfiguration("yolo_model"),
                        "confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
                    }
                ],
            ),
            Node(
                package="boat",
                executable="combined_overlay",
                name="combined_overlay",
                output="screen",
                condition=IfCondition(LaunchConfiguration("combined_overlay")),
            ),
            Node(
                package="boat",
                executable="water_scan",
                name="water_scan",
                output="screen",
                # VRX front_left_camera; the WAM-V's hulls show in the bottom corners of its image
                parameters=[{"ignore_bottom": 0.12}],
                condition=IfCondition(LaunchConfiguration("water_scan")),
            ),
        ]
    )
