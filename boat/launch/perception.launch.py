"""Starts the three perception nodes against VRX (or any other source of the same topics).

    ros2 launch boat perception.launch.py
    ros2 launch boat perception.launch.py image_topic:=/wamv/sensors/cameras/front_right_camera_sensor/image_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")

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
                parameters=[{"image_topic": image_topic}],
            ),
            Node(
                package="boat",
                executable="yolo_detector",
                name="yolo_detector",
                output="screen",
                parameters=[{"image_topic": image_topic, "confidence": 0.5}],
            ),
        ]
    )
