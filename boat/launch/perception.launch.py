"""Starts the perception nodes against VRX (or any other source of the same topics): LiDAR obstacles,
water segmentation, detector, a combined view, and the water scan (distances from the water mask).

    ros2 launch boat perception.launch.py
    ros2 launch boat perception.launch.py image_topic:=/wamv/sensors/cameras/front_right_camera_sensor/image_raw \
        own_boat:=none
    ros2 launch boat perception.launch.py ewasr_model:=/ws/src/boat/models/ewasr_resnet18.onnx
    ros2 launch boat perception.launch.py yolo_model:=/ws/src/boat/models/buoys_best.pt confidence:=0.1

The segmentation node publishes /perception/water_mask (mono8: 0 obstacle, 1 water, 2 sky, 3 our own boat)
for other nodes. combined_overlay shows segmentation and detections together on /perception/combined_overlay
(combined_overlay:=false to leave it out). water_scan turns the water mask into the distance to the
nearest obstacle per direction, as a LaserScan on /perception/water_scan (water_scan:=false to leave it out).

The own-boat outline and water_scan's camera position are for VRX's front_left_camera. For another
camera (or the real boat) measure the outline again: see "Our own boat" in documentation/PERCEPTION.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Where the WAM-V's own hulls are in VRX's front_left_camera image, as x,y fractions of the image width and
# height (0,0 top left, 1,1 bottom right), one outline per hull, separated by ";". Traced around the hulls
# on /perception/combined_overlay screenshots (Nov 2026), with a margin for the model's speckles next to them.
VRX_FRONT_LEFT_OWN_BOAT = (
    "0.06,1.0 0.06,0.96 0.18,0.72 0.235,0.72 0.275,0.77 0.275,0.975 0.245,1.0; "  # left hull
    "0.77,1.0 0.765,0.95 0.79,0.775 0.82,0.72 0.87,0.72 1.0,0.81 1.0,1.0"  # right hull
)


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    water_overlay = ParameterValue(LaunchConfiguration("water_overlay"), value_type=bool)
    own_boat = ParameterValue(LaunchConfiguration("own_boat"), value_type=str)

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
                "own_boat",
                default_value=VRX_FRONT_LEFT_OWN_BOAT,
                description="outline of our own boat in the image (see VRX_FRONT_LEFT_OWN_BOAT); none = off",
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
                        "own_boat_polygons": own_boat,
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
                # the hulls come in marked as our own boat (3), which water_scan looks past; on top of that it
                # skips the bottom corners where they are (VRX front_left_camera), because the model sometimes
                # marks a speckle of water next to a hull as obstacle; the middle still sees what is close ahead
                parameters=[{"ignore_bottom": 0.28, "ignore_left": 0.30, "ignore_right": 0.26}],
                condition=IfCondition(LaunchConfiguration("water_scan")),
            ),
        ]
    )
