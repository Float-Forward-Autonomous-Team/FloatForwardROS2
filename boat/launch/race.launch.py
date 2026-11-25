"""Launch a race stage plus everything the Foxglove dashboard needs, next to a running `make vrx`.

Starts foxglove_bridge, the circle driver (placeholder race stage), the GPS track
and the ground-truth obstacle markers. With public:=true (the default) it also starts
a second, read-only foxglove_bridge on port 8766 for the public live page, plus the
node that turns the cameras into small JPEGs for it. Start the simulator first: the driver frees
the boat from its start platform only during its first seconds.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The only topics the public page may see. Full-match regexes, as foxglove_bridge expects.
PUBLIC_TOPICS = [
    "/wamv/sensors/gps/gps/fix",
    "/gps_track",
    "/wamv/sensors/imu/imu/data",
    "/obstacles/markers",
    "/detections",
    "/detections/inference_ms",
    "/public/cameras/[a-z_]+/compressed",
]
NOTHING = ["^$"]  # matches no name: no publishing, services or parameters from outside


def generate_launch_description():
    """Build the launch description for the race stage and dashboard nodes."""
    thrust = LaunchConfiguration("thrust")
    steer = LaunchConfiguration("steer")

    return LaunchDescription(
        [
            DeclareLaunchArgument("thrust", default_value="600.0", description="engine thrust, N"),
            DeclareLaunchArgument("steer", default_value="0.3", description="engine angle, rad"),
            DeclareLaunchArgument(
                "foxglove", default_value="true", description="also start foxglove_bridge"
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                condition=IfCondition(LaunchConfiguration("foxglove")),
                output="screen",
            ),
            DeclareLaunchArgument(
                "public",
                default_value="true",
                description="also start the read-only bridge for the public live page",
            ),
            # Public bridge: this is the one exposed to the internet through the tunnel.
            # It can only stream the topics above; clients cannot publish, call services
            # or read and set parameters, so nothing reaching it can command the boat.
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge_public",
                condition=IfCondition(LaunchConfiguration("public")),
                parameters=[
                    {
                        "port": 8766,
                        "capabilities": ["time"],
                        "topic_whitelist": PUBLIC_TOPICS,
                        "client_topic_whitelist": NOTHING,
                        "service_whitelist": NOTHING,
                        "param_whitelist": NOTHING,
                        # the bridge's own CPU/memory channel ignores topic_whitelist: turn it off
                        "sysinfo": False,
                        "remote_access": False,
                        # a slow viewer drops frames instead of piling up a backlog
                        "send_buffer_limit": 2_000_000,
                    }
                ],
                output="screen",
            ),
            Node(
                package="boat",
                executable="public_camera",
                name="public_camera",
                condition=IfCondition(LaunchConfiguration("public")),
                output="screen",
            ),
            Node(
                package="boat",
                executable="circle_driver",
                name="circle_driver",
                parameters=[{"thrust": thrust, "steer": steer}],
                output="screen",
            ),
            Node(package="boat", executable="gps_track", name="gps_track", output="screen"),
            Node(
                package="boat", executable="obstacle_truth", name="obstacle_truth", output="screen"
            ),
        ]
    )
