"""Launch a race stage plus everything the Foxglove dashboard needs, next to a running `make vrx`.

Starts foxglove_bridge, the circle driver (placeholder race stage), the GPS track
and the ground-truth obstacle markers. Start the simulator first: the driver frees
the boat from its start platform only during its first seconds.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
