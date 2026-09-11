"""Launch the boat package's nodes with their default parameters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the heartbeat node."""
    params = os.path.join(get_package_share_directory("boat"), "config", "params.yaml")

    return LaunchDescription(
        [
            Node(
                package="boat",
                executable="heartbeat",
                name="heartbeat",
                parameters=[params],
                output="screen",
            ),
        ]
    )
