"""Launch simulator truth inputs, a demo goal, and a visible A* route."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Use the already running VRX wayfinding world as the truth source."""
    package_share = Path(get_package_share_directory("boat"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_sdf", default_value="/ws/worlds/wayfinding_task.sdf"),
            DeclareLaunchArgument(
                "obstacles_file",
                default_value=str(package_share / "config" / "moving_obstacles.yaml"),
            ),
            Node(
                package="boat",
                executable="gazebo_obstacles",
                name="gazebo_obstacles",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_sdf": LaunchConfiguration("world_sdf"),
                        "obstacles_file": LaunchConfiguration("obstacles_file"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="boat",
                executable="sim_truth",
                name="sim_truth",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_sdf": LaunchConfiguration("world_sdf"),
                        "obstacles_file": LaunchConfiguration("obstacles_file"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="boat",
                executable="navigation_monitor",
                name="navigation_monitor",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="boat",
                executable="mission_manager",
                name="mission_manager",
                parameters=[{"use_sim_time": True, "task_timeout_sec": 3600.0}],
                output="screen",
            ),
            Node(
                package="boat",
                executable="astar_navigation",
                name="astar_navigation",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
