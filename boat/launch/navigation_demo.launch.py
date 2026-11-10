"""Launch simulated inputs, the world model, a demo goal, and the global planner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Use the already running VRX wayfinding world as the truth source."""
    package_share = Path(get_package_share_directory("boat"))
    world_sdf = LaunchConfiguration("world_sdf")
    obstacles_file = LaunchConfiguration("obstacles_file")
    # One value for the planner's forbidden zone and guidance's smoothing check,
    # so the smoothed path never enters a zone the planner refuses to start from.
    inflation_radius = ParameterValue(LaunchConfiguration("inflation_radius_m"), value_type=float)
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_sdf", default_value="/ws/worlds/wayfinding_task.sdf"),
            DeclareLaunchArgument("inflation_radius_m", default_value="3.0"),
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
                        "world_sdf": world_sdf,
                        "obstacles_file": obstacles_file,
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
                        "world_sdf": world_sdf,
                        "obstacles_file": obstacles_file,
                    }
                ],
                output="screen",
            ),
            Node(
                package="boat",
                executable="world_model",
                name="world_model",
                parameters=[{"use_sim_time": True, "prior_world_sdf": world_sdf}],
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
                executable="global_planner",
                name="global_planner",
                parameters=[{"use_sim_time": True, "inflation_radius_m": inflation_radius}],
                output="screen",
            ),
            Node(
                package="boat",
                executable="local_planner",
                name="local_planner",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="boat",
                executable="local_spline_guidance",
                name="local_spline_guidance",
                parameters=[{"use_sim_time": True, "inflation_radius_m": inflation_radius}],
                output="screen",
            ),
            Node(
                package="boat",
                executable="pid_controller",
                name="pid_controller",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
