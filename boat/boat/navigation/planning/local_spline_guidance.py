"""Publish a collision-checked spline and LOS targets for the vessel controller.

Guidance knows only the path it is given (``/planning/local_path``), not the
mission. It reports on ``/guidance/status`` whether it is following that path,
blocked, or at its end; the local planner decides what that means.
"""

import math

import numpy as np
import rclpy
from boat_interfaces.msg import NavigationStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from boat.navigation.planning.path_smoothing import (
    generate_safe_spline,
    line_of_sight_target,
)
from boat.navigation.status import StatusReporter
from boat.navigation.world_model.layers import clearance_cells, passable_cells


class LocalSplineGuidance(Node):
    """Smooth the global route and publish a lookahead target for the controller."""

    def __init__(self):
        """Set up route, map and pose inputs with guidance outputs."""
        super().__init__("local_spline_guidance")
        self.declare_parameter("lookahead_distance_m", 5.0)
        self.declare_parameter("target_speed_mps", 1.0)
        self.declare_parameter("goal_tolerance_m", 1.0)
        self.declare_parameter("slowdown_distance_m", 5.0)
        # Must equal the global planner's inflation_radius_m; the demo launch sets both.
        self.declare_parameter("inflation_radius_m", 3.0)

        self.grid = None
        self.clearance = None
        self.passable = None
        self.odometry = None
        self.route = None
        self.last_status = None
        self.completed_goal = None

        map_qos = QoSProfile(
            depth=1,  # keep only the last
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(OccupancyGrid, "/world_model/static_map", self._on_map, map_qos)
        self.create_subscription(Odometry, "/localization/odometry", self._on_odometry, 10)
        self.create_subscription(Path, "/planning/local_path", self._on_route, map_qos)
        self.spline_pub = self.create_publisher(Path, "/planning/smoothed_path", map_qos)
        self.target_pub = self.create_publisher(PoseArray, "/guidance/target_position", 10)
        self.velocity_pub = self.create_publisher(Float64, "/guidance/target_velocity", 10)
        self.status = StatusReporter(self, "/guidance/status")
        self.create_timer(1.0, self._update)

        self.get_logger().info(
            "Local spline guidance started; publishes controller targets, not actuator commands"
        )

    def _on_map(self, message):
        info = message.info
        if len(message.data) != info.width * info.height:
            self.get_logger().warn("Ignoring a map whose data does not match its size")
            return
        cells = np.array(message.data, dtype=np.int8).reshape(info.height, info.width)
        self.clearance = clearance_cells(cells)
        self.passable = None
        self.grid = message

    def _passable(self, inflation_radius):
        """Return the planner's passable mask for this map, rebuilt when the radius changes."""
        if self.passable is None or self.passable[0] != inflation_radius:
            mask = passable_cells(self.clearance, inflation_radius / self.grid.info.resolution)
            self.passable = (inflation_radius, mask)
        return self.passable[1]

    def _on_odometry(self, message):
        self.odometry = message

    def _on_route(self, message):
        self.route = message

    def _path_end(self):
        """Return the last point of the current path, the goal guidance reports on."""
        if self.route is None or not self.route.poses:
            return None
        end = self.route.poses[-1].pose.position
        return (end.x, end.y)

    def _publish_empty(self, reason, state=NavigationStatus.BLOCKED):
        """Clear stale guidance when inputs, validation, or route planning fail."""
        self.status.report(state, self._path_end(), reason)
        stamp = self.get_clock().now().to_msg()
        frame = self.grid.header.frame_id if self.grid else "map"
        empty_path = Path()
        empty_path.header.stamp = stamp
        empty_path.header.frame_id = frame
        self.spline_pub.publish(empty_path)

        target = PoseArray()
        target.header.stamp = stamp
        target.header.frame_id = frame
        self.target_pub.publish(target)
        self.velocity_pub.publish(Float64(data=0.0))
        if reason != self.last_status:
            self.get_logger().warn(reason)
            self.last_status = reason

    def _update(self):
        if self.grid is None or self.odometry is None or self.route is None:
            self._publish_empty(
                "Waiting for occupancy map, boat odometry, and local path", NavigationStatus.IDLE
            )
            return

        grid, odometry, route = self.grid, self.odometry, self.route
        frame = grid.header.frame_id
        if not frame or odometry.header.frame_id != frame or route.header.frame_id != frame:
            self._publish_empty("Map, boat pose, and local path must share one frame")
            return

        lookahead = self.get_parameter("lookahead_distance_m").value
        speed = self.get_parameter("target_speed_mps").value
        goal_tolerance = self.get_parameter("goal_tolerance_m").value
        slowdown_distance = self.get_parameter("slowdown_distance_m").value
        inflation_radius = self.get_parameter("inflation_radius_m").value

        if (
            not math.isfinite(lookahead)
            or lookahead <= 0
            or not math.isfinite(speed)
            or speed < 0
            or not math.isfinite(goal_tolerance)
            or goal_tolerance < 0
            or not math.isfinite(slowdown_distance)
            or slowdown_distance <= 0
            or not math.isfinite(inflation_radius)
            or inflation_radius < 0
        ):
            self._publish_empty(
                "Lookahead and slowdown distance must be positive; target speed and "
                "goal tolerance and inflation radius must be finite and non-negative"
            )
            return

        info = grid.info
        if not route.poses:
            self._publish_empty("Local path is empty", NavigationStatus.IDLE)
            return
        route_points = [(pose.pose.position.x, pose.pose.position.y) for pose in route.poses]
        spline = generate_safe_spline(
            route_points,
            self._passable(inflation_radius),
            info.origin.position.x,
            info.origin.position.y,
            info.resolution,
        )
        if not spline:
            self._publish_empty("No collision-free smoothed path exists")
            return

        boat = odometry.pose.pose.position
        goal = route.poses[-1].pose.position
        goal_point = (goal.x, goal.y)
        if (
            self.completed_goal is not None
            and math.dist(goal_point, self.completed_goal) > info.resolution
        ):
            self.completed_goal = None

        stamp = self.get_clock().now().to_msg()
        smoothed_path = Path()
        smoothed_path.header.stamp = stamp
        smoothed_path.header.frame_id = frame
        for index, (x, y) in enumerate(spline):
            pose = PoseStamped()
            pose.header = smoothed_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            next_index = min(index + 1, len(spline) - 1)
            previous_index = max(index - 1, 0)
            path_yaw = math.atan2(
                spline[next_index][1] - spline[previous_index][1],
                spline[next_index][0] - spline[previous_index][0],
            )
            pose.pose.orientation.z = math.sin(path_yaw * 0.5)
            pose.pose.orientation.w = math.cos(path_yaw * 0.5)
            smoothed_path.poses.append(pose)
        self.spline_pub.publish(smoothed_path)

        goal_distance = math.hypot(goal.x - boat.x, goal.y - boat.y)
        if self.completed_goal is not None or goal_distance <= goal_tolerance:
            self.completed_goal = goal_point
            target = PoseArray()
            target.header.stamp = stamp
            target.header.frame_id = frame
            self.target_pub.publish(target)
            self.velocity_pub.publish(Float64(data=0.0))
            self.status.report(NavigationStatus.SUCCEEDED, goal_point, "End of path reached")
            if self.last_status != "goal_reached":
                self.get_logger().info("End of path reached; publishing stop target")
                self.last_status = "goal_reached"
            return

        target_result = line_of_sight_target(spline, boat.x, boat.y, lookahead)
        if target_result is None:
            self._publish_empty("Could not select an LOS target from the smoothed route")
            return
        (target_x, target_y), target_yaw = target_result

        target = PoseArray()
        target.header.stamp = stamp
        target.header.frame_id = frame
        pose = Pose()
        pose.position.x = target_x
        pose.position.y = target_y
        pose.orientation.z = math.sin(target_yaw * 0.5)
        pose.orientation.w = math.cos(target_yaw * 0.5)
        target.poses.append(pose)
        self.target_pub.publish(target)

        target_speed = speed * min(1.0, goal_distance / slowdown_distance)
        self.velocity_pub.publish(Float64(data=float(target_speed)))
        self.status.report(NavigationStatus.ACTIVE, goal_point, "Following path")
        if self.last_status != "publishing":
            self.get_logger().info(
                f"Publishing validated spline ({len(spline)} samples) and LOS guidance"
            )
            self.last_status = "publishing"


def main(args=None):
    """Run the local spline guidance node until interrupted."""
    rclpy.init(args=args)
    node = LocalSplineGuidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
