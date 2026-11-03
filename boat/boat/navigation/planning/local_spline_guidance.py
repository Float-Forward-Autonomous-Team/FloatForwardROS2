"""Publish a collision-checked spline and LOS targets for the vessel controller."""

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from boat.navigation.planning.path_smoothing import (
    generate_safe_spline,
    line_of_sight_target,
)


class LocalSplineGuidance(Node):
    """Smooth the global route and publish a lookahead target for the controller."""

    def __init__(self):
        """Set up route, map and pose inputs with guidance outputs."""
        super().__init__("local_spline_guidance")
        self.declare_parameter("lookahead_distance_m", 5.0)
        self.declare_parameter("target_speed_mps", 1.0)
        self.declare_parameter("goal_tolerance_m", 1.0)
        self.declare_parameter("slowdown_distance_m", 5.0)
        self.declare_parameter("safety_margin_m", 1.5)

        self.grid = None
        self.odometry = None
        self.route = None
        self.last_status = None
        self.completed_goal = None

        map_qos = QoSProfile(
            depth=1,  # keep only the last  
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(OccupancyGrid, "/navigation/occupancy_map", self._on_map, map_qos)
        self.create_subscription(
            Odometry, "/navigation/boat_odometry", self._on_odometry, 10
        )
        self.create_subscription(Path, "/navigation/planned_path", self._on_route, map_qos)
        self.spline_pub = self.create_publisher(Path, "/navigation/spline_path", map_qos)
        self.target_pub = self.create_publisher(PoseArray, "/guidance/target_position", 10)
        self.velocity_pub = self.create_publisher(Float64, "/guidance/target_velocity", 10)
        self.create_timer(1.0, self._update)

        self.get_logger().info(
            "Local spline guidance started; publishes controller targets, not actuator commands"
        )

    def _on_map(self, message):
        self.grid = message

    def _on_odometry(self, message):
        self.odometry = message

    def _on_route(self, message):
        self.route = message

    def _publish_empty(self, reason):
        """Clear stale guidance when inputs, validation, or route planning fail."""
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
            self._publish_empty("Waiting for occupancy map, boat odometry, and A* route")
            return

        grid, odometry, route = self.grid, self.odometry, self.route
        frame = grid.header.frame_id
        if (
            not frame
            or odometry.header.frame_id != frame
            or route.header.frame_id != frame
        ):
            self._publish_empty("Map, boat pose, and A* route must share one frame")
            return

        lookahead = self.get_parameter("lookahead_distance_m").value
        speed = self.get_parameter("target_speed_mps").value
        goal_tolerance = self.get_parameter("goal_tolerance_m").value
        slowdown_distance = self.get_parameter("slowdown_distance_m").value
        safety_margin = self.get_parameter("safety_margin_m").value
        
        if (
            not math.isfinite(lookahead)
            or lookahead <= 0
            or not math.isfinite(speed)
            or speed < 0
            or not math.isfinite(goal_tolerance)
            or goal_tolerance < 0
            or not math.isfinite(slowdown_distance)
            or slowdown_distance <= 0
            or not math.isfinite(safety_margin)
            or safety_margin < 0
        ):
            self._publish_empty(
                "Lookahead and slowdown distance must be positive; target speed and "
                "goal tolerance and safety margin must be finite and non-negative"
            )
            return

        info = grid.info
        if not route.poses:
            self._publish_empty("A* route is empty or no collision-free smoothed route exists")
            return
        route_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in route.poses
        ]
        spline = generate_safe_spline(
            route_points,
            grid.data,
            info.width,
            info.height,
            info.origin.position.x,
            info.origin.position.y,
            info.resolution,
            safety_margin,
        )
        if not spline:
            self._publish_empty("A* route is empty or no collision-free smoothed route exists")
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
            if self.last_status != "goal_reached":
                self.get_logger().info("Mission goal reached; publishing stop target")
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
