"""Publish a visible A* route through the simulator's current occupancy grid."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from boat.navigation.planning.grid_astar import astar, cell_center, world_to_cell


class AStarNavigation(Node):
    """Plan from the current boat pose to the active mission goal once a second."""

    def __init__(self):
        """Subscribe to simulator state and publish visible routes."""
        super().__init__("astar_navigation")
        self.declare_parameter("inflation_radius_m", 3.0)
        self.grid = None
        self.odometry = None
        self.goal = None
        self.last_status = None

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/navigation/occupancy_map", self._on_map, map_qos)
        self.create_subscription(Odometry, "/navigation/boat_odometry", self._on_odometry, 10)
        self.create_subscription(PoseStamped, "/mission/goal", self._on_goal, 10)
        self.path_pub = self.create_publisher(Path, "/navigation/planned_path", map_qos)
        self.create_timer(1.0, self._plan)
        self.get_logger().info("A* route display started; no boat commands are published")

    def _on_map(self, message):
        self.grid = message

    def _on_odometry(self, message):
        self.odometry = message

    def _on_goal(self, message):
        self.goal = message

    def _empty_path(self, reason):
        route = Path()
        route.header.stamp = self.get_clock().now().to_msg()
        route.header.frame_id = self.grid.header.frame_id if self.grid else "map"
        self.path_pub.publish(route)
        if reason != self.last_status:
            self.get_logger().warn(reason)
            self.last_status = reason

    def _plan(self):
        if self.grid is None or self.odometry is None or self.goal is None:
            return
        grid = self.grid
        frame = grid.header.frame_id
        if (
            not frame
            or self.odometry.header.frame_id != frame
            or self.goal.header.frame_id != frame
        ):
            self._empty_path("Map, boat pose and mission goal must share one frame")
            return

        info = grid.info
        inflation_radius = self.get_parameter("inflation_radius_m").value
        if (
            not isinstance(inflation_radius, (int, float))
            or not math.isfinite(inflation_radius)
            or inflation_radius < 0
        ):
            self._empty_path("inflation_radius_m must be finite and non-negative")
            return
        start_position = self.odometry.pose.pose.position
        goal_position = self.goal.pose.position
        start = world_to_cell(
            start_position.x,
            start_position.y,
            info.origin.position.x,
            info.origin.position.y,
            info.resolution,
            info.width,
            info.height,
        )
        goal = world_to_cell(
            goal_position.x,
            goal_position.y,
            info.origin.position.x,
            info.origin.position.y,
            info.resolution,
            info.width,
            info.height,
        )
        if start is None or goal is None:
            self._empty_path("Boat or mission goal is outside the occupancy map")
            return

        cells = astar(
            grid.data,
            info.width,
            info.height,
            start,
            goal,
            inflation_radius_cells=inflation_radius / info.resolution,
        )
        if cells is None:
            self._empty_path("No route with the current obstacle inflation radius")
            return

        route = Path()
        route.header.stamp = self.get_clock().now().to_msg()
        route.header.frame_id = frame
        points = [(start_position.x, start_position.y)]
        points.extend(
            cell_center(cell, info.origin.position.x, info.origin.position.y, info.resolution)
            for cell in cells[1:-1]
        )
        points.append((goal_position.x, goal_position.y))
        for x, y in points:
            pose = PoseStamped()
            pose.header = route.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            route.poses.append(pose)
        self.path_pub.publish(route)
        if self.last_status != "planned":
            self.get_logger().info(f"Publishing A* route with {len(route.poses)} points")
            self.last_status = "planned"


def main(args=None):
    """Run the A* route publisher until interrupted."""
    rclpy.init(args=args)
    node = AStarNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
