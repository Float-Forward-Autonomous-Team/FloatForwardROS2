"""Global planner: A* over the world model's static map, replanning only when needed.

Moving objects are deliberately ignored here; they belong to the local planner.
A stopped object reaches this planner when the world model adds it to the
static map, which counts as a map update.

Status goes up the chain: when the local planner reports that the end of this
planner's path is reached, this planner reports the mission goal as reached on
``/planning/global_status``. Whether that completes a task is for the mission
manager to decide.
"""

import math
import time
from typing import NamedTuple

import numpy as np
import rclpy
from boat_interfaces.msg import NavigationStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from boat.navigation.planning.grid_astar import astar, cell_center, step_costs, world_to_cell
from boat.navigation.planning.path_geometry import distance_to_polyline
from boat.navigation.status import STATUS_QOS, StatusReporter, same_goal
from boat.navigation.world_model.layers import clearance_cells

GOAL_TOLERANCE_M = 0.5
COST_PARAMETERS = ("inflation_radius_m", "safe_distance_m", "proximity_weight")


class PlanRecord(NamedTuple):
    """What the last planning attempt used, to decide when to plan again."""

    goal: tuple
    map_version: int
    parameters: tuple
    start: tuple


class GlobalPlanner(Node):
    """Publish a collision-free route from the boat to the mission goal."""

    def __init__(self):
        """Subscribe to the static map, boat state and goal; check replan triggers at 1 Hz."""
        super().__init__("global_planner")
        self.declare_parameter("inflation_radius_m", 3.0)
        self.declare_parameter("safe_distance_m", 10.0)
        self.declare_parameter("proximity_weight", 2.0)
        self.declare_parameter("replan_deviation_m", 10.0)
        self.grid = None
        self.clearance = None
        self.map_version = 0
        self.costs = None
        self.odometry = None
        self.goal = None
        self.plan = None
        self.path_points = None
        self.last_status = None
        self.reached_goal = None

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/world_model/static_map", self._on_map, latched)
        self.create_subscription(Odometry, "/localization/odometry", self._on_odometry, 10)
        self.create_subscription(PoseStamped, "/mission/goal", self._on_goal, 10)
        self.path_pub = self.create_publisher(Path, "/planning/global_path", latched)
        self.create_subscription(
            NavigationStatus, "/planning/local_status", self._on_local_status, STATUS_QOS
        )
        self.status = StatusReporter(self, "/planning/global_status")
        self.create_timer(1.0, self._tick)
        self.get_logger().info("Global planner started; no boat commands are published")

    def _on_map(self, message):
        info = message.info
        if len(message.data) != info.width * info.height:
            self.get_logger().warn("Ignoring a map whose data does not match its size")
            return
        started = time.perf_counter()
        cells = np.array(message.data, dtype=np.int8).reshape(info.height, info.width)
        self.clearance = clearance_cells(cells)
        self.grid = message
        self.map_version += 1
        self.get_logger().info(
            f"Static map {info.width}x{info.height} received; clearance field in "
            f"{(time.perf_counter() - started) * 1000:.0f} ms"
        )

    def _on_odometry(self, message):
        self.odometry = message

    def _on_goal(self, message):
        self.goal = message
        if self.reached_goal is not None and not same_goal(
            message.pose.position, self.reached_goal, GOAL_TOLERANCE_M
        ):
            self.reached_goal = None

    def _goal_xy(self):
        if self.goal is None:
            return None
        return (self.goal.pose.position.x, self.goal.pose.position.y)

    def _on_local_status(self, message):
        """Report the goal reached when the end of the current path is reached."""
        goal_xy = self._goal_xy()
        if (
            message.state != NavigationStatus.SUCCEEDED
            or self.path_points is None
            or not same_goal(message.goal, self.path_points[-1], GOAL_TOLERANCE_M)
            or math.dist(self.path_points[-1], goal_xy) > GOAL_TOLERANCE_M
        ):
            return
        if self.reached_goal is None:
            self.get_logger().info(f"Goal ({goal_xy[0]:.1f}, {goal_xy[1]:.1f}) reached")
        self.reached_goal = goal_xy
        self._report(NavigationStatus.SUCCEEDED, "Goal reached")

    def _report(self, state, detail):
        """Report on the current goal; once it is reached, keep reporting it reached."""
        goal_xy = self._goal_xy()
        if self.reached_goal is not None and goal_xy is not None:
            state, detail = NavigationStatus.SUCCEEDED, "Goal reached"
        self.status.report(state, goal_xy, detail)

    def _tick(self):
        if self.grid is None or self.odometry is None or self.goal is None:
            self._report(NavigationStatus.IDLE, "Waiting for static map, odometry and goal")
            return
        info = self.grid.info
        frame = self.grid.header.frame_id
        if (
            not frame
            or self.odometry.header.frame_id != frame
            or self.goal.header.frame_id != frame
        ):
            self._clear("Map, boat pose and mission goal must share one frame")
            return
        parameters = tuple(self.get_parameter(name).value for name in COST_PARAMETERS)
        deviation = self.get_parameter("replan_deviation_m").value
        if not all(_non_negative(value) for value in (*parameters, deviation)):
            self._clear(", ".join((*COST_PARAMETERS, "replan_deviation_m")) + " must be >= 0")
            return

        boat = self.odometry.pose.pose.position
        goal = self.goal.pose.position
        boat_xy, goal_xy = (boat.x, boat.y), (goal.x, goal.y)
        origin = (info.origin.position.x, info.origin.position.y)
        start_cell = world_to_cell(*boat_xy, *origin, info.resolution, info.width, info.height)
        goal_cell = world_to_cell(*goal_xy, *origin, info.resolution, info.width, info.height)
        if start_cell is None or goal_cell is None:
            self._clear("Boat or mission goal is outside the static map")
            return
        costs = self._costs(parameters)
        if costs[start_cell[1] * info.width + start_cell[0]] == math.inf:
            self._clear("Boat is inside an inflated obstacle region")
            return
        if costs[goal_cell[1] * info.width + goal_cell[0]] == math.inf:
            self._clear("Mission goal is inside an inflated obstacle region")
            return

        reason = self._replan_reason(boat_xy, goal_xy, parameters, deviation)
        if reason:
            self._plan(costs, start_cell, goal_cell, boat_xy, goal_xy, parameters, reason)

    def _costs(self, parameters):
        """Return step costs for the current map and parameters, reusing the last result."""
        key = (self.map_version, parameters)
        if self.costs is None or self.costs[0] != key:
            inflation, safe_distance, weight = parameters
            resolution = self.grid.info.resolution
            self.costs = (
                key,
                step_costs(
                    self.clearance, inflation / resolution, safe_distance / resolution, weight
                ),
            )
        return self.costs[1]

    def _replan_reason(self, boat_xy, goal_xy, parameters, deviation):
        if self.plan is None:
            return "no previous plan"
        if math.dist(goal_xy, self.plan.goal) > GOAL_TOLERANCE_M:
            return "new goal"
        if self.plan.map_version != self.map_version:
            return "map updated"
        if self.plan.parameters != parameters:
            return "parameters changed"
        if self.path_points is None:
            if math.dist(boat_xy, self.plan.start) > deviation:
                return "boat moved since the failed attempt"
        elif distance_to_polyline(boat_xy, self.path_points) > deviation:
            return "boat off path"
        return None

    def _plan(self, costs, start_cell, goal_cell, boat_xy, goal_xy, parameters, reason):
        info = self.grid.info
        started = time.perf_counter()
        cells = astar(costs, info.width, info.height, start_cell, goal_cell)
        elapsed_ms = (time.perf_counter() - started) * 1000
        self.plan = PlanRecord(goal_xy, self.map_version, parameters, boat_xy)
        if cells is None:
            self.path_points = None
            self._publish([])
            self.get_logger().warn(f"No route ({reason}); search took {elapsed_ms:.0f} ms")
            self.last_status = "no route"
            self._report(NavigationStatus.BLOCKED, "No route to the goal")
            return
        origin_x, origin_y = info.origin.position.x, info.origin.position.y
        points = [boat_xy]
        points.extend(
            cell_center(cell, origin_x, origin_y, info.resolution) for cell in cells[1:-1]
        )
        points.append(goal_xy)
        self.path_points = points
        self._publish(points)
        self.get_logger().info(f"Planned {len(points)} points in {elapsed_ms:.0f} ms ({reason})")
        self.last_status = "planned"
        self._report(NavigationStatus.ACTIVE, "Following route")

    def _clear(self, problem):
        """Withdraw the current path because the inputs do not allow planning."""
        if self.path_points is not None or self.last_status != problem:
            self._publish([])
            self.get_logger().warn(problem)
        self.plan = None
        self.path_points = None
        self.last_status = problem
        self._report(NavigationStatus.BLOCKED, problem)

    def _publish(self, points):
        route = Path()
        route.header.stamp = self.get_clock().now().to_msg()
        route.header.frame_id = self.grid.header.frame_id or "map"
        for x, y in points:
            pose = PoseStamped()
            pose.header = route.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            route.poses.append(pose)
        self.path_pub.publish(route)


def _non_negative(value):
    return isinstance(value, (int, float)) and math.isfinite(value) and value >= 0


def main(args=None):
    """Run the global planner until interrupted."""
    rclpy.init(args=args)
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
