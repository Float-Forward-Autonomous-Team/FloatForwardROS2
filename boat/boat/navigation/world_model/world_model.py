"""Keep the shared environment model that the task, global and local planners read.

Inputs are a prior map (optional) and Perception's object list. Outputs are a
static occupancy grid, the objects that are still moving, and landmarks. The
planners never read simulator or Perception topics directly, so either source
can be swapped without changing planner code.
"""

import math
from pathlib import Path

import numpy as np
import rclpy
from boat_interfaces.msg import TrackedObject, TrackedObjectArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from boat.navigation.world_model.layers import paint_circle
from boat.navigation.world_model.objects import StationaryFilter

FRAME = "map"


class WorldModel(Node):
    """Merge the prior map with Perception objects into static and moving layers."""

    def __init__(self):
        """Load the prior map, publish it, and start listening to Perception."""
        super().__init__("world_model")
        self.declare_parameter("map_origin_x", -650.0)
        self.declare_parameter("map_origin_y", -50.0)
        self.declare_parameter("map_width_m", 350.0)
        self.declare_parameter("map_height_m", 400.0)
        self.declare_parameter("map_resolution_m", 0.5)
        self.declare_parameter("prior_world_sdf", "")
        self.declare_parameter("stationary_speed_mps", 0.2)
        self.declare_parameter("stationary_time_s", 5.0)

        def param(name):
            return self.get_parameter(name).value

        self.origin_x = float(param("map_origin_x"))
        self.origin_y = float(param("map_origin_y"))
        self.resolution = float(param("map_resolution_m"))
        width_m, height_m = float(param("map_width_m")), float(param("map_height_m"))
        if self.resolution <= 0 or width_m <= 0 or height_m <= 0:
            raise ValueError("Map bounds and resolution must be positive")
        width = math.ceil(width_m / self.resolution)
        height = math.ceil(height_m / self.resolution)
        self.prior = np.zeros((height, width), dtype=np.int8)
        landmarks = []
        if param("prior_world_sdf"):
            landmarks = self._load_prior(Path(param("prior_world_sdf")), width_m, height_m)
        self.filter = StationaryFilter(
            float(param("stationary_speed_mps")), float(param("stationary_time_s"))
        )

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/world_model/static_map", latched)
        self.landmarks_pub = self.create_publisher(
            TrackedObjectArray, "/world_model/landmarks", latched
        )
        self.objects_pub = self.create_publisher(
            TrackedObjectArray, "/world_model/moving_objects", 10
        )
        self.create_subscription(TrackedObjectArray, "/perception/objects", self._on_objects, 10)
        self.last_frame_warning = None
        self._publish_landmarks(landmarks)
        self._publish_static_map()

    def _load_prior(self, world_sdf, width_m, height_m):
        """Rasterize a Gazebo world as the prior map; simulation only."""
        from ament_index_python.packages import get_package_share_directory

        from boat.navigation.simulation.scenario_map import ScenarioMap

        model_roots = [
            str(Path(get_package_share_directory(package)) / "models")
            for package in ("vrx_gz", "vrx_gazebo")
        ]
        scenario = ScenarioMap(
            self.origin_x, self.origin_y, width_m, height_m, self.resolution, model_roots
        )
        scenario.load_world(world_sdf)
        width, height = scenario.width, scenario.height
        self.prior = np.array(scenario.values(), dtype=np.int8).reshape(height, width)
        self.get_logger().info(
            f"Prior map from {world_sdf.name}: {width}x{height} cells, "
            f"{scenario.collision_count} collision shapes, {len(scenario.landmarks)} landmarks"
        )
        return scenario.landmarks

    def _on_objects(self, message):
        if message.header.frame_id != FRAME:
            if self.last_frame_warning != message.header.frame_id:
                self.get_logger().warn(
                    f"Ignoring objects in frame '{message.header.frame_id}', expected '{FRAME}'"
                )
                self.last_frame_warning = message.header.frame_id
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        before = set(self.filter.static)
        if self.filter.update(message.objects, stamp):
            after = set(self.filter.static)
            for track_id in sorted(after - before):
                self.get_logger().info(f"{track_id} stopped; added to the static map")
            for track_id in sorted(before - after):
                self.get_logger().info(f"{track_id} moving again; removed from the static map")
            self._publish_static_map()
        moving = TrackedObjectArray()
        moving.header = message.header
        moving.objects = self.filter.moving(message.objects)
        self.objects_pub.publish(moving)

    def _publish_static_map(self):
        cells = self.prior.copy()
        for obstacle in self.filter.static.values():
            paint_circle(
                cells,
                self.origin_x,
                self.origin_y,
                self.resolution,
                obstacle.x,
                obstacle.y,
                obstacle.radius_m,
            )
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = FRAME
        grid.info.map_load_time = grid.header.stamp
        grid.info.resolution = self.resolution
        grid.info.width = cells.shape[1]
        grid.info.height = cells.shape[0]
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = cells.ravel().tolist()
        self.map_pub.publish(grid)

    def _publish_landmarks(self, landmarks):
        message = TrackedObjectArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = FRAME
        for landmark in landmarks:
            track = TrackedObject()
            track.id = landmark.name
            track.classification = landmark.model
            track.radius_m = landmark.radius_m
            track.pose.position.x = landmark.x
            track.pose.position.y = landmark.y
            track.pose.orientation.z = math.sin(landmark.yaw / 2)
            track.pose.orientation.w = math.cos(landmark.yaw / 2)
            message.objects.append(track)
        self.landmarks_pub.publish(message)


def main(args=None):
    """Run the world model."""
    rclpy.init(args=args)
    node = WorldModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
