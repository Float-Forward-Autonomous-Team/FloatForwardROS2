"""Spawn and move physical Gazebo models along configurable test trajectories."""

import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.scene_pb2 import Scene
from gz.transport13 import Node as GazeboNode
from rclpy.node import Node

from boat.navigation.simulation.obstacle_trajectory import load_obstacles


def _model_sdf(name, radius):
    """Return a visible sphere with collision geometry, fixed against sinking."""
    return f"""<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="body">
      <collision name="collision">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material><ambient>1 0.35 0 1</ambient><diffuse>1 0.35 0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class GazeboObstacles(Node):
    """Drive simulator-only collidable obstacles; publish no navigation inputs."""

    def __init__(self):
        """Read trajectory configuration and connect to Gazebo services."""
        super().__init__("gazebo_obstacles")
        self.declare_parameter("world_sdf", "/ws/worlds/wayfinding_task.sdf")
        self.declare_parameter("obstacles_file", "")
        world_sdf = Path(self.get_parameter("world_sdf").value)
        self.world_name = ET.parse(world_sdf).getroot().find("world").get("name")
        self.obstacles = load_obstacles(self.get_parameter("obstacles_file").value)
        self.gz = GazeboNode()
        prefix = f"/world/{self.world_name}"
        self.scene_service = f"{prefix}/scene/info"
        self.create_service = f"{prefix}/create"
        self.move_service = f"{prefix}/set_pose"
        self.model_ids = {}
        self.last_discovery = 0.0
        self.last_warning = 0.0
        self.start_time = None
        self.create_timer(0.2, self._tick)
        self.get_logger().info(f"Configured {len(self.obstacles)} physical trajectory obstacles")

    def _discover_or_spawn(self):
        if time.monotonic() - self.last_discovery < 2.0:
            return
        self.last_discovery = time.monotonic()
        ok, scene = self.gz.request(self.scene_service, Empty(), Empty, Scene, 1000)
        if not ok:
            return
        models = {model.name: model.id for model in scene.model}
        for obstacle in self.obstacles:
            if obstacle.name in models:
                self.model_ids[obstacle.name] = models[obstacle.name]
                continue
            factory = EntityFactory()
            factory.name = obstacle.name
            factory.sdf = _model_sdf(obstacle.name, obstacle.radius_m)
            factory.pose.position.x, factory.pose.position.y = obstacle.waypoints_xy[0]
            factory.pose.position.z = obstacle.radius_m * 0.5
            factory.pose.orientation.w = 1.0
            created, response = self.gz.request(
                self.create_service, factory, EntityFactory, Boolean, 1000
            )
            if created and response.data:
                self.get_logger().info(f"Spawned collidable {obstacle.name}")

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0:
            return
        if self.start_time is not None and now < self.start_time:
            self.model_ids.clear()
        if self.start_time is None or now < self.start_time:
            self.start_time = now
        if len(self.model_ids) < len(self.obstacles):
            self._discover_or_spawn()
        elapsed = now - self.start_time
        for obstacle in self.obstacles:
            model_id = self.model_ids.get(obstacle.name)
            if model_id is None:
                continue
            x, y, _, _ = obstacle.position_velocity(elapsed)
            pose = Pose()
            pose.id = model_id
            pose.position.x = x
            pose.position.y = y
            pose.position.z = obstacle.radius_m * 0.5
            pose.orientation.w = 1.0
            ok, response = self.gz.request(self.move_service, pose, Pose, Boolean, 1000)
            if (not ok or not response.data) and time.monotonic() - self.last_warning > 5.0:
                self.last_warning = time.monotonic()
                self.get_logger().warn(f"Pose request for {obstacle.name} was not confirmed")


def main(args=None):
    """Run the physical obstacle generator."""
    rclpy.init(args=args)
    node = GazeboObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
