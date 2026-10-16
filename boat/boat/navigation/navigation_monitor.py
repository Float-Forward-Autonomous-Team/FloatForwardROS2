"""Read and report navigation inputs without planning or commanding the boat."""

import math

import rclpy
from boat_interfaces.msg import TrackedObjectArray
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix


class NavigationMonitor(Node):
    """Minimal navigation node: observe input contracts and log their contents."""

    def __init__(self):
        """Subscribe to truth inputs and report them once per second."""
        super().__init__("navigation_monitor")
        self.fix = None
        self.velocity = None
        self.odometry = None
        self.map_size = None
        self.tracks = None
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(NavSatFix, "navigation/gps_fix", self._on_fix, sensor_qos)
        self.create_subscription(
            Vector3Stamped, "navigation/gps_velocity", self._on_velocity, sensor_qos
        )
        self.create_subscription(
            Odometry, "navigation/boat_odometry", self._on_odometry, sensor_qos
        )
        self.create_subscription(OccupancyGrid, "navigation/occupancy_map", self._on_map, 1)
        self.create_subscription(
            TrackedObjectArray, "navigation/dynamic_objects", self._on_tracks, 10
        )
        self.create_timer(1.0, self._report)
        self.get_logger().info("Navigation monitor started; it publishes no commands")

    def _on_fix(self, message):
        self.fix = message

    def _on_velocity(self, message):
        self.velocity = message

    def _on_odometry(self, message):
        self.odometry = message

    def _on_map(self, message):
        self.map_size = (
            message.info.width,
            message.info.height,
            message.info.resolution,
            sum(value == 100 for value in message.data),
        )

    def _on_tracks(self, message):
        self.tracks = message

    def _report(self):
        if self.fix is None or self.velocity is None or self.odometry is None:
            missing = [
                name
                for name, value in (
                    ("GPS fix", self.fix),
                    ("GPS velocity", self.velocity),
                    ("boat odometry", self.odometry),
                )
                if value is None
            ]
            self.get_logger().info("Waiting for " + ", ".join(missing))
            return
        position = self.odometry.pose.pose.position
        velocity = self.velocity.vector
        speed = math.hypot(velocity.x, velocity.y)
        map_text = (
            f"{self.map_size[0]}x{self.map_size[1]} @ "
            f"{self.map_size[2]:.2f} m, {self.map_size[3]} occupied cells"
            if self.map_size
            else "waiting"
        )
        objects = (
            ", ".join(
                f"{track.id} ({track.pose.position.x:.1f}, {track.pose.position.y:.1f})"
                for track in self.tracks.objects
            )
            if self.tracks is not None
            else "waiting"
        )
        self.get_logger().info(
            f"GPS {self.fix.latitude:.7f}, {self.fix.longitude:.7f}; "
            f"ENU velocity ({velocity.x:.2f}, {velocity.y:.2f}) m/s "
            f"(speed {speed:.2f}); map pose ({position.x:.1f}, {position.y:.1f}); "
            f"map {map_text}; moving objects: {objects or 'none'}"
        )


def main(args=None):
    """Run the navigation input monitor."""
    rclpy.init(args=args)
    node = NavigationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
