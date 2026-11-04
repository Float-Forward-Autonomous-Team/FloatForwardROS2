"""Simulation-only: publish Gazebo's ground-truth obstacle positions as Foxglove/RViz markers.

Reads every model pose straight from Gazebo (/world/<world>/pose/info), keeps the
configured obstacles that are within max_range of the boat, and publishes them as
a MarkerArray in the boat's frame. Works only in simulation, since a real boat has no
ground truth. Useful as a stand-in until a detection node exists, and later as the
reference to check detections against.
"""

import math
import signal
import threading

import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

# Marker colour per obstacle name prefix; anything else is drawn grey.
COLORS = {
    "red": (0.9, 0.1, 0.1),
    "green": (0.1, 0.8, 0.2),
    "black": (0.15, 0.15, 0.15),
    "blue": (0.1, 0.3, 0.9),
}


def yaw_from_quaternion(q):
    """Return the heading (rotation about z) of a gz.msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ObstacleTruth(Node):
    """Turns Gazebo model poses into obstacle markers around the boat."""

    def __init__(self):
        """Subscribe to Gazebo's pose info and start the publish timer."""
        super().__init__("obstacle_truth")
        self.declare_parameter("world", "")
        self.declare_parameter("boat_model", "wamv")
        self.declare_parameter("frame_id", "wamv/wamv/base_link")
        self.declare_parameter("obstacles", ["red_0", "green_0", "black_0", "blue_projectile"])
        self.declare_parameter("max_range", 100.0)
        self.declare_parameter("rate_hz", 10.0)

        self.boat_model = self.get_parameter("boat_model").value
        self.frame_id = self.get_parameter("frame_id").value
        self.obstacles = set(self.get_parameter("obstacles").value)
        self.max_range = self.get_parameter("max_range").value
        rate = self.get_parameter("rate_hz").value

        self.gz = GzNode()
        topic = self.find_pose_topic(self.get_parameter("world").value)
        self.lock = threading.Lock()
        self.poses = {}
        if not self.gz.subscribe(Pose_V, topic, self.on_poses):
            raise RuntimeError(f"could not subscribe to {topic}")

        self.pub = self.create_publisher(MarkerArray, "obstacles/markers", 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_markers)
        self.get_logger().info(f"reading {topic}, tracking {sorted(self.obstacles)}")

    def find_pose_topic(self, world):
        """Return /world/<world>/pose/info, auto-detecting the world if none was given."""
        if world:
            return f"/world/{world}/pose/info"
        for topic in self.gz.topic_list():
            if topic.startswith("/world/") and topic.endswith("/pose/info"):
                return topic
        raise RuntimeError("no /world/*/pose/info topic found; is the simulator running?")

    def on_poses(self, msg):
        """Store the latest world pose of the boat and each tracked obstacle (gz thread)."""
        wanted = self.obstacles | {self.boat_model}
        latest = {p.name: p for p in msg.pose if p.name in wanted}
        with self.lock:
            self.poses.update(latest)

    def publish_markers(self):
        """Publish a cylinder and a name label for every obstacle near the boat."""
        with self.lock:
            boat = self.poses.get(self.boat_model)
            obstacles = [self.poses[n] for n in self.obstacles if n in self.poses]
        if boat is None:
            return

        # Express each obstacle in the boat frame (heading only; roll/pitch are small).
        yaw = yaw_from_quaternion(boat.orientation)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for i, obs in enumerate(sorted(obstacles, key=lambda p: p.name)):
            dx = obs.position.x - boat.position.x
            dy = obs.position.y - boat.position.y
            dist = math.hypot(dx, dy)
            if dist > self.max_range:
                continue
            x = cos_y * dx + sin_y * dy
            y = -sin_y * dx + cos_y * dy
            color = next((c for k, c in COLORS.items() if obs.name.startswith(k)), (0.6, 0.6, 0.6))
            markers.markers.append(self.make_marker(i, Marker.CYLINDER, x, y, 0.5, color, stamp))
            label = self.make_marker(i, Marker.TEXT_VIEW_FACING, x, y, 2.0, (1.0, 1.0, 1.0), stamp)
            label.text = f"{obs.name}  {dist:.1f} m"
            markers.markers.append(label)
        self.pub.publish(markers)

    def make_marker(self, i, kind, x, y, z, color, stamp):
        """Build one marker; it expires quickly so removed obstacles disappear."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = "label" if kind == Marker.TEXT_VIEW_FACING else "obstacle"
        m.id = i
        m.type = kind
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        if kind == Marker.TEXT_VIEW_FACING:
            m.scale.z = 0.8
        else:
            m.scale.x = m.scale.y = 0.8
            m.scale.z = 1.5
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 0.85
        m.lifetime.nanosec = 500_000_000
        return m


def main(args=None):
    """Start the obstacle_truth node and spin until interrupted."""
    rclpy.init(args=args)
    node = ObstacleTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch forwards Ctrl+C on top of the terminal's own, so a second
        # SIGINT can land mid-cleanup; ignore it so shutdown finishes quietly.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
