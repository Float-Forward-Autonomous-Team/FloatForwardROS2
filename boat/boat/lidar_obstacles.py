import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray


class LidarObstacles(Node):
    """Finds obstacles (buoys, boats, docks) in the WAM-V LiDAR point cloud."""

    def __init__(self):
        super().__init__("lidar_obstacles")
        self.declare_parameter("cloud_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points")
        self.declare_parameter("min_range", 3.0)  # m, ignore our own boat
        self.declare_parameter("max_range", 60.0)  # m
        self.declare_parameter("min_z", -1.5)  # m below the LiDAR, lower = water
        self.declare_parameter("cell", 0.5)  # m, grid size for grouping points
        self.declare_parameter("min_points", 3)  # fewer points = noise

        topic = self.get_parameter("cloud_topic").value
        self.sub = self.create_subscription(PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(MarkerArray, "/perception/obstacle_markers", 10)
        self.get_logger().info(f"Listening to {topic}")

    def on_cloud(self, msg):
        # 1. point cloud -> N x 3 array of x, y, z (metres, in the LiDAR frame)
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if len(pts) == 0:
            return

        # 2. keep only points that can be obstacles
        dist = np.hypot(pts[:, 0], pts[:, 1])
        keep = (
            (dist > self.get_parameter("min_range").value)
            & (dist < self.get_parameter("max_range").value)
            & (pts[:, 2] > self.get_parameter("min_z").value)
        )
        pts = pts[keep]

        # 3. group nearby points into objects
        clusters = self.cluster(
            pts[:, :2], self.get_parameter("cell").value, self.get_parameter("min_points").value
        )

        # 4. describe each object: centre, size, distance, bearing
        objects = []
        for idx in clusters:
            p = pts[idx]
            centre = p.mean(axis=0)
            size = p.max(axis=0) - p.min(axis=0)
            distance = math.hypot(centre[0], centre[1])
            bearing = math.degrees(math.atan2(centre[1], centre[0]))  # 0 = ahead, + = left
            objects.append((distance, bearing, centre, size))
        objects.sort(key=lambda o: o[0])

        # 5. report: one line in the terminal + markers for RViz
        text = ", ".join(f"{d:.0f} m at {b:+.0f} deg" for d, b, _, _ in objects[:6])
        self.get_logger().info(f"{len(objects)} obstacles: {text}")
        self.pub.publish(self.make_markers(objects, msg.header))

    @staticmethod
    def cluster(xy, cell, min_points):
        """Put points in a grid; touching occupied cells form one object."""
        cells = {}
        for i, key in enumerate(map(tuple, np.floor(xy / cell).astype(int))):
            cells.setdefault(key, []).append(i)
        seen, clusters = set(), []
        for start in cells:
            if start in seen:
                continue
            seen.add(start)
            stack, members = [start], []
            while stack:
                cx, cy = stack.pop()
                members += cells[(cx, cy)]
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nb = (cx + dx, cy + dy)
                        if nb in cells and nb not in seen:
                            seen.add(nb)
                            stack.append(nb)
            if len(members) >= min_points:
                clusters.append(members)
        return clusters

    @staticmethod
    def make_markers(objects, header):
        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, (distance, bearing, centre, size) in enumerate(objects):
            box = Marker()
            box.header = header
            box.ns, box.id = "obstacles", i
            box.type, box.action = Marker.CUBE, Marker.ADD
            box.pose.position.x, box.pose.position.y, box.pose.position.z = map(float, centre)
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = (max(0.3, float(s)) for s in size)
            box.color.r, box.color.g, box.color.b, box.color.a = 1.0, 0.4, 0.0, 0.8

            label = Marker()
            label.header = header
            label.ns, label.id = "labels", i
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y = float(centre[0]), float(centre[1])
            label.pose.position.z = float(centre[2]) + 1.5
            label.pose.orientation.w = 1.0
            label.scale.z = 1.0
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"{distance:.0f} m"
            markers.markers += [box, label]
        return markers


def main():
    rclpy.init()
    node = LidarObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
