"""Collect the boat's GPS fixes into a track and publish it as GeoJSON for Foxglove's Map panel.

The Map panel only shows the latest NavSatFix on a live connection, so the path the
boat has driven is lost. This node remembers the fixes and republishes the whole
track as one LineString.
"""

import json
import math
import signal

import rclpy
from foxglove_msgs.msg import GeoJSON
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

EARTH_RADIUS = 6371000.0  # m


def distance(a, b):
    """Return the approximate distance in metres between two (lon, lat) points."""
    dlat = math.radians(b[1] - a[1])
    dlon = math.radians(b[0] - a[0]) * math.cos(math.radians(a[1]))
    return EARTH_RADIUS * math.hypot(dlat, dlon)


class GpsTrack(Node):
    """Turns GPS fixes into a growing GeoJSON line."""

    def __init__(self):
        """Subscribe to the GPS and start the publish timer."""
        super().__init__("gps_track")
        self.declare_parameter("gps_topic", "/wamv/sensors/gps/gps/fix")
        self.declare_parameter("min_step", 0.5)
        self.declare_parameter("max_points", 5000)
        self.declare_parameter("rate_hz", 2.0)
        self.min_step = self.get_parameter("min_step").value
        self.max_points = self.get_parameter("max_points").value

        self.points = []
        self.create_subscription(NavSatFix, self.get_parameter("gps_topic").value, self.on_fix, 10)
        self.pub = self.create_publisher(GeoJSON, "gps_track", 10)
        self.timer = self.create_timer(
            1.0 / self.get_parameter("rate_hz").value, self.publish_track
        )

    def on_fix(self, msg):
        """Add the fix to the track once the boat has moved at least min_step."""
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        point = (msg.longitude, msg.latitude)  # GeoJSON order is lon, lat
        if self.points and distance(self.points[-1], point) < self.min_step:
            return
        self.points.append(point)
        del self.points[: -self.max_points]

    def publish_track(self):
        """Publish the track so far as a GeoJSON LineString."""
        if len(self.points) < 2:
            return
        feature = {
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": self.points},
            "properties": {"name": "boat track", "style": {"color": "#ff3b30", "weight": 4}},
        }
        self.pub.publish(GeoJSON(geojson=json.dumps(feature)))


def main(args=None):
    """Start the GPS track node and spin until interrupted."""
    rclpy.init(args=args)
    node = GpsTrack()
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
