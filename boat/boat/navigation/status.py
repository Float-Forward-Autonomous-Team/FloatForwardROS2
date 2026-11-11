"""Progress reports that each navigation layer sends to the layer above it."""

import math

from boat_interfaces.msg import NavigationStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# Latched, so a node that starts late still receives the last report.
STATUS_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

STATE_NAMES = {
    NavigationStatus.IDLE: "IDLE",
    NavigationStatus.ACTIVE: "ACTIVE",
    NavigationStatus.SUCCEEDED: "SUCCEEDED",
    NavigationStatus.BLOCKED: "BLOCKED",
}


def same_goal(point, xy, tolerance):
    """Return whether a report's goal point lies within tolerance of an (x, y) goal."""
    return xy is not None and math.dist((point.x, point.y), xy) <= tolerance


class StatusReporter:
    """Publish a layer's status, only when the state, goal or detail changes."""

    def __init__(self, node, topic, frame_id="map"):
        """Create the latched status publisher on the given node."""
        self.node = node
        self.frame_id = frame_id
        self.publisher = node.create_publisher(NavigationStatus, topic, STATUS_QOS)
        self.last = None

    def report(self, state, goal=None, detail=""):
        """Publish the status if it differs from the last one; return whether it did."""
        key = (state, goal, detail)
        if key == self.last:
            return False
        self.last = key
        message = NavigationStatus()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.state = state
        if goal is not None:
            message.goal.x, message.goal.y = float(goal[0]), float(goal[1])
        message.detail = detail
        self.publisher.publish(message)
        return True
