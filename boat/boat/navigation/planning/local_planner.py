"""Local planner placeholder: forwards paths down and status reports up, unchanged.

Its place in the chain is fixed already, so the real local planner (avoiding
moving objects, cutting a short horizon from the global path) can replace this
node without touching the global planner or guidance:

    global_planner --/planning/global_path--> local_planner --/planning/local_path--> guidance
    global_planner <--/planning/local_status-- local_planner <--/guidance/status-- guidance
"""

import rclpy
from boat_interfaces.msg import NavigationStatus
from nav_msgs.msg import Path
from rclpy.node import Node

from boat.navigation.status import STATUS_QOS

# Paths are latched too: a late subscriber gets the current one.
PATH_QOS = STATUS_QOS


class LocalPlanner(Node):
    """Pass the global path to guidance and guidance's status to the global planner."""

    def __init__(self):
        """Connect the two forwarding directions."""
        super().__init__("local_planner")
        self.path_pub = self.create_publisher(Path, "/planning/local_path", PATH_QOS)
        self.status_pub = self.create_publisher(
            NavigationStatus, "/planning/local_status", STATUS_QOS
        )
        self.create_subscription(Path, "/planning/global_path", self.path_pub.publish, PATH_QOS)
        self.create_subscription(
            NavigationStatus, "/guidance/status", self.status_pub.publish, STATUS_QOS
        )
        self.get_logger().info("Local planner placeholder started; forwarding paths and status")


def main(args=None):
    """Run the local planner until interrupted."""
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
