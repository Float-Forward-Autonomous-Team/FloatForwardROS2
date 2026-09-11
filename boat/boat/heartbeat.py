"""Minimal example node: publishes a counter on /boat/heartbeat once a second."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Heartbeat(Node):
    """Publishes an incrementing counter so you can confirm the stack is alive."""

    def __init__(self):
        """Set up the publisher and the periodic timer."""
        super().__init__("heartbeat")
        self.declare_parameter("period_sec", 1.0)
        period = self.get_parameter("period_sec").value

        self.pub = self.create_publisher(String, "heartbeat", 10)
        self.count = 0
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(f"heartbeat up, period={period}s")

    def tick(self):
        """Publish the next counter value."""
        msg = String()
        msg.data = f"alive {self.count}"
        self.pub.publish(msg)
        self.count += 1


def main(args=None):
    """Start the heartbeat node and spin until interrupted."""
    rclpy.init(args=args)
    node = Heartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
