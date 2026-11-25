"""Drive the WAM-V in circles: constant engine thrust with a fixed steering angle.

A placeholder race stage until real navigation exists. Thrust is in newtons
(full throttle is boat_params.yaml's max_thrust); steering is the engine angle in
radians (see documentation/ROS_CHEATSHEET.md for the sign).

VRX spawns the boat attached to an anchored platform and only lets go when a task
starts, so worlds without a task (sydney_regatta) keep it pinned. This node sends
the release itself, which does nothing if the boat is already free.
"""

import signal

import rclpy
from gz.msgs10.empty_pb2 import Empty
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

RELEASE_TOPIC = "/vrx/release"


class CircleDriver(Node):
    """Publishes the same thrust and steering command at a steady rate."""

    def __init__(self):
        """Set up the engine publishers, the platform release and the command timer."""
        super().__init__("circle_driver")
        self.declare_parameter("thrust", 600.0)
        self.declare_parameter("steer", 0.3)
        self.declare_parameter("rate_hz", 10.0)
        self.thrust = self.get_parameter("thrust").value
        self.steer = self.get_parameter("steer").value
        rate = self.get_parameter("rate_hz").value

        self.thrust_pub = self.create_publisher(Float64, "/wamv/thrusters/main/thrust", 10)
        self.steer_pub = self.create_publisher(Float64, "/wamv/thrusters/main/pos", 10)

        # Gazebo needs a moment to discover a new publisher, so repeat the release
        # for the first couple of seconds instead of sending it once.
        self.gz = GzNode()
        self.release_pub = self.gz.advertise(RELEASE_TOPIC, Empty)
        self.release_ticks = int(2 * rate)

        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"driving in circles: thrust={self.thrust} N, steer={self.steer} rad"
        )

    def tick(self):
        """Send the engine command again; the thruster holds only the latest value."""
        if self.release_ticks > 0:
            self.release_pub.publish(Empty())
            self.release_ticks -= 1
        self.send(self.thrust, self.steer)

    def send(self, thrust, steer):
        """Publish one thrust and one steering command."""
        self.thrust_pub.publish(Float64(data=thrust))
        self.steer_pub.publish(Float64(data=steer))


def main(args=None):
    """Start the circle driver; on Ctrl+C or SIGTERM, stop the engine before exiting."""
    # Handle signals ourselves: rclpy's default handler shuts the context down
    # first, which would make the final stop command impossible to publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = CircleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch forwards Ctrl+C on top of the terminal's own, so a second
        # SIGINT can land mid-cleanup; ignore it so shutdown finishes quietly.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.send(0.0, 0.0)
        node.get_logger().info("engine stopped")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
