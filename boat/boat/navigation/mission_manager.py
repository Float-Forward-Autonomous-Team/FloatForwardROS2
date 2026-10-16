"""Mission manager: sends tasks one at a time and moves on when each is reported done."""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

# One wayfinding demo goal in the local "map" frame (metres, x east, y north).
# Later the mission can come from a scenario or config file.
MISSION = [
    {"type": "goto", "x": -593.0, "y": 241.0},
]


class MissionManager(Node):
    """Steps through MISSION: IDLE -> RUNNING task i -> ... -> COMPLETE (or ABORTED)."""

    def __init__(self):
        """Set up topics and the periodic status timer, then start the mission."""
        super().__init__("mission_manager")
        self.declare_parameter("task_timeout_sec", 120.0)

        self.goal_pub = self.create_publisher(PoseStamped, "/mission/goal", 10)
        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.create_subscription(Bool, "/mission/task_done", self.on_task_done, 10)

        self.tasks = MISSION
        self.index = 0
        self.state = "IDLE"
        self.task_started = None
        self.timer = self.create_timer(1.0, self.tick)
        self.start_task()

    def start_task(self):
        """Begin the current task, or finish the mission if there are none left."""
        if self.index >= len(self.tasks):
            self.set_state("COMPLETE")
            return
        self.task_started = self.get_clock().now()
        self.set_state("RUNNING")
        self.get_logger().info(
            f"task {self.index + 1}/{len(self.tasks)}: {self.tasks[self.index]}"
        )

    def on_task_done(self, msg):
        """Advance on success, abort on failure; ignore reports when nothing is running."""
        if self.state != "RUNNING":
            return
        if not msg.data:
            self.set_state("ABORTED")
            return
        self.index += 1
        self.start_task()

    def tick(self):
        """Check the timeout and republish the goal so late-starting nodes still get it."""
        if self.state == "RUNNING":
            elapsed = (self.get_clock().now() - self.task_started).nanoseconds / 1e9
            if elapsed > self.get_parameter("task_timeout_sec").value:
                self.get_logger().warn(f"task {self.index + 1} timed out after {elapsed:.0f}s")
                self.set_state("ABORTED")
            else:
                self.publish_goal(self.tasks[self.index])
        self.publish_state()

    def publish_goal(self, task):
        """Send the current goto target."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = task["x"]
        goal.pose.position.y = task["y"]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def set_state(self, state):
        """Change state, log it and publish it immediately."""
        self.state = state
        if state in ("COMPLETE", "ABORTED"):
            self.get_logger().info(f"mission {state.lower()}")
        self.publish_state()

    def publish_state(self):
        """Publish e.g. 'RUNNING 2/3' or 'COMPLETE'."""
        text = self.state
        if self.state == "RUNNING":
            text += f" {self.index + 1}/{len(self.tasks)}"
        self.state_pub.publish(String(data=text))


def main(args=None):
    """Start the mission manager node and spin until interrupted."""
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
