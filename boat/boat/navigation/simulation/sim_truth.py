"""Stand in for Localization and Perception by publishing Gazebo ground truth.

Topics match what the real modules will provide, so everything downstream
(world model, planners) runs unchanged when they replace this node.
"""

import math
import queue
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from boat_interfaces.msg import TrackedObject, TrackedObjectArray
from geometry_msgs.msg import Vector3Stamped
from gz.msgs10.navsat_pb2 import NavSat
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from boat.navigation.simulation.obstacle_trajectory import load_obstacles


def _stamp(source):
    """Copy a Gazebo protobuf stamp into a ROS Time message."""
    from builtin_interfaces.msg import Time

    stamp = Time()
    stamp.sec = source.sec
    stamp.nanosec = source.nsec
    return stamp


def _seconds(source):
    return source.sec + source.nsec * 1e-9


class SimTruth(Node):
    """Bridge Gazebo truth to the Localization and Perception topics."""

    def __init__(self):
        """Read the scenario, subscribe to Gazebo, and create ROS publishers."""
        super().__init__("sim_truth")
        self.declare_parameter("world_sdf", "/ws/worlds/wayfinding_task.sdf")
        self.declare_parameter("boat_model", "wamv")
        self.declare_parameter("obstacles_file", "")

        def param(name):
            return self.get_parameter(name).value

        self.world_sdf = Path(param("world_sdf"))
        self.world_name = ET.parse(self.world_sdf).getroot().find("world").get("name")
        self.boat_model = param("boat_model")
        self.obstacles = load_obstacles(param("obstacles_file"))
        self.get_logger().info(
            f"Reporting {self.boat_model} in {self.world_name} and "
            f"{len(self.obstacles)} trajectory obstacles"
        )

        self.odom_pub = self.create_publisher(Odometry, "/localization/odometry", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "/localization/gps_fix", 10)
        self.velocity_pub = self.create_publisher(Vector3Stamped, "/localization/gps_velocity", 10)
        self.tracks_pub = self.create_publisher(TrackedObjectArray, "/perception/objects", 10)

        self.gz = GazeboNode()
        self.pose_topic = f"/world/{self.world_name}/pose/info"
        self.pose_queue = queue.Queue(maxsize=2)
        self.navsat_queue = queue.Queue(maxsize=2)
        self.gz.subscribe(Pose_V, self.pose_topic, self._enqueue_pose)
        self.navsat_topic = None
        self.poses = {}
        self.last_boat = None
        self.last_track_sample = {}
        self.create_timer(0.1, self._update_state)
        self.create_timer(0.5, self._publish_objects)
        self.create_timer(1.0, self._discover_navsat)
        self._discover_navsat()

    @staticmethod
    def _queue_latest(target, message):
        try:
            target.put_nowait(message)
        except queue.Full:
            try:
                target.get_nowait()
            except queue.Empty:
                pass
            target.put_nowait(message)

    def _enqueue_pose(self, message):
        self._queue_latest(self.pose_queue, message)

    def _enqueue_navsat(self, message):
        self._queue_latest(self.navsat_queue, message)

    def _discover_navsat(self):
        if self.navsat_topic:
            return
        match = f"/model/{self.boat_model}/"
        for topic in self.gz.topic_list():
            if match in topic and topic.endswith("/navsat"):
                self.gz.subscribe(NavSat, topic, self._enqueue_navsat)
                self.navsat_topic = topic
                self.get_logger().info(f"Using Gazebo GNSS truth topic {topic}")
                return

    def _update_state(self):
        while not self.pose_queue.empty():
            message = self.pose_queue.get_nowait()
            for pose in message.pose:
                self.poses[pose.name] = (pose, message.header.stamp)
                if pose.name == self.boat_model:
                    self._publish_odometry(pose, message.header.stamp)
        while not self.navsat_queue.empty():
            self._publish_navsat(self.navsat_queue.get_nowait())

    def _publish_odometry(self, pose, stamp):
        message = Odometry()
        message.header.stamp = _stamp(stamp)
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = pose.position.x
        message.pose.pose.position.y = pose.position.y
        message.pose.pose.position.z = pose.position.z
        message.pose.pose.orientation.x = pose.orientation.x
        message.pose.pose.orientation.y = pose.orientation.y
        message.pose.pose.orientation.z = pose.orientation.z
        message.pose.pose.orientation.w = pose.orientation.w
        current_time = _seconds(stamp)
        if self.last_boat:
            previous_pose, previous_time = self.last_boat
            dt = current_time - previous_time
            if 0 < dt < 1.0:
                yaw = math.atan2(
                    2
                    * (
                        pose.orientation.w * pose.orientation.z
                        + pose.orientation.x * pose.orientation.y
                    ),
                    1 - 2 * (pose.orientation.y**2 + pose.orientation.z**2),
                )
                dx = (pose.position.x - previous_pose.position.x) / dt
                dy = (pose.position.y - previous_pose.position.y) / dt
                message.twist.twist.linear.x = math.cos(yaw) * dx + math.sin(yaw) * dy
                message.twist.twist.linear.y = -math.sin(yaw) * dx + math.cos(yaw) * dy
                message.twist.twist.linear.z = (pose.position.z - previous_pose.position.z) / dt
        previous = GzPose()
        previous.CopyFrom(pose)
        self.last_boat = previous, current_time
        self.odom_pub.publish(message)

    def _publish_navsat(self, navsat):
        fix = NavSatFix()
        fix.header.stamp = _stamp(navsat.header.stamp)
        fix.header.frame_id = navsat.frame_id or "gps"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = navsat.latitude_deg
        fix.longitude = navsat.longitude_deg
        fix.altitude = navsat.altitude
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(fix)

        velocity = Vector3Stamped()
        velocity.header.stamp = fix.header.stamp
        velocity.header.frame_id = "map"
        velocity.vector.x = navsat.velocity_east
        velocity.vector.y = navsat.velocity_north
        velocity.vector.z = navsat.velocity_up
        self.velocity_pub.publish(velocity)

    def _publish_objects(self):
        timestamp = self.get_clock().now().to_msg()
        current_time = timestamp.sec + timestamp.nanosec * 1e-9
        tracks = TrackedObjectArray()
        tracks.header.stamp = timestamp
        tracks.header.frame_id = "map"
        for obstacle in self.obstacles:
            observed = self.poses.get(obstacle.name)
            if observed is None:
                continue
            pose, _ = observed
            track = TrackedObject()
            track.id = obstacle.name
            track.classification = obstacle.classification
            track.radius_m = obstacle.radius_m
            track.pose.position.x = pose.position.x
            track.pose.position.y = pose.position.y
            track.pose.position.z = pose.position.z
            track.pose.orientation.x = pose.orientation.x
            track.pose.orientation.y = pose.orientation.y
            track.pose.orientation.z = pose.orientation.z
            track.pose.orientation.w = pose.orientation.w
            previous = self.last_track_sample.get(obstacle.name)
            if previous and 0 < current_time - previous[0] < 2.0:
                dt = current_time - previous[0]
                track.twist.linear.x = (pose.position.x - previous[1]) / dt
                track.twist.linear.y = (pose.position.y - previous[2]) / dt
            self.last_track_sample[obstacle.name] = (
                current_time,
                pose.position.x,
                pose.position.y,
            )
            tracks.objects.append(track)
        self.tracks_pub.publish(tracks)


def main(args=None):
    """Run the simulator truth publisher."""
    rclpy.init(args=args)
    node = SimTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
