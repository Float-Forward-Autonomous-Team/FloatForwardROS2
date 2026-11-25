"""Run one test slalom: place the course, drive it on GPS and IMU, and show it in Foxglove.

The course is built ahead of the boat and, in simulation, its buoys are put into Gazebo.
The boat follows a precomputed line through the known buoy positions; it does not look
for the buoys with its cameras or lidar yet. Times are in simulation time.
"""

import json
import math
import signal

import rclpy
from foxglove_msgs.msg import GeoJSON
from geometry_msgs.msg import Point, TransformStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix
from std_msgs.msg import Float64, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from boat.scoreboard import draw_scoreboard
from boat.slalom_course import Anchor, Referee, build_course, hits_boat, pursue, wrap

MAP_FRAME = "race_map"
BOAT_FRAME = "wamv/wamv/base_link"
KNOTS = 1.943844  # per m/s
WHITE, GREEN = (0.95, 0.95, 0.95), (0.1, 0.75, 0.25)
BLUE, RED, YELLOW = (0.2, 0.5, 1.0), (0.95, 0.2, 0.2), (1.0, 0.8, 0.1)

WAITING, COUNTDOWN, RACING, FINISHED = "WAITING", "COUNTDOWN", "RACING", "FINISHED"


class SlalomRace(Node):
    """Places the course, steers the boat, scores the run and publishes it all."""

    def __init__(self):
        """Read the parameters and build the course; the boat waits for GPS and IMU."""
        super().__init__("slalom_race")
        defaults = {
            "buoy_count": 4,
            "buoy_spacing": 22.0,
            "first_buoy": 20.0,
            "start_line": 8.0,
            "clearance": 4.0,
            "course_bearing_deg": 0.0,
            "buoy_radius": 0.35,
            "thrust": 1000.0,
            "lookahead": 6.0,
            "heading_gain": 1.2,
            "yaw_rate_gain": 0.6,
            "max_steer_deg": 40.0,
            "start_delay": 3.0,
            "sensor_timeout": 2.0,
            "gps_x": -0.85,
            "boat_length": 4.9,
            "boat_width": 2.4,
            "spawn_buoys": True,
        }
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.course = build_course(
            self.p["buoy_count"],
            self.p["buoy_spacing"],
            self.p["first_buoy"],
            self.p["start_line"],
            self.p["clearance"],
        )
        self.referee = Referee(self.course.gates)
        self.state = WAITING
        self.anchor = None
        self.fix = self.imu = None
        self.fix_time = self.imu_time = -math.inf
        self.release_at = None
        self.release_ticks = 0
        self.path_index = 0
        self.x = self.y = self.yaw = self.speed = 0.0
        self.last_fix_xy = None
        self.touched = set()
        self.stale = False
        self.track = []

        self.gazebo = None  # connected once the world exists, see place_course

        self.thrust_pub = self.create_publisher(Float64, "/wamv/thrusters/main/thrust", 10)
        self.steer_pub = self.create_publisher(Float64, "/wamv/thrusters/main/pos", 10)
        self.state_pub = self.create_publisher(String, "race/state", 10)
        self.result_pub = self.create_publisher(
            String, "race/result", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.marker_pub = self.create_publisher(MarkerArray, "race/markers", 10)
        self.geo_pub = self.create_publisher(GeoJSON, "race/course", 10)
        self.board_pub = self.create_publisher(CompressedImage, "race/scoreboard", 1)
        self.board = None  # what the scoreboard shows, see publish_status
        names = (
            "elapsed",
            "speed_kn",
            "heading_deg",
            "next_gate_m",
            "passed",
            "missed",
            "touched",
        )
        self.numbers = {n: self.create_publisher(Float64, f"race/{n}", 10) for n in names}
        self.tf = TransformBroadcaster(self)

        self.create_subscription(
            NavSatFix, "/wamv/sensors/gps/gps/fix", self.on_fix, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, "/wamv/sensors/imu/imu/data", self.on_imu, qos_profile_sensor_data
        )
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.publish_course)
        self.create_timer(0.5, self.publish_scoreboard)
        # On wall time: simulation time stands still until the world has loaded.
        self.create_timer(5.0, self.report_waiting, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("waiting for GPS, IMU and the boat's position")

    def now(self):
        """Return the current (simulation) time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg):
        """Keep the latest heading (0 = east, counter-clockwise) and turn rate."""
        q = msg.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        self.imu = msg
        self.imu_time = self.now()

    def on_fix(self, msg):
        """Turn a GPS fix into the boat's position, speed and score."""
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self.fix = msg
        self.fix_time = self.now()
        if self.anchor is None or self.imu is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.x, self.y = self.boat_position(msg)
        if self.last_fix_xy is not None:
            x0, y0, t0 = self.last_fix_xy
            if stamp - t0 >= 0.5:
                self.speed = math.hypot(self.x - x0, self.y - y0) / (stamp - t0)
                self.last_fix_xy = (self.x, self.y, stamp)
        else:
            self.last_fix_xy = (self.x, self.y, stamp)
        if self.state == RACING:
            self.score(stamp)

    def boat_position(self, fix):
        """Return the course position of the boat's centre from a fix of its antenna."""
        x, y = self.anchor.to_course(fix.latitude, fix.longitude)
        heading = self.yaw - self.anchor.yaw
        return x - self.p["gps_x"] * math.cos(heading), y - self.p["gps_x"] * math.sin(heading)

    def score(self, stamp):
        """Pass the new position to the referee and check for buoy touches."""
        for gate in self.referee.update(self.x, self.y, stamp):
            verdict = "MISSED" if gate.name in self.referee.missed else "ok"
            self.get_logger().info(f"{self.referee.elapsed(stamp):6.1f} s  {gate.name}: {verdict}")
        heading = self.yaw - self.anchor.yaw
        for i, buoy in enumerate(self.course.buoys):
            if i not in self.touched and hits_boat(
                self.x,
                self.y,
                heading,
                buoy,
                self.p["buoy_radius"],
                self.p["boat_length"],
                self.p["boat_width"],
            ):
                self.touched.add(i)
                self.get_logger().warning(f"touched buoy {i + 1}")
        if self.referee.finished:
            self.finish(stamp)

    def finish(self, stamp):
        """Stop the boat and report the result."""
        self.state = FINISHED
        result = {
            "time_s": round(self.referee.elapsed(stamp), 2),
            "gates_passed": self.referee.passed,
            "gates_missed": self.referee.missed,
            "buoys_touched": sorted(i + 1 for i in self.touched),
        }
        self.result_pub.publish(String(data=json.dumps(result)))
        self.get_logger().info(f"finished: {json.dumps(result)}")

    def place_course(self):
        """Put the course ahead of the boat as soon as everything needed is known."""
        if self.p["spawn_buoys"] and self.gazebo is None:
            from boat.gz_buoys import GazeboCourse  # not available on a real boat

            try:
                self.gazebo = GazeboCourse()
            except RuntimeError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        if self.fix is None or self.imu is None:
            return
        if self.gazebo is not None and self.gazebo.boat is None:
            return
        bearing = math.radians(self.p["course_bearing_deg"])
        # The GPS antenna is off-centre: start from the boat's centre instead.
        rough = Anchor(self.fix.latitude, self.fix.longitude, self.yaw)
        lat, lon = rough.to_gps(-self.p["gps_x"], 0.0)
        self.anchor = Anchor(lat, lon, self.yaw + bearing)
        if self.gazebo is not None and not self.spawn_buoys():
            self.anchor = None
            raise RuntimeError("Gazebo refused the race buoys")
        self.release_at = self.now() + self.p["start_delay"]
        self.state = COUNTDOWN
        self.get_logger().info(f"course placed at {lat:.6f}, {lon:.6f}")

    def report_waiting(self):
        """Say what the race is still waiting for."""
        if self.state != WAITING:
            return
        missing = [n for n, v in (("GPS", self.fix), ("IMU", self.imu)) if v is None]
        if missing:
            self.get_logger().info(
                f"waiting for {' and '.join(missing)}: is the boat in the simulator yet? "
                "(the first start after `make down` downloads the world)"
            )

    def spawn_buoys(self):
        """Put the buoys into Gazebo around the boat's true position; True if all worked."""
        x0, y0 = self.gazebo.boat
        self.gazebo.remove_old()
        ok = True
        for i, (x, y, is_end) in enumerate(self.course.buoys):
            east, north = self.anchor.to_enu(x, y)
            color = GREEN if is_end else WHITE
            ok &= self.gazebo.spawn(
                str(i + 1), x0 + east, y0 + north, self.p["buoy_radius"], color
            )
        return ok

    def tick(self):
        """Run the race state machine at 10 Hz and publish the dashboard values."""
        now = self.now()
        if self.state == WAITING:
            self.place_course()
        elif self.state == COUNTDOWN and now >= self.release_at:
            self.state = RACING
            self.release_ticks = 20
        if self.state == RACING:
            if self.gazebo is not None and self.release_ticks > 0:
                # Repeated, because Gazebo needs a moment to notice the publisher.
                self.gazebo.release()
                self.release_ticks -= 1
            self.drive(now)
        else:
            self.send(0.0, 0.0)
        self.publish_status(now)

    def drive(self, now):
        """Steer toward the line; stop the engine while GPS or IMU is missing."""
        stale = now - min(self.fix_time, self.imu_time) > self.p["sensor_timeout"]
        if stale != self.stale:
            self.stale = stale
            self.get_logger().warning("sensors lost, engine stopped" if stale else "sensors back")
        if stale:
            self.send(0.0, 0.0)
            return
        self.path_index, (tx, ty) = pursue(
            self.course.path, self.path_index, self.x, self.y, self.p["lookahead"]
        )
        heading = self.yaw - self.anchor.yaw
        error = wrap(math.atan2(ty - self.y, tx - self.x) - heading)
        turn = self.p["heading_gain"] * error - (
            self.p["yaw_rate_gain"] * self.imu.angular_velocity.z
        )
        limit = math.radians(self.p["max_steer_deg"])
        # A positive engine angle turns the boat to starboard, so the sign flips.
        self.send(self.p["thrust"], -max(-limit, min(limit, turn)))

    def send(self, thrust, steer):
        """Publish one thrust (N) and one engine angle (rad)."""
        self.thrust_pub.publish(Float64(data=float(thrust)))
        self.steer_pub.publish(Float64(data=float(steer)))

    def publish_status(self, now):
        """Publish the state, the dashboard numbers and the boat's place on the course."""
        gate = self.referee.next_gate()
        label = self.state
        if self.state == RACING and gate is not None:
            label = f"{RACING}: next {gate.name}" if self.referee.started else "RACING: to start"
            if self.stale:
                label = "STOPPED: no GPS or IMU"
        self.state_pub.publish(String(data=label))
        values = {
            "elapsed": self.referee.elapsed(now),
            "speed_kn": self.speed * KNOTS,
            "heading_deg": (90.0 - math.degrees(self.yaw)) % 360.0,
            "next_gate_m": abs(gate.x - self.x) if gate is not None else 0.0,
            "passed": self.referee.passed,
            "missed": len(self.referee.missed),
            "touched": len(self.touched),
        }
        for name, value in values.items():
            self.numbers[name].publish(Float64(data=float(value)))
        self.board = (label, values)
        if self.anchor is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = MAP_FRAME, BOAT_FRAME
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        half = (self.yaw - self.anchor.yaw) / 2.0
        t.transform.rotation.z, t.transform.rotation.w = math.sin(half), math.cos(half)
        self.tf.sendTransform(t)
        if self.state == RACING and (
            not self.track or math.dist(self.track[-1], (self.x, self.y)) > 0.5
        ):
            self.track.append((self.x, self.y))

    def publish_scoreboard(self):
        """Publish the race state and numbers as one picture, twice a second."""
        if self.board is None:
            return
        gates = sum(1 for g in self.course.gates if g.side != 0)  # start and finish do not count
        msg = CompressedImage(format="jpeg")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "race_scoreboard"
        msg.data = draw_scoreboard(*self.board, gates)
        self.board_pub.publish(msg)

    def publish_course(self):
        """Publish the buoys, lines and track for the 3D and Map panels, once a second."""
        if self.anchor is None:
            return
        start = self.p["start_line"]
        gate = self.referee.next_gate()
        markers = []
        for i, (x, y, is_end) in enumerate(self.course.buoys):
            color = RED if i in self.touched else GREEN if is_end else WHITE
            m = self.marker("buoys", i, Marker.CYLINDER, color)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.45
            m.scale.x = m.scale.y = 2.0 * self.p["buoy_radius"]
            m.scale.z = 1.3
            markers.append(m)
        markers.append(self.line("planned", self.course.path, BLUE, 0.15))
        markers.append(self.line("start", [(start, -6.0), (start, 6.0)], YELLOW, 0.3))
        if len(self.track) > 1:
            markers.append(self.line("track", self.track, RED, 0.25))
        m = self.line("next", [(gate.x, -5.0), (gate.x, 5.0)] if gate else [], GREEN, 0.2)
        if gate is None:
            m.action = Marker.DELETE
        markers.append(m)
        self.marker_pub.publish(MarkerArray(markers=markers))
        self.geo_pub.publish(GeoJSON(geojson=json.dumps(self.geojson(start))))

    def marker(self, ns, i, kind, color):
        """Return a marker in the course frame."""
        m = Marker()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, i, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        return m

    def line(self, ns, points, color, width):
        """Return a line marker through the points."""
        m = self.marker(ns, 0, Marker.LINE_STRIP, color)
        m.scale.x = width
        m.points = [Point(x=float(x), y=float(y), z=0.1) for x, y in points]
        return m

    def geojson(self, start):
        """Return the course as GeoJSON features for the Map panel."""

        def lonlat(points):
            return [tuple(reversed(self.anchor.to_gps(x, y))) for x, y in points]

        def feature(kind, coordinates, name, color):
            return {
                "type": "Feature",
                "geometry": {"type": kind, "coordinates": coordinates},
                "properties": {"name": name, "style": {"color": color, "weight": 3}},
            }

        features = [
            feature("LineString", lonlat(self.course.path[::4]), "planned line", "#3380ff"),
            feature("LineString", lonlat([(start, -6.0), (start, 6.0)]), "start", "#ffcc1a"),
        ]
        for i, (x, y, is_end) in enumerate(self.course.buoys):
            name, color = ("end buoy", "#1abf40") if is_end else (f"buoy {i + 1}", "#ff7f0e")
            features.append(feature("Point", lonlat([(x, y)])[0], name, color))
        return {"type": "FeatureCollection", "features": features}


def main(args=None):
    """Run the race; on Ctrl+C or SIGTERM, stop the engine before exiting."""
    # Handle signals ourselves: rclpy's default handler shuts the context down
    # first, which would make the final stop command impossible to publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = SlalomRace()
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
