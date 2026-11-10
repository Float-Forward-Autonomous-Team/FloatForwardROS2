"""Turns the water mask into the distance to the nearest obstacle in each direction, as a LaserScan.

For each column of the water mask it takes the first pixel above the bottom of the image that is not
water: where an obstacle meets the water. The camera's lens data (camera_info), its height above the
water and its tilt, plus the boat's roll and pitch from the IMU, turn that pixel into a point on the
water: a direction and a distance. A column that is water all the way up to the sky is free.

The result goes out as sensor_msgs/LaserScan on /perception/water_scan, so the planner can read it like
a LiDAR scan: a second obstacle source that does not depend on the LiDAR.

The distance comes from where the pixel's ray meets a flat water surface, so it gets less accurate
with distance (at 1.6 m camera height and 80 degrees field of view, one pixel at 30 m is about 0.6 m),
and it is only as good as camera_height and camera_pitch_deg. It sees only what the camera sees.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

WATER, SKY = 1, 2  # classes in the water mask: 0 obstacle, 1 water, 2 sky


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def roll_pitch(q):
    """Roll and pitch (radians) from an orientation quaternion. Pitch is positive nose down (ROS convention)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def camera_to_level(mount_rpy, boat_rp):
    """Rotation from camera axes (x forward, y left, z up) to a level frame that turns with the boat's heading.

    mount_rpy: the camera's roll, pitch (positive = tilted down) and yaw on the boat, in radians.
    boat_rp: the boat's roll and pitch from the IMU."""
    mount = rot_z(mount_rpy[2]) @ rot_y(mount_rpy[1]) @ rot_x(mount_rpy[0])
    return rot_y(boat_rp[1]) @ rot_x(boat_rp[0]) @ mount


def rays(u, v, fx, fy, cx, cy, rotation):
    """Viewing direction of pixels (u, v) in the level frame (x forward, y left, z up)."""
    x_opt, y_opt = (u - cx) / fx, (v - cy) / fy  # optical frame: x right, y down, z forward
    body = np.stack([np.ones_like(x_opt), -x_opt, -y_opt])  # camera axes: x forward, y left, z up
    return rotation @ body


def edge_rows(labels, ignore_rows):
    """Per column: the row of the first non-water pixel going up from the bottom (above the ignored rows),
    or -1 where the column is water all the way to the top."""
    h = labels.shape[0] - ignore_rows
    if h <= 0:
        return np.full(labels.shape[1], -1)
    nonwater = labels[:h][::-1] != WATER
    found = nonwater.any(axis=0)
    return np.where(found, h - 1 - nonwater.argmax(axis=0), -1)


def compute_scan(labels, intrinsics, camera_height, mount_rpy, boat_rp, camera_xy, angle_min, angle_max,
                 angle_increment, max_range, ignore_bottom, column_step):
    """The scan for one water mask. Returns (ranges, details).

    ranges: one value per angle from angle_min to angle_max (radians, positive to port), as in a LaserScan:
    the distance in metres to the nearest obstacle, +inf where the camera sees free water up to the horizon
    (or the obstacle is beyond max_range), nan where no column of the image looks in that direction.
    Distances and angles are from camera_xy, the camera's position on the boat (x forward, y to port), so
    they are relative to the boat's origin."""
    fx, fy, cx, cy, info_w, info_h = intrinsics
    mask_h, mask_w = labels.shape
    sx, sy = info_w / mask_w, info_h / mask_h  # the mask may be smaller than the camera image
    ignore_rows = int(round(ignore_bottom * mask_h))
    cols = np.arange(0, mask_w, max(1, column_step))
    rows = edge_rows(labels[:, cols], ignore_rows)
    edge_class = np.where(rows >= 0, labels[np.clip(rows, 0, None), cols], WATER)

    # where to look in each column: the boundary between the obstacle pixel and the water below it;
    # for a free column, the lowest water pixel (only its direction is used). Camera pixel coordinates
    # as in camera_info (OpenCV): the centre of pixel (0, 0) is at (0, 0).
    bottom = mask_h - ignore_rows - 1
    v = np.where(rows >= 0, rows + 1.0, bottom + 0.5) * sy - 0.5
    u = (cols + 0.5) * sx - 0.5
    d = rays(u, v, fx, fy, cx, cy, camera_to_level(mount_rpy, boat_rp))

    below = d[2] < -1e-9  # rays above the horizon never reach the water
    t = np.where(below, camera_height / np.where(below, -d[2], 1.0), np.inf)
    gx = t * d[0] + camera_xy[0]
    gy = t * d[1] + camera_xy[1]
    rng = np.hypot(gx, gy)
    bearing = np.arctan2(np.where(below, gy, d[1]), np.where(below, gx, d[0]))

    obstacle = (rows >= 0) & (edge_class != SKY) & below & (rng <= max_range)
    n = int(round((angle_max - angle_min) / angle_increment)) + 1
    bins = np.floor((bearing - angle_min) / angle_increment + 0.5).astype(int)
    inside = (bins >= 0) & (bins < n)

    ranges = np.full(n, np.nan)
    ranges[bins[inside & ~obstacle]] = np.inf
    nearest_per_bin = np.full(n, np.inf)
    np.minimum.at(nearest_per_bin, bins[inside & obstacle], rng[inside & obstacle])
    hit = np.isfinite(nearest_per_bin)
    ranges[hit] = nearest_per_bin[hit]

    details = {"columns": len(cols), "obstacle_columns": int(obstacle.sum()),
               "bottom_obstacle_share": float(np.mean(labels[bottom, :] != WATER)) if bottom >= 0 else 0.0}
    if hit.any():
        i = int(np.nanargmin(np.where(hit, ranges, np.nan)))
        details["nearest"] = (float(ranges[i]), math.degrees(angle_min + i * angle_increment))
    return ranges, details


class WaterScan(Node):
    """Water mask + camera lens data + IMU -> LaserScan with the distance to the nearest obstacle per direction."""

    def __init__(self):
        super().__init__("water_scan")
        self.declare_parameter("mask_topic", "/perception/water_mask")
        self.declare_parameter("camera_info_topic", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info")
        self.declare_parameter("imu_topic", "/wamv/sensors/imu/imu/data")
        self.declare_parameter("use_imu", True)  # false: assume the boat is level
        # where the camera sits; defaults are the VRX front_left_camera (boat/config/wamv_sensors.yaml)
        self.declare_parameter("camera_height", 1.6)  # m above the water: 1.5 above the boat's centre + ~0.1
        self.declare_parameter("camera_pitch_deg", 15.0)  # tilted down; 0 when the IMU is inside the camera
        self.declare_parameter("camera_roll_deg", 0.0)
        self.declare_parameter("camera_yaw_deg", 0.0)  # 0 = looking forward, positive = to port
        self.declare_parameter("camera_x", 0.75)  # m forward of the boat's origin
        self.declare_parameter("camera_y", 0.1)  # m to port of the boat's origin
        self.declare_parameter("frame_id", "wamv/wamv/base_link")  # frame of the published scan
        self.declare_parameter("angle_increment_deg", 0.5)
        self.declare_parameter("max_range", 60.0)  # m; obstacles farther away count as free
        self.declare_parameter("ignore_bottom", 0.0)  # share of the image height at the bottom to skip (own hull)
        self.declare_parameter("column_step", 2)  # look at every n-th column

        self.info = None  # (fx, fy, cx, cy, width, height)
        self.boat_rp = None
        self.warned_no_imu = False
        self.frames = 0
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self.on_info,
                                 qos_profile_sensor_data)
        if self.get_parameter("use_imu").value:
            self.create_subscription(Imu, self.get_parameter("imu_topic").value, self.on_imu,
                                     qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("mask_topic").value, self.on_mask, 10)
        self.pub = self.create_publisher(LaserScan, "/perception/water_scan", 10)
        self.get_logger().info(
            f"turning {self.get_parameter('mask_topic').value} into /perception/water_scan; camera "
            f"{self.get_parameter('camera_height').value} m above the water, tilted "
            f"{self.get_parameter('camera_pitch_deg').value} deg down"
        )

    def on_info(self, msg):
        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        if fx <= 0 or fy <= 0:
            self.get_logger().warn("camera_info has no lens data (fx or fy is 0)", throttle_duration_sec=10.0)
            return
        first = self.info is None
        self.info = (fx, fy, cx, cy, msg.width, msg.height)
        if first:
            fov = math.degrees(2 * math.atan(msg.width / (2 * fx)))
            self.get_logger().info(f"camera: {msg.width} x {msg.height} pixels, {fov:.0f} deg wide")

    def on_imu(self, msg):
        self.boat_rp = roll_pitch(msg.orientation)

    def angle_limits(self):
        """Fixed scan limits: the directions of the image's left and right edges, with the boat level.
        A tilted camera sees widest at the bottom of the image, so the top, middle and bottom rows count."""
        fx, fy, cx, cy, w, h = self.info
        u = np.array([-0.5, w - 0.5] * 3)
        v = np.repeat([-0.5, h / 2.0, h - 0.5], 2)
        d = rays(u, v, fx, fy, cx, cy, camera_to_level(self.mount_rpy(), (0.0, 0.0)))
        inc = math.radians(self.get_parameter("angle_increment_deg").value)
        bearings = np.arctan2(d[1], d[0])
        lo, hi = float(bearings.min()), float(bearings.max())
        return math.floor(lo / inc) * inc, math.ceil(hi / inc) * inc, inc

    def mount_rpy(self):
        return (math.radians(self.get_parameter("camera_roll_deg").value),
                math.radians(self.get_parameter("camera_pitch_deg").value),
                math.radians(self.get_parameter("camera_yaw_deg").value))

    def on_mask(self, msg):
        if self.info is None:
            self.get_logger().warn("waiting for camera_info", throttle_duration_sec=10.0)
            return
        if msg.encoding != "mono8":
            self.get_logger().warn(f"mask: unsupported encoding {msg.encoding}", throttle_duration_sec=10.0)
            return
        boat_rp = self.boat_rp
        if boat_rp is None:
            boat_rp = (0.0, 0.0)
            if self.get_parameter("use_imu").value and not self.warned_no_imu:
                self.get_logger().warn("no IMU message yet: assuming the boat is level")
                self.warned_no_imu = True

        labels = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width]
        angle_min, angle_max, inc = self.angle_limits()
        max_range = self.get_parameter("max_range").value
        ranges, details = compute_scan(
            labels, self.info, self.get_parameter("camera_height").value, self.mount_rpy(), boat_rp,
            (self.get_parameter("camera_x").value, self.get_parameter("camera_y").value),
            angle_min, angle_max, inc, max_range, self.get_parameter("ignore_bottom").value,
            self.get_parameter("column_step").value)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp  # the camera image's time
        scan.header.frame_id = self.get_parameter("frame_id").value
        scan.angle_min, scan.angle_max, scan.angle_increment = float(angle_min), float(angle_max), float(inc)
        scan.range_min, scan.range_max = 0.0, float(max_range)
        scan.ranges = [float(r) for r in ranges]
        self.pub.publish(scan)

        self.frames += 1
        near = details.get("nearest")
        text = f"nearest obstacle {near[0]:.1f} m at {near[1]:+.0f} deg" if near else "no obstacle in view"
        free = 100.0 * np.mean(np.isinf(ranges))
        self.get_logger().info(
            f"{text}, {free:.0f} % of directions free | roll {math.degrees(boat_rp[0]):+.1f}, "
            f"pitch {math.degrees(boat_rp[1]):+.1f} deg"
        )
        if self.frames == 1 and details["bottom_obstacle_share"] > 0.3:
            self.get_logger().warn(
                f"{100 * details['bottom_obstacle_share']:.0f} % of the bottom row is obstacle; if that is our own "
                "hull, raise ignore_bottom (share of the image height to skip, e.g. 0.1)"
            )


def main():
    rclpy.init()
    node = WaterScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
