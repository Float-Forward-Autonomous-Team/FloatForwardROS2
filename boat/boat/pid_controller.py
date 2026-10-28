"""PID controller node for vessel heading and velocity tracking."""

import math
import time

from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64


class PID:
    """Discrete PID controller with integral anti-windup."""

    def __init__(self, kp, ki, kd, integral_limit):
        """Initialize PID gains and state variables."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def reset(self):
        """Reset the PID controller state."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def compute(self, error, feedforward=0.0):
        """Compute the PID control effort."""
        current_time = time.time()
        dt = current_time - self.prev_time

        if dt <= 0.0 or dt > 0.5:
            dt = 0.01

        p_term = self.kp * error

        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i_term = self.ki * self.integral

        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        self.prev_error = error
        self.prev_time = current_time

        return feedforward + p_term + i_term + d_term


class pid_controller(Node):
    """ROS 2 node for steering and velocity PID control."""

    def __init__(self):
        """Initialize the PID controller node, subscribers, and publishers."""
        super().__init__("pid_controller")

        # Hard-coded values for testing purposes (maybe uncomment if really weird behaviour)
        # self.lat_origin = -33.724223
        # self.lon_origin = 150.679736

        self.lat_origin = None
        self.lon_origin = None

        # Initialize variables for later
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.current_yaw = 0.0

        self.target_vel = 0.0
        self.current_vel = 0.0
        self.prev_x = None
        self.prev_y = None
        self.prev_gps_time = None

        # Flags for testing purposes
        self.has_gps = False
        self.has_waypoint = False

        # Sensors
        self.waypoint = self.create_subscription(
            PoseArray, "/vrx/wayfinding/waypoints", self.waypoint_callback, 10
        )
        self.gps_data = self.create_subscription(
            NavSatFix, "/wamv/sensors/gps/gps/fix", self.gps_data_callback, 10
        )
        self.imus = self.create_subscription(
            Imu, "/wamv/sensors/imu/imu/data", self.imu_callback, 10
        )

        # CHANGE THIS TO MATCH THE WHAT YOU ARE PUBLISHING FOR VELOCITY
        self.velocity_sub = self.create_subscription(
            Float64, "/guidance/target_velocity", self.vel_callback, 10
        )

        # Actuators
        self.angle_pub = self.create_publisher(Float64, "/wamv/thrusters/main/pos", 10)
        self.thrust_pub = self.create_publisher(Float64, "/wamv/thrusters/main/thrust", 10)

        # PID instances
        self.yaw_pid = PID(kp=1.0, ki=0.1, kd=0.1, integral_limit=0.4)
        self.vel_pid = PID(kp=400.0, ki=50.0, kd=10.0, integral_limit=500.0)
        self.max_thrust = 2300.0
        self.min_thrust = -1000.0

        # Thrust to be changed with later iterations of controllers
        self.log_counter = 0

        # Testing testing 1-2-3
        self.get_logger().info("Steering PID Node initialized. Ready to track heading!")

    def imu_callback(self, msg):
        """Process IMU orientation data and publish actuator commands."""
        if not self.has_gps or not self.has_waypoint:
            return

        # Imu sensor feedback
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        target_yaw = math.atan2(dy, dx)

        raw_error = target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(raw_error), math.cos(raw_error))
        self.u_angle = -self.yaw_pid.compute(yaw_error)

        max_angle = math.radians(45.0)
        commanded_angle = max(min(self.u_angle, max_angle), -max_angle)

        angle_msg = Float64()
        angle_msg.data = float(commanded_angle)
        self.angle_pub.publish(angle_msg)

        vel_error = self.target_vel - self.current_vel
        self.u_thrust = self.vel_pid.compute(vel_error)
        commanded_thrust = max(min(self.u_thrust, self.max_thrust), self.min_thrust)

        thrust_msg = Float64()
        thrust_msg.data = float(commanded_thrust)
        self.thrust_pub.publish(thrust_msg)

        self.log_counter += 1
        if self.log_counter % 100 == 0:
            self.get_logger().info(
                f"Yaw Ref: {math.degrees(target_yaw):.1f}° | "
                f"Yaw: {math.degrees(self.current_yaw):.1f}° | "
                f"Angle Cmd: {commanded_angle:.2f} rad || "
                f"Vel Ref: {self.target_vel:.1f} m/s | "
                f"Vel: {self.current_vel:.2f} m/s | "
                f"Thrust Cmd: {commanded_thrust:.0f} N"
            )

    def gps_data_callback(self, msg):
        """Process GPS coordinates and estimate forward velocity."""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        if self.lat_origin is None:
            self.lat_origin = self.current_lat
            self.lon_origin = self.current_lon

        self.current_x = (
            (self.current_lon - self.lon_origin)
            * 111320.0
            * math.cos(math.radians(self.lat_origin))
        )
        self.current_y = (self.current_lat - self.lat_origin) * 111320.0

        now = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self.prev_gps_time is not None:
            dt = now - self.prev_gps_time
            if dt > 0.0:
                dx = self.current_x - self.prev_x
                dy = self.current_y - self.prev_y
                forward_dist = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
                raw_vel = forward_dist / dt
                self.current_vel = 0.3 * raw_vel + 0.7 * self.current_vel

        self.prev_x = self.current_x
        self.prev_y = self.current_y
        self.prev_gps_time = now

        self.has_gps = True

    def waypoint_callback(self, msg):
        """Update target waypoint coordinates and reset PID states."""
        if len(msg.poses) > 0:
            self.target_x = msg.poses[0].position.x
            self.target_y = msg.poses[0].position.y
            self.has_waypoint = True

            self.yaw_pid.reset()
            self.vel_pid.reset()

    def vel_callback(self, msg):
        """Update target velocity setpoint."""
        self.target_vel = msg.data


def main(args=None):
    """Initialize and spin the ROS 2 PID controller node."""
    rclpy.init(args=args)
    node = pid_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
