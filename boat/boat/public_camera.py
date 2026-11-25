"""Turn the raw camera images into small JPEGs for the public live page.

A raw frame is about 2.7 MB, far too much to send over the internet to every viewer.
This node scales each camera down, encodes it as JPEG at a few frames per second and
publishes it as sensor_msgs/CompressedImage on /public/cameras/<name>/compressed.
A camera is only encoded while someone is subscribed to its compressed topic.
"""

import signal
from io import BytesIO

import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

CAMERAS = ["front_left", "front_right", "far_left"]
RAW_TOPIC = "/wamv/sensors/cameras/{}_camera_sensor/optical/image_raw"
PUBLIC_TOPIC = "/public/cameras/{}/compressed"
# ROS encoding -> (Pillow image mode, Pillow raw mode, bytes per pixel)
ENCODINGS = {
    "rgb8": ("RGB", "RGB", 3),
    "bgr8": ("RGB", "BGR", 3),
    "rgba8": ("RGBA", "RGBA", 4),
    "mono8": ("L", "L", 1),
}


def to_jpeg(msg, width, quality):
    """Return the image as JPEG bytes no wider than width, or None for an unknown encoding."""
    if msg.encoding not in ENCODINGS:
        return None
    mode, raw_mode, _ = ENCODINGS[msg.encoding]
    img = PilImage.frombuffer(
        mode, (msg.width, msg.height), bytes(msg.data), "raw", raw_mode, msg.step, 1
    )
    if img.mode == "RGBA":
        img = img.convert("RGB")
    if img.width > width:
        img = img.resize((width, round(img.height * width / img.width)), PilImage.BILINEAR)
    out = BytesIO()
    img.save(out, format="JPEG", quality=quality)
    return out.getvalue()


class PublicCamera(Node):
    """Republishes each camera as a small, rate-limited JPEG stream."""

    def __init__(self):
        """Subscribe to every camera and create its public publisher."""
        super().__init__("public_camera")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("quality", 70)
        self.period_ns = int(1e9 / self.get_parameter("rate_hz").value)
        self.width = self.get_parameter("width").value
        self.quality = self.get_parameter("quality").value

        self.last_ns = {}
        self.pubs = {}
        for name in CAMERAS:
            self.pubs[name] = self.create_publisher(CompressedImage, PUBLIC_TOPIC.format(name), 1)
            self.create_subscription(
                Image,
                RAW_TOPIC.format(name),
                lambda msg, name=name: self.on_image(name, msg),
                qos_profile_sensor_data,
            )

    def on_image(self, name, msg):
        """Encode and publish the frame if someone is watching and the rate allows it."""
        pub = self.pubs[name]
        if pub.get_subscription_count() == 0:
            return
        now = self.get_clock().now().nanoseconds
        if now - self.last_ns.get(name, 0) < self.period_ns:
            return
        jpeg = to_jpeg(msg, self.width, self.quality)
        if jpeg is None:
            self.get_logger().warn(f"{name}: unsupported encoding {msg.encoding}", once=True)
            return
        self.last_ns[name] = now
        out = CompressedImage(header=msg.header, format="jpeg")
        out.data = jpeg
        pub.publish(out)


def main(args=None):
    """Start the public camera node and spin until interrupted."""
    rclpy.init(args=args)
    node = PublicCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch forwards Ctrl+C on top of the terminal's own; see gps_track.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
