"""Shows segmentation and detections together in one image, for people watching and for the dashboard.

Paints the water mask from water_segmentation (/perception/water_mask) onto the detector's image with
its boxes (/perception/yolo_overlay) and publishes the result on /perception/combined_overlay. It changes
neither model and neither node: it only combines what the two nodes already publish.

The two nodes pick their frames on their own, so each detector image gets the mask with the nearest
timestamp: the same frame when there is one (on the boat, where both process every frame), otherwise
the nearest mask up to max_age seconds away. The label in the top-left corner says which.
"""

import collections

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

# same colours as water_segmentation: 0 obstacle = orange, 1 water = blue; sky (2) is left unpainted
TINT = {0: np.array([255, 120, 0], np.float32), 1: np.array([0, 90, 255], np.float32)}


class CombinedOverlay(Node):
    """Water mask + detection boxes in one picture."""

    def __init__(self):
        super().__init__("combined_overlay")
        self.declare_parameter("mask_topic", "/perception/water_mask")
        self.declare_parameter("boxes_topic", "/perception/yolo_overlay")
        self.declare_parameter("max_age", 2.0)  # s; a mask further away in time is not painted
        self.declare_parameter("alpha", 0.4)  # strength of the segmentation colours, 0 to 1

        mask_topic = self.get_parameter("mask_topic").value
        boxes_topic = self.get_parameter("boxes_topic").value
        self.masks = collections.deque(maxlen=30)  # (timestamp, labels) of the latest masks
        self.create_subscription(Image, mask_topic, self.on_mask, 10)
        self.create_subscription(Image, boxes_topic, self.on_boxes, 10)
        self.pub = self.create_publisher(Image, "/perception/combined_overlay", 10)
        self.get_logger().info(f"combining {mask_topic} and {boxes_topic} into /perception/combined_overlay")

    def on_mask(self, msg):
        if msg.encoding != "mono8":
            self.get_logger().warn(f"mask: unsupported encoding {msg.encoding}")
            return
        labels = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width]
        self.masks.append((stamp(msg), labels.copy()))

    def on_boxes(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(f"boxes image: unsupported encoding {msg.encoding}")
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            img = img[..., ::-1]

        # pick the mask closest in time to this detector image
        note, labels = "no water mask yet", None
        if self.masks:
            t = stamp(msg)
            mask_t, nearest = min(self.masks, key=lambda m: abs(m[0] - t))
            gap = mask_t - t
            if abs(gap) < 1e-6:
                note, labels = "segmentation + detections, same frame", nearest
            elif abs(gap) <= self.get_parameter("max_age").value:
                when = "older" if gap < 0 else "newer"
                note, labels = f"segmentation + detections, mask {abs(gap):.1f} s {when}", nearest
            else:
                note = f"detections only: nearest mask is {abs(gap):.1f} s away"

        out_img = combine(img, labels, self.get_parameter("alpha").value, note)
        out = Image()
        out.header = msg.header
        out.height, out.width = out_img.shape[:2]
        out.encoding = "rgb8"
        out.step = out.width * 3
        out.data = out_img.tobytes()
        self.pub.publish(out)


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def combine(img, labels, alpha, note):
    """Detector image (RGB) with the water and obstacle pixels tinted, and a label in the corner."""
    out = img.astype(np.float32)
    if labels is not None:
        if labels.shape != img.shape[:2]:
            labels = cv2.resize(labels, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        for cls, colour in TINT.items():
            sel = labels == cls
            out[sel] = (1.0 - alpha) * out[sel] + alpha * colour
    out = out.astype(np.uint8)
    (w, h), _ = cv2.getTextSize(note, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.rectangle(out, (0, 0), (w + 16, h + 16), (0, 0, 0), -1)
    cv2.putText(out, note, (8, h + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def main():
    rclpy.init()
    node = CombinedOverlay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
