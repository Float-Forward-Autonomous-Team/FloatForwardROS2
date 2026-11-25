import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


class YoloDetector(Node):
    """Runs YOLO26-s on the WAM-V camera: what the objects are and where they are in the image."""

    def __init__(self):
        super().__init__("yolo_detector")
        self.declare_parameter("image_topic", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw")
        self.declare_parameter("model_path", "/ws/src/boat/models/yolo26s.pt")
        self.declare_parameter("confidence", 0.3)
        self.declare_parameter("seconds_between_frames", 0.5)

        self.model = YOLO(self.get_parameter("model_path").value)
        topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, "/perception/yolo_overlay", 10)
        self.last = 0.0
        self.get_logger().info(f"YOLO loaded ({len(self.model.names)} classes), listening to {topic}")

    def on_image(self, msg):
        if time.time() - self.last < self.get_parameter("seconds_between_frames").value:
            return
        self.last = time.time()

        # 1. ROS image -> numpy array in BGR, which is what Ultralytics expects
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(f"unsupported encoding {msg.encoding}")
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = img[..., ::-1]
        img = np.ascontiguousarray(img)

        # 2. run the detector
        start = time.time()
        result = self.model.predict(img, conf=self.get_parameter("confidence").value, verbose=False)[0]
        ms = (time.time() - start) * 1000

        # 3. report each detection: class, confidence, box centre in pixels
        found = []
        for box in result.boxes:
            name = self.model.names[int(box.cls[0])]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            found.append(f"{name} {float(box.conf[0]):.2f} at x={int((x1 + x2) / 2)} y={int((y1 + y2) / 2)}")
        self.get_logger().info(f"{len(found)} detections: {', '.join(found)}  ({ms:.0f} ms)")

        # 4. publish the image with the boxes drawn on it
        drawn = np.ascontiguousarray(result.plot()[..., ::-1])  # plot() gives BGR, ROS wants rgb8
        out = Image()
        out.header = msg.header
        out.height, out.width = drawn.shape[:2]
        out.encoding = "rgb8"
        out.step = out.width * 3
        out.data = drawn.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
