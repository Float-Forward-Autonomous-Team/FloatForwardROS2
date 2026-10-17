import time

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# what eWaSR expects: RGB, 512 x 384, normalised like ImageNet
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
# colour per class for the overlay: 0 obstacle = orange, 1 water = blue, 2 sky = grey
COLOURS = np.array([[255, 120, 0], [0, 90, 255], [200, 200, 200]], np.uint8)


class WaterSegmentation(Node):
    """Runs eWaSR on the WAM-V camera: which pixels are water, sky or obstacle."""

    def __init__(self):
        super().__init__("water_segmentation")
        self.declare_parameter("image_topic", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw")
        self.declare_parameter("model_path", "/ws/src/boat/models/ewasr_resnet18.onnx")
        self.declare_parameter("seconds_between_frames", 1.0)  # eWaSR on a CPU is slow

        self.model = ort.InferenceSession(
            self.get_parameter("model_path").value, providers=["CPUExecutionProvider"]
        )
        self.input_name = self.model.get_inputs()[0].name
        topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, "/perception/water_overlay", 10)
        self.last = 0.0
        self.get_logger().info(f"eWaSR loaded, listening to {topic}")

    def on_image(self, msg):
        if time.time() - self.last < self.get_parameter("seconds_between_frames").value:
            return
        self.last = time.time()

        # 1. ROS image -> numpy array (height x width x 3, RGB)
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(f"unsupported encoding {msg.encoding}")
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            img = img[..., ::-1]

        # 2. prepare the input eWaSR expects
        x = cv2.resize(img, (512, 384)).astype(np.float32) / 255.0
        x = ((x - MEAN) / STD).transpose(2, 0, 1)[None].astype(np.float32)

        # 3. run the network: one score per class per pixel, take the best class
        start = time.time()
        scores = self.model.run(None, {self.input_name: x})[0][0]
        labels = scores.argmax(axis=0).astype(np.uint8)  # 0 obstacle, 1 water, 2 sky
        ms = (time.time() - start) * 1000

        # 4. report how much of the image is water, sky and obstacle
        share = [100.0 * np.mean(labels == k) for k in range(3)]
        self.get_logger().info(
            f"water {share[1]:.0f} %, sky {share[2]:.0f} %, obstacles {share[0]:.0f} %  ({ms:.0f} ms)"
        )

        # 5. publish the camera image with the classes painted on top
        big = cv2.resize(labels, (msg.width, msg.height), interpolation=cv2.INTER_NEAREST)
        overlay = (0.5 * img + 0.5 * COLOURS[big]).astype(np.uint8)
        out = Image()
        out.header = msg.header
        out.height, out.width = overlay.shape[:2]
        out.encoding = "rgb8"
        out.step = out.width * 3
        out.data = overlay.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = WaterSegmentation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
