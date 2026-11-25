import os
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
OBSTACLE, WATER, SKY, OWN_BOAT = 0, 1, 2, 3  # classes in the published mask; eWaSR itself gives 0 to 2
# colour per class for the overlay: obstacle orange, water blue, sky grey, our own boat dark grey
COLOURS = np.array([[255, 120, 0], [0, 90, 255], [200, 200, 200], [40, 40, 40]], np.uint8)


def parse_polygons(text):
    """Own-boat outline from the own_boat_polygons parameter -> list of (n, 2) arrays.

    Format: "x,y x,y x,y ...; x,y x,y x,y ..." with x, y as fractions of the image width and height
    (0,0 = top left corner, 1,1 = bottom right corner), at least 3 points per polygon, polygons separated
    by ";". "" or "none" = no own boat in view."""
    polygons = []
    if text.strip().lower() in ("", "none"):
        return polygons
    for part in text.split(";"):
        if not part.strip():
            continue
        points = [p.split(",") for p in part.split()]
        if len(points) < 3 or any(len(p) != 2 for p in points):
            raise ValueError(f"'{part.strip()}': need at least 3 points written as x,y")
        try:
            pts = np.array(points, np.float64)
        except ValueError:
            raise ValueError(f"'{part.strip()}': x and y must be numbers") from None
        if (pts < 0).any() or (pts > 1).any():
            raise ValueError(f"'{part.strip()}': x and y are fractions of the image size, from 0 to 1")
        polygons.append(pts)
    return polygons


def own_boat_region(polygons, width, height):
    """The pixels inside the own-boat polygons, as a boolean image of the given size."""
    region = np.zeros((height, width), np.uint8)
    for pts in polygons:
        cv2.fillPoly(region, [np.round(pts * [width, height]).astype(np.int32)], 1)
    return region.astype(bool)


class WaterSegmentation(Node):
    """Runs eWaSR on the WAM-V camera: which pixels are water, sky or obstacle.

    Publishes the class of every pixel on /perception/water_mask (mono8: 0 obstacle, 1 water,
    2 sky, 3 our own boat; same size and header as the camera image) and, if publish_overlay is true,
    the camera image with the classes painted on it on /perception/water_overlay.

    Our own boat: the camera is fixed to the boat, so the parts of our boat it sees (the WAM-V's hulls in
    the bottom corners in VRX) are always at the same pixels. own_boat_polygons outlines them, and every
    pixel inside is published as 3 instead of what the model says, so nothing downstream takes our own
    hulls for an obstacle."""

    def __init__(self):
        super().__init__("water_segmentation")
        self.declare_parameter("image_topic", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw")
        self.declare_parameter("model_path", "/ws/src/boat/models/ewasr_resnet18_lars.onnx")
        self.declare_parameter("seconds_between_frames", 1.0)  # eWaSR on a CPU is slow; 0.0 = every frame
        self.declare_parameter("publish_overlay", True)  # false saves the drawing time on the boat
        # where our own boat is in the image (see parse_polygons); "" = not in view
        self.declare_parameter("own_boat_polygons", "")
        self.declare_parameter("report_every", 20)  # frames per timing summary

        model_path = self.get_parameter("model_path").value
        if not os.path.exists(model_path):
            self.get_logger().fatal(
                f"eWaSR model not found: {model_path}. Put the .onnx file in the models folder "
                "(see 'Models' in documentation/PERCEPTION.md) or pass -p model_path:=<file>."
            )
            raise SystemExit(1)
        try:
            self.own_boat = parse_polygons(self.get_parameter("own_boat_polygons").value)
        except ValueError as e:
            self.get_logger().fatal(f"own_boat_polygons: {e}")
            raise SystemExit(1)
        self.own_boat_pixels = None  # own_boat_region at the camera's resolution, made on the first frame
        self.model = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.model.get_inputs()[0].name
        topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.mask_pub = self.create_publisher(Image, "/perception/water_mask", 10)
        self.pub = self.create_publisher(Image, "/perception/water_overlay", 10)
        self.last = float("-inf")
        self.frames = 0
        self.inference_ms = []
        self.total_ms = []
        self.get_logger().info(
            f"eWaSR loaded ({os.path.basename(model_path)}), listening to {topic}; "
            + (f"our own boat: {len(self.own_boat)} outline(s)" if self.own_boat else "no own boat in view")
        )

    def on_image(self, msg):
        start = time.perf_counter()
        if start - self.last < self.get_parameter("seconds_between_frames").value:
            return
        self.last = start

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

        # 3. run the network, timed on its own: one score per class per pixel, take the best class
        t_model = time.perf_counter()
        scores = self.model.run(None, {self.input_name: x})[0][0]
        inference_ms = (time.perf_counter() - t_model) * 1000
        labels = scores.argmax(axis=0).astype(np.uint8)  # 0 obstacle, 1 water, 2 sky

        # 4. publish the class of every pixel, at the camera's resolution and with its header,
        #    so pixel (u, v) of the mask is pixel (u, v) of the image and of its camera_info;
        #    our own boat (always the same pixels) is 3, whatever the model says there
        big = cv2.resize(labels, (msg.width, msg.height), interpolation=cv2.INTER_NEAREST)
        if self.own_boat:
            if self.own_boat_pixels is None or self.own_boat_pixels.shape != big.shape:
                self.own_boat_pixels = own_boat_region(self.own_boat, msg.width, msg.height)
            big[self.own_boat_pixels] = OWN_BOAT
        self.mask_pub.publish(to_mask(big, msg.header))

        # 5. publish the camera image with the classes painted on top
        if self.get_parameter("publish_overlay").value:
            overlay = (0.5 * img + 0.5 * COLOURS[big]).astype(np.uint8)
            out = Image()
            out.header = msg.header
            out.height, out.width = overlay.shape[:2]
            out.encoding = "rgb8"
            out.step = out.width * 3
            out.data = overlay.tobytes()
            self.pub.publish(out)

        # 6. report how much of the image is water, sky, obstacle and our own boat, and how long this frame took
        total_ms = (time.perf_counter() - start) * 1000
        share = 100.0 * np.bincount(big.ravel(), minlength=4)[:4] / big.size
        own = f", own boat {share[3]:.0f} %" if share[3] > 0 else ""
        self.get_logger().info(
            f"water {share[1]:.0f} %, sky {share[2]:.0f} %, obstacles {share[0]:.0f} %{own} | "
            f"eWaSR inference: {inference_ms:.1f} ms, total: {total_ms:.1f} ms"
        )
        self.record_timing(inference_ms, total_ms)

    def record_timing(self, inference_ms, total_ms):
        """Collects frame times and logs a summary every report_every frames."""
        self.frames += 1
        if self.frames == 1:
            return  # the first frame includes one-off start-up cost, keep it out of the summary
        self.inference_ms.append(inference_ms)
        self.total_ms.append(total_ms)
        if len(self.inference_ms) < self.get_parameter("report_every").value:
            return
        inf = np.array(self.inference_ms)
        tot = np.array(self.total_ms)
        self.get_logger().info(
            f"eWaSR timing over {len(inf)} frames: inference mean {inf.mean():.1f} ms, "
            f"median {np.median(inf):.1f} ms, p95 {np.percentile(inf, 95):.1f} ms, max {inf.max():.1f} ms; "
            f"total per frame mean {tot.mean():.1f} ms, "
            f"so this node can do at most {1000 / tot.mean():.1f} frames/s"
        )
        self.inference_ms.clear()
        self.total_ms.clear()


def to_mask(labels, header):
    """Class per pixel (height x width, uint8) -> sensor_msgs/Image, mono8."""
    out = Image()
    out.header = header
    out.height, out.width = labels.shape
    out.encoding = "mono8"
    out.is_bigendian = 0
    out.step = out.width
    out.data = np.ascontiguousarray(labels, np.uint8).tobytes()
    return out


def main():
    rclpy.init()
    try:
        node = WaterSegmentation()
    except SystemExit:
        rclpy.try_shutdown()
        raise
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
