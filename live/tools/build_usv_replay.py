"""Turn a short USV-01 recording into the replay the public page loops while the boat is offline.

Reads a rosbag2 (MCAP) recorded with `make record`, keeps only what the public page
shows, thins each topic to the rate the page needs, and writes:
  live/data/usv01/replay.json         timed messages, in the shape the page's handlers expect
  live/data/usv01/<camera>/NNNN.jpg   camera frames, downscaled

Runs inside the container, where rosbag2_py and Pillow are installed:
  python3 live/tools/build_usv_replay.py <bag folder> [output folder]
"""

import json
import shutil
import sys
from io import BytesIO
from pathlib import Path

import rosbag2_py
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

GPS = "/wamv/sensors/gps/gps/fix"
TRACK = "/gps_track"
IMU = "/wamv/sensors/imu/imu/data"
OBSTACLES = "/obstacles/markers"
DETECTIONS = "/detections"
INFERENCE = "/detections/inference_ms"
CAMERA_PREFIX = "/public/cameras/"

# minimum seconds between kept messages, per topic
MIN_STEP_S = {GPS: 0.2, TRACK: 5.0, IMU: 0.1, OBSTACLES: 0.2, DETECTIONS: 0.2, INFERENCE: 0.5}
FRAME_STEP_S = 0.5  # camera frames per second in the replay: 2
FRAME_WIDTH = 480
FRAME_QUALITY = 65


def r(x, digits=3):
    """Round a float for a smaller file."""
    return round(float(x), digits)


def stamp(header):
    """Return a header stamp as the dict the page reads."""
    return {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec}


def color(c):
    """Return a std_msgs/ColorRGBA as a dict."""
    return {"r": r(c.r), "g": r(c.g), "b": r(c.b), "a": r(c.a)}


def slim(topic, msg):
    """Return only the fields of msg that the live page reads."""
    if topic == GPS:
        return {
            "header": {"stamp": stamp(msg.header)},
            "latitude": r(msg.latitude, 8),
            "longitude": r(msg.longitude, 8),
        }
    if topic == TRACK:
        return {"geojson": msg.geojson}
    if topic == IMU:
        q = msg.orientation
        return {"orientation": {"x": r(q.x, 4), "y": r(q.y, 4), "z": r(q.z, 4), "w": r(q.w, 4)}}
    if topic == OBSTACLES:
        return {
            "markers": [
                {
                    "id": m.id,
                    "ns": m.ns,
                    "text": m.text,
                    "pose": {
                        "position": {"x": r(m.pose.position.x, 2), "y": r(m.pose.position.y, 2)}
                    },
                    "color": color(m.color),
                }
                for m in msg.markers
            ]
        }
    if topic == DETECTIONS:
        return {
            "points": [
                {
                    "points": [{"x": r(p.x, 1), "y": r(p.y, 1)} for p in shape.points],
                    "outline_color": color(shape.outline_color),
                }
                for shape in msg.points
            ],
            "texts": [
                {"text": t.text, "position": {"x": r(t.position.x, 1), "y": r(t.position.y, 1)}}
                for t in msg.texts
            ],
        }
    if topic == INFERENCE:
        return {"data": r(msg.data, 2)}
    return None


def shrink_jpeg(data):
    """Return the JPEG scaled down to FRAME_WIDTH and re-encoded."""
    img = Image.open(BytesIO(bytes(data))).convert("RGB")
    if img.width > FRAME_WIDTH:
        height = round(img.height * FRAME_WIDTH / img.width)
        img = img.resize((FRAME_WIDTH, height), Image.BILINEAR)
    out = BytesIO()
    img.save(out, format="JPEG", quality=FRAME_QUALITY)
    return out.getvalue()


def main(bag, out_dir):
    """Read the bag and write the replay JSON and camera frames."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}

    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)

    start = None
    last = {}
    messages = []
    frames = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if start is None:
            start = t_ns
        t = (t_ns - start) / 1e9
        is_camera = topic.startswith(CAMERA_PREFIX)
        step = FRAME_STEP_S if is_camera else MIN_STEP_S.get(topic)
        if step is None or (topic in last and t - last[topic] < step):
            continue
        msg = deserialize_message(data, types[topic])
        if is_camera:
            name = topic[len(CAMERA_PREFIX) :].split("/")[0]
            index = len(frames.setdefault(name, []))
            path = Path(name) / f"{index:04d}.jpg"
            (out_dir / name).mkdir(exist_ok=True)
            (out_dir / path).write_bytes(shrink_jpeg(msg.data))
            frames[name].append([r(t, 2), path.as_posix()])
        else:
            messages.append([r(t, 2), topic, slim(topic, msg)])
        last[topic] = t

    if not messages:
        sys.exit("No public topics in this recording; was `make race` running?")
    duration = max(m[0] for m in messages)
    replay = {"duration": r(duration, 2), "messages": messages, "frames": frames}
    (out_dir / "replay.json").write_text(json.dumps(replay, separators=(",", ":")) + "\n")

    size = sum(f.stat().st_size for f in out_dir.rglob("*") if f.is_file())
    counts = ", ".join(f"{name}: {len(f)} frames" for name, f in frames.items()) or "no camera"
    print(f"{duration:.0f} s, {len(messages)} messages, {counts}, {size / 1e6:.1f} MB")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(Path(sys.argv[1]), Path(sys.argv[2] if len(sys.argv) > 2 else "live/data/usv01"))
