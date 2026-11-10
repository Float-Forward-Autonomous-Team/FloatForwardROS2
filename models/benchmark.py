"""Times eWaSR and YOLO26-s on this computer, without ROS or Gazebo.

Run it inside the container, from this folder, with the simulator stopped:
    cd /ws/src/boat/models
    python3 benchmark.py

Options:
    --runs 100          timed runs per model (default 50)
    --image frame.jpg   time on a real camera frame instead of a random 1280x720 image
    --only ewasr        time one model only (ewasr or yolo)

It only needs Python, OpenCV, onnxruntime and ultralytics, so the same script
can be run on each candidate onboard computer to compare them.
"""

import argparse
import os
import platform
import time

import cv2
import numpy as np

# what eWaSR expects: RGB, 512 x 384, normalised like ImageNet (same as the ROS node)
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def cpu_name():
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or platform.machine()


def summary(label, values, show_rate=False):
    a = np.array(values)
    line = (
        f"  {label:<20} mean {a.mean():7.1f}   median {np.median(a):7.1f}   "
        f"p95 {np.percentile(a, 95):7.1f}   max {a.max():7.1f} ms"
    )
    if show_rate:
        line += f"   -> {1000 / a.mean():.1f} frames/s"
    print(line)


def time_ewasr(path, rgb, runs, warmup):
    import onnxruntime as ort

    if not os.path.exists(path):
        print(f"eWaSR: {path} not found, skipped (see documentation/PERCEPTION.md, Models)\n")
        return
    session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    name = session.get_inputs()[0].name

    def prepare(image):
        x = cv2.resize(image, (512, 384)).astype(np.float32) / 255.0
        return ((x - MEAN) / STD).transpose(2, 0, 1)[None].astype(np.float32)

    for _ in range(warmup):
        session.run(None, {name: prepare(rgb)})
    model_ms, total_ms = [], []
    for _ in range(runs):
        t0 = time.perf_counter()
        x = prepare(rgb)
        t1 = time.perf_counter()
        scores = session.run(None, {name: x})[0][0]
        t2 = time.perf_counter()
        scores.argmax(axis=0)
        t3 = time.perf_counter()
        model_ms.append((t2 - t1) * 1000)
        total_ms.append((t3 - t0) * 1000)
    print(f"eWaSR ResNet-18 (onnxruntime {ort.__version__}, CPU)")
    summary("model only", model_ms)
    summary("with pre/post", total_ms, show_rate=True)
    print()


def time_yolo(path, rgb, runs, warmup):
    import torch
    import ultralytics
    from ultralytics import YOLO

    model = YOLO(path)  # downloads yolo26s.pt into this folder if it is missing
    bgr = np.ascontiguousarray(rgb[..., ::-1])  # Ultralytics expects BGR, like OpenCV
    for _ in range(warmup):
        model.predict(bgr, verbose=False)
    model_ms, call_ms = [], []
    for _ in range(runs):
        t0 = time.perf_counter()
        result = model.predict(bgr, verbose=False)[0]
        call_ms.append((time.perf_counter() - t0) * 1000)
        model_ms.append(result.speed["inference"])  # Ultralytics' own measurement, in ms
    device = "GPU (CUDA)" if torch.cuda.is_available() else "CPU"
    print(f"YOLO26-s (ultralytics {ultralytics.__version__}, {device})")
    summary("model only", model_ms)
    summary("full predict call", call_ms, show_rate=True)
    print()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--runs", type=int, default=50, help="timed runs per model")
    parser.add_argument("--warmup", type=int, default=5, help="untimed runs before timing starts")
    parser.add_argument("--image", help="image file to use instead of a random 1280x720 image")
    parser.add_argument("--only", choices=["ewasr", "yolo"], help="time one model only")
    parser.add_argument("--ewasr", default="ewasr_resnet18.onnx", help="eWaSR ONNX file")
    parser.add_argument("--yolo", default="yolo26s.pt", help="YOLO weights")
    args = parser.parse_args()

    if args.image:
        bgr = cv2.imread(args.image)
        if bgr is None:
            raise SystemExit(f"cannot read {args.image}")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    else:
        rgb = np.random.default_rng(0).integers(0, 256, (720, 1280, 3), dtype=np.uint8)

    print(f"CPU: {cpu_name()}, {os.cpu_count()} logical cores")
    print(f"Image: {rgb.shape[1]}x{rgb.shape[0]}, {args.warmup} warm-up + {args.runs} timed runs per model\n")
    if args.only != "yolo":
        time_ewasr(args.ewasr, rgb, args.runs, args.warmup)
    if args.only != "ewasr":
        time_yolo(args.yolo, rgb, args.runs, args.warmup)


if __name__ == "__main__":
    main()
