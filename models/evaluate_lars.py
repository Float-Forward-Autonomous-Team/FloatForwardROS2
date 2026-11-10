"""Scores the eWaSR water segmentation model on the LaRS dataset.

The model runs exactly as in the water_segmentation node (resize to 512x384, ImageNet
normalisation, best class per pixel, nearest-neighbour resize back), so the numbers
describe what the node does.

Run inside the container, from this folder:
    cd /ws/src/boat/models
    python3 evaluate_lars.py --lars /ws/src/boat/datasets/LaRS

Compare two models on the same frames (the difference is printed as a third table):
    python3 evaluate_lars.py --lars /ws/src/boat/datasets/LaRS \\
        --model ewasr_resnet18.onnx --model ewasr_resnet18_lars.onnx

Options:
    --split val              LaRS part to score: val (default) or train; test has no public labels
    --frames-file FILE       score only the frames listed in FILE, e.g. the held-out training frames:
                             --split train --frames-file ../datasets/LaRS_split/holdout.txt
    --min-frames 5           hide scene conditions with fewer frames than this
    --limit 20               score only the first N frames, for a quick check
    --save-predictions DIR   also write the predicted masks (PNG, class ids 0 1 2), the format the
                             official LaRS evaluator reads with configs/v1.0.0/lars_val_semantic_lbl.yaml

What the columns mean, per scene condition from LaRS's image_annotations.json:
    IoU            overlap of predicted and true region, in % (higher is better)
    water->sky     share of true water pixels called sky (the VRX near-water error)
    water->obst    share of true water pixels called obstacle (false alarms, e.g. sun glitter)
    obst->water    share of true obstacle pixels called water (missed obstacles, the dangerous one)

Per-frame results are also written to lars_<split>_<model name>.csv in the current folder
(lars_<split>-<frames file name>_<model name>.csv with --frames-file).
"""

import argparse
import csv
import os
import time

import cv2
import numpy as np
import onnxruntime as ort

import lars_tools as LT

# what eWaSR expects: RGB, 512 x 384, normalised like ImageNet (same as the ROS node)
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
# class ids, the same in eWaSR's output and in LaRS's semantic masks; 255 in LaRS = ignore
OBSTACLE, WATER, SKY = 0, 1, 2


class Model:
    """The eWaSR ONNX model, with the same pre- and post-processing as the ROS node."""

    def __init__(self, path):
        self.name = os.path.splitext(os.path.basename(path))[0]
        self.session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0].name

    def predict(self, rgb, size):
        x = cv2.resize(rgb, (512, 384)).astype(np.float32) / 255.0
        x = ((x - MEAN) / STD).transpose(2, 0, 1)[None].astype(np.float32)
        scores = self.session.run(None, {self.input: x})[0][0]
        labels = scores.argmax(axis=0).astype(np.uint8)
        return cv2.resize(labels, size, interpolation=cv2.INTER_NEAREST)  # size = (width, height)


def confusion(pred, gt):
    """3x3 pixel counts, rows = true class, columns = predicted class; ignore pixels left out."""
    valid = gt < 3
    return np.bincount(gt[valid].astype(np.int64) * 3 + pred[valid], minlength=9).reshape(3, 3)


def measures(c):
    """IoU per class, mean IoU and the three error rates from a confusion matrix, all in %."""
    with np.errstate(invalid="ignore", divide="ignore"):
        iou = 100.0 * np.diag(c) / (c.sum(1) + c.sum(0) - np.diag(c))
        rate = lambda true, pred: 100.0 * c[true, pred] / c[true].sum()
        return {
            "water IoU": iou[WATER], "sky IoU": iou[SKY], "obst IoU": iou[OBSTACLE],
            "mIoU": np.nanmean(iou) if not np.isnan(iou).all() else np.nan,
            "water->sky": rate(WATER, SKY), "water->obst": rate(WATER, OBSTACLE),
            "obst->water": rate(OBSTACLE, WATER),
        }


COLUMNS = ["water IoU", "sky IoU", "obst IoU", "mIoU", "water->sky", "water->obst", "obst->water"]
LOWER_IS_BETTER = {"water->sky", "water->obst", "obst->water"}


def groups(frames, conditions, min_frames):
    """[(label, list of frame names)]: all frames first, then each scene condition."""
    out = [("all frames", frames)]
    tags = sorted({t for n in frames for t in conditions.get(n, ())})
    for tag in tags:
        members = [n for n in frames if tag in conditions.get(n, ())]
        if len(members) >= min_frames:
            out.append((tag, members))
    return out


def table(title, rows):
    print(f"\n{title}")
    print(f"  {'condition':<30}{'frames':>7}" + "".join(f"{c:>13}" for c in COLUMNS))
    for label, n, values, flag in rows:
        cells = "".join(f"{values[c]:>13.1f}" if not np.isnan(values[c]) else f"{'-':>13}" for c in COLUMNS)
        print(f"  {label[:30]:<30}{n:>7}{cells}{flag}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lars", required=True, help="folder you unzipped LaRS into")
    parser.add_argument("--split", default="val", choices=["val", "train"])
    parser.add_argument("--model", action="append", help="eWaSR ONNX file; give twice to compare")
    parser.add_argument("--frames-file", help="score only the frames listed in this file")
    parser.add_argument("--min-frames", type=int, default=5)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--save-predictions", metavar="DIR")
    args = parser.parse_args()
    model_paths = args.model or ["ewasr_resnet18.onnx"]

    # 1. the frames: names from image_list.txt, an image and a label mask for each
    frames = LT.list_frames(args.lars, args.split, args.frames_file, args.limit)
    names, masks_dir = frames["names"], frames["masks_dir"]

    # 2. scene conditions (glitter, reflections, lighting, ...) per frame, if LaRS provides them
    conditions = LT.conditions_for(frames["split_dir"], names)
    part = args.split + (f" ({os.path.basename(args.frames_file)})" if args.frames_file else "")
    print(f"LaRS {part}: {len(names)} frames; scene conditions read for {len(conditions)} of them")
    if not conditions:
        print("  (no conditions read: results are for all frames only; send the first lines of "
              "image_annotations.json if you want them per condition)")

    # 3. run every model on every frame and count pixels per (true class, predicted class)
    models = [Model(p) for p in model_paths]
    per_frame = {m.name: {} for m in models}
    start = time.perf_counter()
    for i, name in enumerate(names, 1):
        rgb = cv2.cvtColor(cv2.imread(frames["image_of"][name]), cv2.COLOR_BGR2RGB)
        gt = LT.read_label_mask(os.path.join(masks_dir, name + ".png"))
        for m in models:
            pred = m.predict(rgb, (gt.shape[1], gt.shape[0]))
            per_frame[m.name][name] = confusion(pred, gt)
            if args.save_predictions:
                out_dir = os.path.join(args.save_predictions, m.name)
                os.makedirs(out_dir, exist_ok=True)
                cv2.imwrite(os.path.join(out_dir, name + ".png"), pred)
        if i % 25 == 0 or i == len(names):
            per = (time.perf_counter() - start) / i
            print(f"  scored {i}/{len(names)} frames ({per:.2f} s per frame)", flush=True)

    # 4. per-frame CSV for each model
    tag = args.split + (f"-{LT.stem(args.frames_file)}" if args.frames_file else "")
    for m in models:
        out = f"lars_{tag}_{m.name}.csv"
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["image", "conditions"] + COLUMNS + [f"true{t}_pred{p}" for t in range(3) for p in range(3)])
            for name, c in per_frame[m.name].items():
                v = measures(c)
                w.writerow([name, ";".join(sorted(conditions.get(name, ())))]
                           + [f"{v[k]:.2f}" for k in COLUMNS] + list(c.flatten()))
        print(f"  per-frame results: {out}")

    # 5. tables: one per model, then the difference if there are two
    grouped = groups(names, conditions, args.min_frames)
    results = {}
    for m in models:
        rows = []
        for label, members in grouped:
            c = sum(per_frame[m.name][n] for n in members)
            rows.append((label, len(members), measures(c), ""))
        results[m.name] = rows
        table(f"{m.name}  (IoU higher is better; the three error rates lower is better, all in %)", rows)

    if len(models) == 2:
        a, b = (results[m.name] for m in models)
        diff_rows = []
        for (label, n, va, _), (_, _, vb, _) in zip(a, b):
            d = {c: vb[c] - va[c] for c in COLUMNS}
            worse = [c for c in COLUMNS if (d[c] > 1.0 if c in LOWER_IS_BETTER else d[c] < -1.0)]
            diff_rows.append((label, n, d, "   worse: " + ", ".join(worse) if worse else ""))
        table(f"difference: {models[1].name} minus {models[0].name}, in percentage points "
              "('worse' = more than 1 point worse)", diff_rows)


if __name__ == "__main__":
    main()
