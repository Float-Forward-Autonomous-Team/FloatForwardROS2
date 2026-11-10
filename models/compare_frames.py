"""Side-by-side pictures of what changed between two eWaSR models, frame by frame.

Each picture has four panels: the camera image, the true labels, the old model and the new model.
In the model panels, magenta marks missed obstacles (obstacle called water) and yellow marks false
obstacles (water called obstacle); blue, orange and grey are water, obstacle and sky where the model
is right. Every frame is also listed with its numbers, so you can see whether a change is a few edge
pixels or a whole object.

Run inside the container, from this folder:
    cd /ws/src/boat/models
    python3 compare_frames.py --lars /ws/src/boat/datasets/LaRS --condition glitter
    python3 compare_frames.py --lars /ws/src/boat/datasets/LaRS --condition waves=still --sort improved --metric false

Options:
    --condition TAG     frames with this scene condition, as named in the evaluate_lars.py tables
                        (glitter, lighting=dusk_like, waves=still, reflections=heavy, extra_bright, ...)
    --split val         LaRS part: val (default) or train (with --frames-file for the held-out frames)
    --frames-file FILE  only the frames listed in FILE, e.g. ../datasets/LaRS_split/holdout.txt
    --names A,B         only these frames (names as in the list this script prints)
    --sort worse        worse (default): frames where the metric grew most come first;
                        improved: frames where it shrank most come first
    --metric missed     missed (obstacle called water, default) or false (water called obstacle)
    --count 8           how many pictures to write; every matching frame is listed either way
    --old / --new       the two ONNX models (default ewasr_resnet18.onnx and ewasr_resnet18_lars.onnx)
    --out DIR           where the pictures go (default ../datasets/compare/<condition>)

Objects found: labelled boats, buoys and other objects (not shore) count as found when the model calls at
least 70 % of the object obstacle, after growing its outline by 3 pixels. This is the rule of the official
LaRS evaluator, so a few wrong edge pixels do not count as a missed object.
"""

import argparse
import os

import cv2
import numpy as np

import lars_tools as LT
from evaluate_lars import OBSTACLE, WATER, Model, confusion

CLASS_RGB = np.array([[255, 120, 0], [0, 90, 255], [200, 200, 200]], np.uint8)  # obstacle, water, sky, as in the node
MISSED_RGB = np.array([230, 0, 230], np.uint8)  # magenta: rarely seen on water, unlike red buoys
FALSE_RGB = np.array([255, 220, 0], np.uint8)
UNCLEAR_RGB = np.array([40, 40, 40], np.uint8)
PANEL_W, BAR_H = 640, 30


def rates(c):
    """Missed obstacles (% of obstacle pixels called water) and false obstacles (% of water called obstacle)."""
    with np.errstate(invalid="ignore", divide="ignore"):
        return 100.0 * c[OBSTACLE, WATER] / c[OBSTACLE].sum(), 100.0 * c[WATER, OBSTACLE] / c[WATER].sum()


def objects_found(pred, objs, pan_ids, static):
    """(found, total) labelled objects, with the LaRS evaluator's 70 % coverage rule."""
    if pan_ids is None:
        return 0, 0
    valid = (pred == OBSTACLE) & ~static
    kernel = np.ones((7, 7), np.uint8)
    found = total = 0
    for o in objs:
        if o["crowd"] or o.get("id") is None:
            continue
        mask = (pan_ids == o["id"]).astype(np.uint8)
        area = int(mask.sum())
        if area == 0:
            continue
        total += 1
        grown = cv2.dilate(mask, kernel).astype(bool)
        if (valid & grown).sum() > 0.7 * area:
            found += 1
    return found, total


def panel(rgb, title, labels=None, truth=None):
    """One panel with a title bar: the plain image, or the image blended with class colours (labels).
    With truth given, missed and false obstacles are drawn solid on top; without, unclear pixels are dark."""
    h, w = rgb.shape[:2]
    size = (PANEL_W, round(h * PANEL_W / w))
    out = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA)
    if labels is not None:
        lab = cv2.resize(labels.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST)
        out = (0.5 * out + 0.5 * CLASS_RGB[np.clip(lab, 0, 2)]).astype(np.uint8)
        if truth is None:
            out[lab == 255] = UNCLEAR_RGB
        else:
            tru = cv2.resize(truth.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST)
            out[(tru == OBSTACLE) & (lab == WATER)] = MISSED_RGB
            out[(tru == WATER) & (lab == OBSTACLE)] = FALSE_RGB
    bar = np.zeros((BAR_H, size[0], 3), np.uint8)
    cv2.putText(bar, title, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([bar, out])


def legend(width):
    bar = np.zeros((BAR_H, width, 3), np.uint8)
    items = [("water", CLASS_RGB[1]), ("obstacle", CLASS_RGB[0]), ("sky", CLASS_RGB[2]),
             ("missed obstacle", MISSED_RGB), ("false obstacle", FALSE_RGB), ("unclear, not scored", UNCLEAR_RGB)]
    x = 8
    for text, colour in items:
        cv2.rectangle(bar, (x, 9), (x + 14, 23), tuple(int(v) for v in colour), -1)
        cv2.rectangle(bar, (x, 9), (x + 14, 23), (255, 255, 255), 1)
        cv2.putText(bar, text, (x + 20, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        x += 20 + 9 * len(text) + 24
    return bar


def fmt(v):
    return f"{v:5.1f}" if not np.isnan(v) else "    -"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lars", required=True, help="folder you unzipped LaRS into")
    parser.add_argument("--split", default="val", choices=["val", "train"])
    parser.add_argument("--frames-file")
    parser.add_argument("--condition")
    parser.add_argument("--names")
    parser.add_argument("--sort", default="worse", choices=["worse", "improved"])
    parser.add_argument("--metric", default="missed", choices=["missed", "false"])
    parser.add_argument("--count", type=int, default=8)
    parser.add_argument("--old", default="ewasr_resnet18.onnx")
    parser.add_argument("--new", default="ewasr_resnet18_lars.onnx")
    parser.add_argument("--out")
    args = parser.parse_args()

    frames = LT.list_frames(args.lars, args.split, args.frames_file)
    names = frames["names"]
    conditions = LT.conditions_for(frames["split_dir"], names)
    if args.condition:
        names = [n for n in names if args.condition in conditions.get(n, ())]
    if args.names:
        wanted = set(args.names.split(","))
        names = [n for n in names if n in wanted]
    if not names:
        tags = sorted({t for n in frames["names"] for t in conditions.get(n, ())})
        raise SystemExit("No frames match. Conditions in this part: " + ", ".join(tags))
    objects, _, static_id = LT.load_objects(frames["split_dir"], names)
    models = {"old": Model(args.old), "new": Model(args.new)}
    pan_dir = os.path.join(frames["split_dir"], "panoptic_masks")

    def load(name):
        rgb = cv2.cvtColor(cv2.imread(frames["image_of"][name]), cv2.COLOR_BGR2RGB)
        gt = LT.read_label_mask(os.path.join(frames["masks_dir"], name + ".png"))
        pan = cv2.imread(os.path.join(pan_dir, name + ".png"), cv2.IMREAD_COLOR)
        pan_ids = None if pan is None else (pan[..., 2].astype(np.int64) + 256 * pan[..., 1].astype(np.int64)
                                            + 65536 * pan[..., 0].astype(np.int64))
        static = None if pan is None else pan[..., 2] == static_id
        preds = {k: m.predict(rgb, (gt.shape[1], gt.shape[0])) for k, m in models.items()}
        return rgb, gt, pan_ids, static, preds

    # 1. numbers for every matching frame
    rows, pooled = [], {"old": np.zeros((3, 3), np.int64), "new": np.zeros((3, 3), np.int64)}
    objs_total = {"old": [0, 0], "new": [0, 0]}
    for name in names:
        rgb, gt, pan_ids, static, preds = load(name)
        row = {"name": name}
        for k, pred in preds.items():
            c = confusion(pred, gt)
            pooled[k] += c
            missed, false = rates(c)
            found, total = objects_found(pred, objects.get(name, []), pan_ids, static)
            objs_total[k][0] += found
            objs_total[k][1] += total
            row[k] = {"missed": missed, "false": false, "found": found, "total": total}
        row["delta"] = row["new"][args.metric] - row["old"][args.metric]
        rows.append(row)
    key = (lambda r: -r["delta"]) if args.sort == "worse" else (lambda r: r["delta"])
    rows.sort(key=lambda r: (np.isnan(r["delta"]), key(r) if not np.isnan(r["delta"]) else 0))

    what = args.condition or "selected frames"
    print(f"{what}: {len(rows)} frames in LaRS {args.split}; old = {models['old'].name}, new = {models['new'].name}")
    print(f"sorted by {args.metric} obstacles, {'largest increase' if args.sort == 'worse' else 'largest drop'} first\n")
    print(f"  {'frame':<34}{'missed % old -> new':>22}{'false % old -> new':>22}{'objects found old -> new':>27}")
    for r in rows:
        o, n = r["old"], r["new"]
        print(f"  {r['name'][:34]:<34}{fmt(o['missed']):>11} -> {fmt(n['missed']):<7}{fmt(o['false']):>13} -> "
              f"{fmt(n['false']):<7}{o['found']:>13}/{o['total']} -> {n['found']}/{n['total']}")
    po, pn = rates(pooled["old"]), rates(pooled["new"])
    print(f"\n  {'all these frames together':<34}{fmt(po[0]):>11} -> {fmt(pn[0]):<7}{fmt(po[1]):>13} -> {fmt(pn[1]):<7}"
          f"{objs_total['old'][0]:>13}/{objs_total['old'][1]} -> {objs_total['new'][0]}/{objs_total['new'][1]}")

    # 2. pictures for the first frames of the list
    tag = (args.condition or "frames").replace("=", "_")
    out_dir = args.out or os.path.join(os.path.dirname(os.path.abspath(args.lars.rstrip("/\\"))), "compare", tag)
    os.makedirs(out_dir, exist_ok=True)
    for i, r in enumerate(rows[:args.count], 1):
        rgb, gt, _, _, preds = load(r["name"])
        o, n = r["old"], r["new"]
        top = np.hstack([panel(rgb, r["name"]), panel(rgb, "True labels", labels=gt)])
        bottom = np.hstack([
            panel(rgb, f"Old: missed {fmt(o['missed']).strip()}%, false {fmt(o['false']).strip()}%, "
                       f"objects {o['found']}/{o['total']}", labels=preds["old"], truth=gt),
            panel(rgb, f"New: missed {fmt(n['missed']).strip()}%, false {fmt(n['false']).strip()}%, "
                       f"objects {n['found']}/{n['total']}", labels=preds["new"], truth=gt)])
        picture = np.vstack([top, bottom, legend(top.shape[1])])
        path = os.path.join(out_dir, f"{i:02d}_{r['name']}.jpg")
        cv2.imwrite(path, cv2.cvtColor(picture, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
    print(f"\n{min(args.count, len(rows))} pictures written to {out_dir}")


if __name__ == "__main__":
    main()
