"""Old and new eWaSR side by side on photos without labels, for example the Monaco photos.

Each picture has three panels: the photo, the old model and the new model, coloured water (blue),
obstacle (orange) and sky (grey) as in compare_frames.py. With no labels there are no scores and no
missed or false marks, so judge by eye where each model puts the water and the obstacles. The list
shows how much of each photo the two models call water, obstacle and sky, and on how much of the
photo they disagree.

Put the photos in a folder that git ignores, such as datasets/monaco, then run inside the container,
from this folder:
    cd /ws/src/boat/models
    python3 compare_photos.py --photos ../datasets/monaco

Options:
    --photos PATH ...   folders and/or photo files (.jpg, .jpeg, .png)
    --old / --new       the two ONNX models (default ewasr_resnet18.onnx and ewasr_resnet18_lars.onnx)
    --out DIR           where the pictures go (default: compare/photos next to the photos' folder,
                        so datasets/compare/photos for photos in datasets/monaco)
"""

import argparse
import glob
import os

import cv2
import numpy as np

from evaluate_lars import Model

OBSTACLE, WATER, SKY = 0, 1, 2  # eWaSR's class ids
CLASS_RGB = np.array([[255, 120, 0], [0, 90, 255], [200, 200, 200]], np.uint8)  # as in compare_frames.py
PANEL_W, BAR_H = 640, 30
EXTENSIONS = (".jpg", ".jpeg", ".png")


def find_photos(paths):
    photos = []
    for p in paths:
        if os.path.isdir(p):
            photos += sorted(f for f in glob.glob(os.path.join(p, "*")) if f.lower().endswith(EXTENSIONS))
        elif os.path.isfile(p):
            photos.append(p)
        else:
            raise SystemExit(f"Not found: {p}")
    if not photos:
        raise SystemExit("No .jpg, .jpeg or .png photos in " + ", ".join(paths))
    return photos


def shares(labels):
    """% of the photo called water, obstacle and sky, indexed by class id."""
    return 100.0 * np.bincount(labels.ravel().astype(np.int64), minlength=3)[:3] / labels.size


def panel(rgb, title, labels=None):
    """One panel with a title bar: the plain photo, or the photo blended with the class colours."""
    h, w = rgb.shape[:2]
    size = (PANEL_W, round(h * PANEL_W / w))
    out = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA)
    if labels is not None:
        lab = cv2.resize(labels.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST)
        out = (0.5 * out + 0.5 * CLASS_RGB[np.clip(lab, 0, 2)]).astype(np.uint8)
    bar = np.zeros((BAR_H, size[0], 3), np.uint8)
    cv2.putText(bar, title, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([bar, out])


def legend(width):
    bar = np.zeros((BAR_H, width, 3), np.uint8)
    x = 8
    for text, cls in (("water", WATER), ("obstacle", OBSTACLE), ("sky", SKY)):
        colour = tuple(int(v) for v in CLASS_RGB[cls])
        cv2.rectangle(bar, (x, 9), (x + 14, 23), colour, -1)
        cv2.rectangle(bar, (x, 9), (x + 14, 23), (255, 255, 255), 1)
        cv2.putText(bar, text, (x + 20, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        x += 20 + 9 * len(text) + 24
    cv2.putText(bar, "no labels for these photos: no scores, judge by eye", (x + 16, 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
    return bar


def split3(s):
    return f"{s[WATER]:5.1f} / {s[OBSTACLE]:5.1f} / {s[SKY]:5.1f}"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--photos", nargs="+", required=True)
    parser.add_argument("--old", default="ewasr_resnet18.onnx")
    parser.add_argument("--new", default="ewasr_resnet18_lars.onnx")
    parser.add_argument("--out")
    args = parser.parse_args()

    photos = find_photos(args.photos)
    first = args.photos[0] if os.path.isdir(args.photos[0]) else (os.path.dirname(args.photos[0]) or ".")
    out_dir = args.out or os.path.join(os.path.dirname(os.path.abspath(first.rstrip("/\\"))), "compare", "photos")
    os.makedirs(out_dir, exist_ok=True)
    models = {"old": Model(args.old), "new": Model(args.new)}

    print(f"{len(photos)} photos; old = {models['old'].name}, new = {models['new'].name}\n")
    print(f"  {'photo':<34}{'old: water / obstacle / sky %':>31}{'new: water / obstacle / sky %':>31}"
          f"{'disagree %':>12}")
    written = 0
    for path in photos:
        bgr = cv2.imread(path)
        if bgr is None:
            print(f"  {os.path.basename(path)}: could not be read, skipped")
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        preds = {k: np.asarray(m.predict(rgb, (w, h))) for k, m in models.items()}
        s = {k: shares(p) for k, p in preds.items()}
        disagree = 100.0 * float((preds["old"] != preds["new"]).mean())
        name = os.path.splitext(os.path.basename(path))[0]
        print(f"  {name[:34]:<34}{split3(s['old']):>31}{split3(s['new']):>31}{disagree:>11.1f}")

        titles = {k: f"{k.capitalize()}: water {s[k][WATER]:.0f}%, obstacle {s[k][OBSTACLE]:.0f}%, "
                     f"sky {s[k][SKY]:.0f}%" for k in preds}
        row = np.hstack([panel(rgb, name[:60]),
                         panel(rgb, titles["old"], preds["old"]),
                         panel(rgb, titles["new"], preds["new"])])
        picture = np.vstack([row, legend(row.shape[1])])
        cv2.imwrite(os.path.join(out_dir, name + ".jpg"), cv2.cvtColor(picture, cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        written += 1
    print(f"\n{written} pictures written to {out_dir}")


if __name__ == "__main__":
    main()
