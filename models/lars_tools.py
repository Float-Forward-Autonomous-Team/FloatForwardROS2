"""Shared helpers for the LaRS scripts.

Used by evaluate_lars.py, evaluate_yolo_lars.py, finetune_ewasr.py and finetune_yolo.py, so they all
find the files, read the scene conditions and split the training frames in exactly the same way.
"""

import json
import os
import random

# the Monaco-like conditions the fine-tuning focuses on, as tags read from image_annotations.json
FOCUS_TAGS = ("waves=still", "reflections=heavy", "extra_bright")
# share of the LaRS training frames held out for testing, and the seed that makes the split repeatable
HOLDOUT_SHARE = 0.2
SPLIT_SEED = 0

# fields in image_annotations.json that name or describe the image rather than its conditions
NAME_KEYS = {"file_name", "filename", "image", "image_name", "name", "id", "image_id"}
SKIP_KEYS = {"width", "height", "license", "url", "coco_url", "flickr_url", "date_captured",
             "segments_info", "sequence", "seq_name", "frame", "frame_id", "split"}


def stem(name):
    return os.path.splitext(os.path.basename(str(name)))[0]


def video_of(name):
    """LaRS names each frame <video name>_<frame number>; frames of one video look nearly the same."""
    parts = str(name).split("_")
    return "_".join(parts[:-1]) if len(parts) > 1 else str(name)


def find_split(root, split):
    """Finds <split>/images and <split>/semantic_masks anywhere under root (the zips add top folders)."""
    images = masks = None
    for path, dirs, _ in os.walk(root):
        if os.path.basename(path) != split:
            continue
        if images is None and "images" in dirs:
            images = os.path.join(path, "images")
        if masks is None and "semantic_masks" in dirs:
            masks = os.path.join(path, "semantic_masks")
    return images, masks


def read_names(path):
    with open(path) as f:
        return [line.strip() for line in f if line.strip()]


def write_names(path, names):
    with open(path, "w") as f:
        f.write("\n".join(names) + "\n")


def list_frames(root, split, frames_file=None, limit=0):
    """The frames of one LaRS split that have both an image and a label mask.

    Returns a dict: names (in LaRS order), image_of (name -> image path), images_dir, masks_dir,
    split_dir (the folder with image_list.txt and the annotation files)."""
    images_dir, masks_dir = find_split(root, split)
    if not images_dir or not masks_dir:
        raise SystemExit(f"Could not find {split}/images and {split}/semantic_masks under {root}. "
                         "Unzip both lars_v1.0.0_images.zip and lars_v1.0.0_annotations.zip there.")
    split_dir = os.path.dirname(masks_dir)
    list_file = os.path.join(split_dir, "image_list.txt")
    if os.path.exists(list_file):
        names = read_names(list_file)
    else:
        names = sorted(stem(f) for f in os.listdir(masks_dir) if f.endswith(".png"))
    image_of = {stem(f): os.path.join(images_dir, f) for f in os.listdir(images_dir)}
    names = [n for n in names if n in image_of and os.path.exists(os.path.join(masks_dir, n + ".png"))]
    if frames_file:
        wanted = set(read_names(frames_file))
        names = [n for n in names if n in wanted]
    if limit:
        names = names[:limit]
    if not names:
        raise SystemExit("Found the folders but no frame with both an image and a mask"
                         + (f" among the frames in {frames_file}." if frames_file else "."))
    return {"names": names, "image_of": image_of, "images_dir": images_dir,
            "masks_dir": masks_dir, "split_dir": split_dir}


def read_label_mask(path):
    """LaRS semantic mask as class ids: 0 obstacle, 1 water, 2 sky, 255 ignore."""
    try:
        from PIL import Image
        import numpy as np
        mask = np.array(Image.open(path))
    except ImportError:
        import cv2
        mask = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    return mask[..., 0] if mask.ndim == 3 else mask


def tags_of(value, key=""):
    """Turns one image's attributes into readable tags such as 'lighting=dusk_like' or 'glitter'."""
    tags = set()
    if isinstance(value, dict):
        for k, v in value.items():
            if k not in NAME_KEYS and k not in SKIP_KEYS:
                tags |= tags_of(v, k)
    elif isinstance(value, (list, tuple)):
        for v in value:
            tags |= tags_of(v, key)
    elif isinstance(value, bool):
        if value and key:
            tags.add(key)
    elif value is not None:
        text = str(value).strip()
        if text.lower() in ("yes", "true"):
            if key:
                tags.add(key)
        elif text.lower() not in ("no", "false", "none", ""):
            tags.add(f"{key}={text}" if key else text)
    return tags


def load_conditions(path, names):
    """Reads image_annotations.json into {image name: tags}. Copes with a dict keyed by image,
    a list of per-image records, or COCO-style 'images' plus 'annotations' lists."""
    with open(path) as f:
        data = json.load(f)
    names = set(names)
    result = {}
    if isinstance(data, dict) and names & {stem(k) for k in data}:
        for key, attrs in data.items():
            result[stem(key)] = tags_of(attrs)
        return {n: t for n, t in result.items() if n in names}

    lists = [data] if isinstance(data, list) else [v for v in data.values() if isinstance(v, list)]
    id_to_name = {}
    if isinstance(data, dict) and isinstance(data.get("images"), list):
        for img in data["images"]:
            if isinstance(img, dict) and "id" in img:
                id_to_name[img["id"]] = stem(img.get("file_name", img.get("name", img["id"])))
    for records in lists:
        for rec in records:
            if not isinstance(rec, dict):
                continue
            name = next((stem(rec[k]) for k in ("file_name", "filename", "image_name", "image", "name")
                         if isinstance(rec.get(k), (str, int))), None)
            if name not in names and rec.get("image_id") in id_to_name:
                name = id_to_name[rec["image_id"]]
            if name in names:
                result.setdefault(name, set()).update(tags_of(rec))
    return result


def conditions_for(split_dir, names):
    """Scene conditions of the frames, from image_annotations.json in the split folder or above it."""
    for candidate in (os.path.join(split_dir, "image_annotations.json"),
                      os.path.join(os.path.dirname(split_dir), "image_annotations.json")):
        if os.path.exists(candidate):
            return load_conditions(candidate, names)
    return {}


def make_or_load_split(root, split_dir=None, share=HOLDOUT_SHARE, seed=SPLIT_SEED):
    """Splits the LaRS training frames by video into a part to train on and a part held out for testing.

    Whole videos go to one side, so no near-copy of a test frame is ever trained on. Videos with a
    Monaco-like frame (FOCUS_TAGS) and the other videos are split separately, so both kinds are held
    out in the same share. The split is written once to split_dir (train.txt, holdout.txt) and read
    back on every later run, so every model is tested on the same frames."""
    split_dir = split_dir or os.path.join(os.path.dirname(os.path.abspath(root.rstrip("/\\"))), "LaRS_split")
    train_file, holdout_file = os.path.join(split_dir, "train.txt"), os.path.join(split_dir, "holdout.txt")
    frames = list_frames(root, "train")
    if os.path.exists(train_file) and os.path.exists(holdout_file):
        train, holdout = read_names(train_file), read_names(holdout_file)
        missing = set(train + holdout) - set(frames["names"])
        if missing:
            print(f"  note: {len(missing)} frames in the saved split are not in this LaRS folder")
        return train, holdout, split_dir

    names = frames["names"]
    conditions = conditions_for(frames["split_dir"], names)
    videos = {}
    for n in names:
        videos.setdefault(video_of(n), []).append(n)
    focus = sorted(v for v, members in videos.items()
                   if any(t in conditions.get(m, ()) for m in members for t in FOCUS_TAGS))
    normal = sorted(v for v in videos if v not in set(focus))
    rng = random.Random(seed)
    held = set()
    for group in (focus, normal):
        order = group[:]
        rng.shuffle(order)
        total = sum(len(videos[v]) for v in group)
        taken = 0
        for v in order:
            if taken >= share * total:
                break
            held.add(v)
            taken += len(videos[v])
    train = [n for n in names if video_of(n) not in held]
    holdout = [n for n in names if video_of(n) in held]
    os.makedirs(split_dir, exist_ok=True)
    write_names(train_file, train)
    write_names(holdout_file, holdout)
    print(f"  split by video: {len(train)} training frames, {len(holdout)} held out "
          f"({len(held)} of {len(videos)} videos), saved in {split_dir}")
    return train, holdout, split_dir


def load_objects(split_dir, names):
    """Labelled objects (boats, buoys, ...) per frame from LaRS's panoptic_annotations.json.

    Returns (objects, sizes, static_id): objects = {name: [{'id', 'category', 'group', 'box' (x1, y1, x2, y2),
    'crowd'}]}, sizes = {name: (width, height)}, static_id = the class id of static obstacles (shore,
    piers) in the red channel of the panoptic masks. 'id' is the object's segment id in the panoptic mask
    (red + 256 * green + 65536 * blue)."""
    path = os.path.join(split_dir, "panoptic_annotations.json")
    if not os.path.exists(path):
        raise SystemExit(f"{path} not found: it comes in lars_v1.0.0_annotations.zip.")
    with open(path) as f:
        data = json.load(f)
    categories = {c["id"]: c for c in data.get("categories", [])}
    static_id = next((c["id"] for c in categories.values()
                      if not c.get("isthing") and "static" in c.get("name", "").lower()), 1)
    image_name = {img["id"]: stem(img["file_name"]) for img in data.get("images", [])}
    sizes = {stem(img["file_name"]): (img.get("width"), img.get("height")) for img in data.get("images", [])}
    wanted = set(names)
    objects = {n: [] for n in names}
    for ann in data.get("annotations", []):
        name = image_name.get(ann.get("image_id"), stem(ann.get("file_name", "")))
        if name not in wanted:
            continue
        for seg in ann.get("segments_info", []):
            cat = categories.get(seg.get("category_id"), {})
            if not cat.get("isthing") or "bbox" not in seg:
                continue
            x, y, w, h = seg["bbox"]
            objects[name].append({"id": seg.get("id"), "category": cat.get("name", str(seg["category_id"])),
                                  "group": group_of(cat.get("name", "")),
                                  "box": (float(x), float(y), float(x + w), float(y + h)),
                                  "crowd": bool(seg.get("iscrowd", 0))})
    return objects, sizes, static_id


def group_of(category_name):
    """boat (also row boat), buoy, or other (paddle board, swimmer, animal, float, other)."""
    n = category_name.lower()
    if "buoy" in n:
        return "buoy"
    if "boat" in n or "ship" in n:
        return "boat"
    return "other"
