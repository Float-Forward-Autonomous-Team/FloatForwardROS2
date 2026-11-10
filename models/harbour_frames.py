"""Before/after pictures on harbour-like LaRS frames: frames with several labelled boats or ships.

Only frames the new model never trained on are used: the held-out training frames and the LaRS
validation frames. For each of the two, compare_frames.py makes the same list and four-panel pictures
as before, ending with one line for all the chosen frames together.

Run inside the container, from this folder:
    cd /ws/src/boat/models
    python3 harbour_frames.py --lars /ws/src/boat/datasets/LaRS

The pictures go to datasets/compare/harbour_heldout and datasets/compare/harbour_val.

Frames you picked yourself: this says for each one whether it is a fair test, and makes pictures of
the ones that are (in datasets/compare/chosen_heldout and chosen_val):
    python3 harbour_frames.py --lars /ws/src/boat/datasets/LaRS --names inhouse_seq_28_00030,inhouse_seq_215_00179

Options:
    --min-boats 3       a frame counts as harbour-like with at least this many labelled boats or ships;
                        row boats and kayaks do not count, so rivers with canoes are left out
    --condition TAG     also require a scene condition, e.g. glitter or reflections=heavy
    --names A,B         use these frames instead of picking by boats. Each one is reported as validation
                        or held-out (a fair test), or training: the new model learned from those, so they
                        get no picture
    --sort improved     improved (default): frames where the metric dropped most come first;
                        worse: frames where it grew most come first
    --metric false      false (water called obstacle, default) or missed (obstacle called water)
    --count 8           pictures per part; every chosen frame is listed either way
    --old / --new       the two ONNX models (defaults as in compare_frames.py)
    --holdout FILE      the list of held-out frames (default: LaRS_split/holdout.txt next to the LaRS folder)

A run with other --sort or --metric settings writes to its own folders (e.g. harbour_heldout_worse_missed),
so pictures from different runs don't mix.
"""

import argparse
import os
import subprocess
import sys

import lars_tools as LT

HERE = os.path.dirname(os.path.abspath(__file__))


def is_boat(obj):
    """Boats and ships. Row boats (kayaks, canoes) are left out."""
    name = str(obj.get("category", "")).lower().strip()
    return "ship" in name or name in ("boat", "boats")


def load_part(lars, split, frames_file):
    """Frame names, labelled objects and scene conditions of one part of LaRS."""
    frames = LT.list_frames(lars, split, frames_file)
    names = list(frames["names"])
    objects, _, _ = LT.load_objects(frames["split_dir"], names)
    conditions = LT.conditions_for(frames["split_dir"], names)
    return names, objects, conditions


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lars", required=True, help="folder you unzipped LaRS into")
    parser.add_argument("--min-boats", type=int, default=3)
    parser.add_argument("--condition")
    parser.add_argument("--names")
    parser.add_argument("--sort", default="improved", choices=["improved", "worse"])
    parser.add_argument("--metric", default="false", choices=["false", "missed"])
    parser.add_argument("--count", type=int, default=8)
    parser.add_argument("--old")
    parser.add_argument("--new")
    parser.add_argument("--holdout")
    args = parser.parse_args()

    lars = os.path.abspath(args.lars.rstrip("/\\"))
    data_dir = os.path.dirname(lars)
    holdout = os.path.abspath(args.holdout or os.path.join(data_dir, "LaRS_split", "holdout.txt"))
    if not os.path.isfile(holdout):
        raise SystemExit(f"No list of held-out frames at {holdout}. finetune_ewasr.py writes it; "
                         "give its path with --holdout.")

    # the two parts the new model never trained on: (description, LaRS split, frames file)
    parts = {"heldout": ("held-out training frames", "train", holdout),
             "val": ("validation frames", "val", None)}
    data = {key: load_part(lars, split, frames_file) for key, (_, split, frames_file) in parts.items()}

    if args.names:
        wanted = [os.path.splitext(n.strip())[0] for n in args.names.split(",") if n.strip()]
        held, val = set(data["heldout"][0]), set(data["val"][0])
        train_all = set(LT.list_frames(lars, "train", None)["names"])
        print("Where each frame is:")
        for n in wanted:
            if n in val:
                where = "validation: a fair test"
            elif n in held:
                where = "held-out: a fair test"
            elif n in train_all:
                where = "training: the new model learned from it, so it is not a fair test (no picture)"
            else:
                where = "not found in LaRS train or val"
            print(f"  {n:<34} {where}")
        chosen = {"heldout": [n for n in wanted if n in held], "val": [n for n in wanted if n in val]}
        tag = "chosen"
    else:
        if args.condition:
            tags = {t for key in data for frame_tags in data[key][2].values() for t in frame_tags}
            if args.condition not in tags:
                raise SystemExit("No frames have that condition. Conditions: " + ", ".join(sorted(tags)))
        rule = f"at least {args.min_boats} boats or ships" + (f" and {args.condition}" if args.condition else "")
        chosen = {}
        for key, (label, _, _) in parts.items():
            names, objects, conditions = data[key]
            chosen[key] = [n for n in names
                           if sum(is_boat(o) for o in objects.get(n, [])) >= args.min_boats
                           and (not args.condition or args.condition in conditions.get(n, ()))]
            print(f"{label}: {len(chosen[key])} of {len(names)} frames have {rule}")
        if not any(chosen.values()):
            categories = {str(o.get("category")) for key in data for objs in data[key][1].values() for o in objs}
            raise SystemExit("No frame matches. Object categories in these frames: " + ", ".join(sorted(categories)))
        tag = "harbour" + (f"_{args.condition.replace('=', '_')}" if args.condition else "")
    if (args.sort, args.metric) != ("improved", "false"):
        tag += f"_{args.sort}_{args.metric}"

    for key, (label, split, frames_file) in parts.items():
        if not chosen[key]:
            print(f"\nNo {label} to show.")
            continue
        cmd = [sys.executable, os.path.join(HERE, "compare_frames.py"), "--lars", lars, "--split", split,
               "--names", ",".join(chosen[key]), "--sort", args.sort, "--metric", args.metric,
               "--count", str(args.count), "--out", os.path.join(data_dir, "compare", f"{tag}_{key}")]
        if frames_file:
            cmd += ["--frames-file", frames_file]
        for flag, value in (("--old", args.old), ("--new", args.new)):
            if value:
                cmd += [flag, value]
        print(f"\n=== {label} ===", flush=True)
        if subprocess.run(cmd).returncode:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
