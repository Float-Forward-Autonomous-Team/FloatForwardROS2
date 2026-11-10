#!/usr/bin/env python3
"""Spawn a small course of MEBC-style buoys in front of the WAM-V, for perception tests.

Run inside the container while `make vrx` is running (after ScoringPlugin::OnRunning):
    python3 /ws/src/boat/sim_models/spawn_test_course.py
Options:
    --line              all five buoys side by side in one row, facing the boat
    --at 15             with --line: how far ahead the row is (metres, default 15)
    --spacing 3         with --line: gap between buoys (metres, default 3)
    --distance 1.5      without --line: scale every distance (1.5 = course 50% further away)
    --remove            remove every buoy this script spawned, then exit
    --x X --y Y --yaw YAW   use this boat pose instead of asking Gazebo

Positions are in the boat's frame: forward metres, left metres (negative = right).
Edit COURSE below to change the layout. Spawned buoys disappear when the sim restarts.
"""

import argparse
import math
import re
import subprocess
import sys
from pathlib import Path

MODELS = Path(__file__).resolve().parent

COURSE = [  # (model file, forward m, left m)
    ("slalom_buoy", 12, 3),
    ("slalom_buoy", 12, -3),
    ("green_buoy", 20, 5),
    ("mooring_buoy", 22, -5),
    ("orange_float", 28, 2),
    ("white_float", 28, -1),
    ("slalom_buoy", 45, 0),  # far one, to see at what distance detection starts
]

LINE = [
    "green_buoy",
    "slalom_buoy",
    "mooring_buoy",
    "orange_float",
    "white_float",
]  # left to right


def remove_spawned(world):
    """Remove every model whose name starts with test_ (the ones this script made)."""
    out = run(["gz", "model", "--list"])
    names = sorted(set(re.findall(r"\b(test_[A-Za-z0-9_]+)", out)))
    for n in names:
        run(
            [
                "gz",
                "service",
                "-s",
                f"/world/{world}/remove",
                "--reqtype",
                "gz.msgs.Entity",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "3000",
                "--req",
                f'name: "{n}", type: MODEL',
            ]
        )
        print(f"  removed {n}")
    if not names:
        print("  nothing to remove")


def run(cmd):
    """Run a command and return what it printed."""
    return subprocess.run(cmd, capture_output=True, text=True, timeout=30).stdout


def find_world():
    """Return the world's real (internal) name, read from its /world/<name>/create service."""
    names = re.findall(r"^/world/([^/]+)/create$", run(["gz", "service", "-l"]), re.M)
    if not names:
        sys.exit("No Gazebo world found. Is `make vrx` running and past ScoringPlugin::OnRunning?")
    return names[0]


def boat_pose():
    """x, y, yaw of the WAM-V, parsed from `gz model -m wamv -p`."""
    out = run(["gz", "model", "-m", "wamv", "-p"])
    triples = re.findall(r"\[\s*(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)\s*\]", out)
    if len(triples) < 2:
        sys.exit("Could not read the boat's pose. Pass it by hand: --x X --y Y --yaw YAW\n" + out)
    (x, y, _), (_, _, yaw) = triples[0], triples[1]
    return float(x), float(y), float(yaw)


def main():
    """Spawn the course (or the line), or remove the spawned buoys."""
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--line", action="store_true")
    p.add_argument("--at", type=float, default=15.0)
    p.add_argument("--spacing", type=float, default=3.0)
    p.add_argument("--distance", type=float, default=1.0)
    p.add_argument("--remove", action="store_true")
    p.add_argument("--x", type=float)
    p.add_argument("--y", type=float)
    p.add_argument("--yaw", type=float)
    a = p.parse_args()

    world = find_world()
    if a.remove:
        remove_spawned(world)
        return
    if a.x is not None and a.y is not None and a.yaw is not None:
        bx, by, yaw = a.x, a.y, a.yaw
    else:
        bx, by, yaw = boat_pose()
    print(f"World: {world}   boat at x={bx:.1f} y={by:.1f} heading={math.degrees(yaw):.0f} deg")

    if a.line:
        half = (len(LINE) - 1) / 2
        layout = [(m, a.at, (half - i) * a.spacing) for i, m in enumerate(LINE)]
    else:
        layout = [(m, fwd * a.distance, left) for m, fwd, left in COURSE]

    for i, (model, fwd, left) in enumerate(layout):
        x = bx + fwd * math.cos(yaw) - left * math.sin(yaw)
        y = by + fwd * math.sin(yaw) + left * math.cos(yaw)
        name = f"test_{model}_{i}"
        subprocess.run(
            [
                "ros2",
                "run",
                "ros_gz_sim",
                "create",
                "-world",
                world,
                "-file",
                str(MODELS / f"{model}.sdf"),
                "-name",
                name,
                "-allow_renaming",
                "true",
                "-x",
                f"{x:.2f}",
                "-y",
                f"{y:.2f}",
                "-z",
                "0",
                "-Y",
                f"{yaw:.3f}",
            ],
            capture_output=True,
            timeout=60,
        )
        side = "left" if left > 0 else "right" if left < 0 else "ahead"
        print(f"  spawned {model:13s} {fwd:5.1f} m ahead, {abs(left):.0f} m {side}")


if __name__ == "__main__":
    main()
