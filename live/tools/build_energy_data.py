"""Turn the Monaco 2026 race CSVs into the small data file the public live page replays.

Keeps only what the public page may show: time, position, speed and heading.
Battery, motor, thrust and fault columns are left out on purpose.

Usage: python3 live/tools/build_energy_data.py <race_data folder> [output.json]
The race data lives in DTU Float/Telemetry/Database2026/race_data.
"""

import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path

RACES = [
    ("telemetry_08_07_2026_lapstest.csv", "08/07", "Laps test"),
    ("telemetry_09_07_2026_speedrecord.csv", "09/07", "Speed record"),
    ("telemetry_10_07_2026_endurance_race.csv", "10/07", "Endurance race"),
    ("telemetry_11_07_2026_slalom_race.csv", "11/07", "Slalom race"),
]
MIN_STEP_S = 1.0  # at most one fix per second
MIN_MOVE_FOR_HEADING_M = 3.0  # below this the computed bearing is GPS noise


def distance_m(lat1, lon1, lat2, lon2):
    """Return the flat-earth distance in metres between two nearby positions."""
    k = 111_320.0
    dx = (lon2 - lon1) * k * math.cos(math.radians((lat1 + lat2) / 2))
    dy = (lat2 - lat1) * k
    return math.hypot(dx, dy)


def bearing_deg(lat1, lon1, lat2, lon2):
    """Return the compass bearing in degrees from the first position to the second."""
    dx = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    dy = lat2 - lat1
    return (math.degrees(math.atan2(dx, dy)) + 360) % 360


def load_fixes(path):
    """Rows with a position, thinned to one per MIN_STEP_S, as [t, lat, lon, speed, heading]."""
    fixes = []
    last_speed = None
    with open(path) as fh:
        for row in csv.DictReader(fh):
            if row.get("gps_speed"):
                last_speed = float(row["gps_speed"])
            if not (row.get("gps_latitude") and row.get("gps_longitude")):
                continue
            if row.get("gps_valid") == "0.0":
                continue
            t = datetime.fromisoformat(row["timestamp"].replace("Z", "+00:00")).timestamp()
            if fixes and t - fixes[-1][0] < MIN_STEP_S:
                continue
            heading = float(row["gps_heading"]) if row.get("gps_heading") else None
            fixes.append(
                [t, float(row["gps_latitude"]), float(row["gps_longitude"]), last_speed, heading]
            )
    return fixes


def fill_headings(fixes):
    """Where the receiver gave no heading, use the bearing from the last fix far enough away."""
    computed = 0
    anchor = None
    current = None
    for fix in fixes:
        if fix[4] is not None:
            current = fix[4]
            anchor = fix
            continue
        if anchor is None:
            anchor = fix
        elif distance_m(anchor[1], anchor[2], fix[1], fix[2]) >= MIN_MOVE_FOR_HEADING_M:
            current = bearing_deg(anchor[1], anchor[2], fix[1], fix[2])
            anchor = fix
        fix[4] = current
        computed += 1
    return computed


def main():
    """Build the replay file from the race CSVs in the given folder."""
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    src = Path(sys.argv[1])
    out = (
        Path(sys.argv[2])
        if len(sys.argv) > 2
        else Path(__file__).resolve().parent.parent / "data" / "energy_monaco_2026.json"
    )

    races = []
    for filename, day, name in RACES:
        print(f"Reading {filename} ...")
        fixes = load_fixes(src / filename)
        computed = fill_headings(fixes)
        top = max((f[3] or 0) for f in fixes)
        print(f"  {len(fixes)} fixes, {computed} headings computed, top speed {top:.1f} m/s")
        t0 = fixes[0][0]
        races.append(
            {
                "id": filename.split("_2026_")[1].removesuffix(".csv"),
                "day": day,
                "name": name,
                "start": t0,
                # [seconds since start, lat, lon, speed m/s, heading deg] keeps the file small
                "fixes": [
                    [
                        round(t - t0, 1),
                        round(lat, 6),
                        round(lon, 6),
                        None if s is None else round(s, 2),
                        None if h is None else round(h),
                    ]
                    for t, lat, lon, s, h in fixes
                ],
            }
        )

    folder = f"{src.resolve().parent.name}/{src.resolve().name}"
    out.write_text(
        json.dumps(
            {
                "source": f"DTU Float telemetry, Monaco 8-11 July 2026 ({folder})",
                "races": races,
            },
            separators=(",", ":"),
        )
    )
    print(f"Wrote {out} ({out.stat().st_size / 1024:.0f} KB)")


if __name__ == "__main__":
    main()
