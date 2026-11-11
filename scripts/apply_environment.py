"""Copy a VRX world, applying development settings to the copy."""

import argparse
import math
import os
import re
import sys
from pathlib import Path

import yaml


def find_world(world):
    """Return the path of `world`: an .sdf file, or the name of a VRX world."""
    if os.path.isfile(world):
        return Path(world)
    from ament_index_python.packages import get_package_share_directory

    worlds = Path(get_package_share_directory("vrx_gz")) / "worlds"
    found = sorted(worlds.rglob(f"{world}.sdf"))
    if not found:
        sys.exit(f"apply_environment: no world named '{world}' under {worlds}")
    return found[0]


def substitute(sdf, pattern, value, what):
    """Replace the single match of `pattern`'s group 2 with `value`, failing otherwise."""
    sdf, count = re.subn(pattern, rf"\g<1>{value}\g<3>", sdf)
    if count != 1:
        sys.exit(f"apply_environment: expected one {what} in the world, found {count}")
    return sdf


def set_tag(sdf, tag, value):
    """Set the text of the world's single `<tag>` element."""
    return substitute(sdf, rf"(<{tag}>)([^<]*)(</{tag}>)", value, f"<{tag}>")


def set_wave_param(sdf, key, value):
    """Set one parameter of the wave field message published by the world."""
    pattern = rf'(key: "{key}"\s+value \{{\s+type: DOUBLE\s+double_value: )([^\s]+)(\s)'
    return substitute(sdf, pattern, value, f'wave parameter "{key}"')


def keep_running_after_task(sdf):
    """Keep VRX scoring active without a deadline or simulator shutdown."""
    plugin_pattern = re.compile(
        r'(<plugin\b[^>]*\bfilename="[^"]*ScoringPlugin\.so"[^>]*>)(.*?)(</plugin>)',
        re.DOTALL,
    )

    def configure_plugin(match):
        body = match.group(2)
        for tag, value in (
            ("running_state_duration", "inf"),
            ("exit_delay", "inf"),
            ("per_plugin_exit_on_completion", "false"),
        ):
            pattern = rf"(<{tag}>)[^<]*(</{tag}>)"
            if re.search(pattern, body):
                body = re.sub(
                    pattern,
                    lambda found: f"{found.group(1)}{value}{found.group(2)}",
                    body,
                )
            else:
                body = body.rstrip() + f"\n      <{tag}>{value}</{tag}>\n    "
        return match.group(1) + body + match.group(3)

    sdf, count = plugin_pattern.subn(configure_plugin, sdf)
    if count == 0:
        print("apply_environment: no VRX scoring plugin found; world is already unbounded")
    return sdf


def main():
    """Apply the environment file to the world given on the command line."""
    parser = argparse.ArgumentParser()
    parser.add_argument("environment", help="environment.yaml")
    parser.add_argument("world", help="VRX world name or .sdf path")
    parser.add_argument("out_dir", help="directory for the generated world")
    parser.add_argument(
        "--keep-running",
        action="store_true",
        help="keep scoring worlds running until Gazebo is stopped manually",
    )
    args = parser.parse_args()
    env_path, world, out_dir = args.environment, args.world, args.out_dir

    with open(env_path) as f:
        env = yaml.safe_load(f)
    wind, waves = env["wind"], env["waves"]

    world_path = find_world(world)
    sdf = world_path.read_text()
    sdf = set_tag(sdf, "wind_mean_velocity", wind["speed"])
    sdf = set_tag(sdf, "wind_direction", wind["direction"])
    sdf = set_tag(sdf, "var_wind_gain_constants", wind["gust_strength"])
    sdf = set_tag(sdf, "var_wind_time_constants", wind["gust_time"])
    sdf = set_wave_param(sdf, "gain", waves["gain"])
    sdf = set_wave_param(sdf, "period", waves["period"])
    sdf = set_wave_param(sdf, "direction", math.radians(waves["direction"]))
    if args.keep_running:
        sdf = keep_running_after_task(sdf)

    # Same file name as the original: VRX derives its bridge topics from it.
    out_path = Path(out_dir) / world_path.name
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(sdf)
    print(f"apply_environment: wrote {env_path} into {out_path}")


if __name__ == "__main__":
    main()
