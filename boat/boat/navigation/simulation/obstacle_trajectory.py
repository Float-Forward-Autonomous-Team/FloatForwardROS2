"""Small, deterministic waypoint trajectories for collidable simulator obstacles."""

import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class MovingObstacle:
    """A model moving at constant speed along XY waypoints in Gazebo world metres."""

    name: str
    classification: str
    radius_m: float
    speed_mps: float
    waypoints_xy: tuple[tuple[float, float], ...]
    loop: bool = True

    def __post_init__(self):
        """Reject trajectories that cannot be spawned or traversed."""
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", self.name):
            raise ValueError(f"Invalid Gazebo model name: {self.name}")
        if self.radius_m <= 0 or self.speed_mps <= 0:
            raise ValueError(f"{self.name}: radius and speed must be positive")
        if len(self.waypoints_xy) < 2:
            raise ValueError(f"{self.name}: at least two waypoints are required")
        if all(a == b for a, b in zip(self.waypoints_xy, self.waypoints_xy[1:])):
            raise ValueError(f"{self.name}: waypoints must define a nonzero path")

    def position_velocity(self, elapsed_sec):
        """Return (x, y, vx, vy) at time elapsed since obstacle start."""
        points = self.waypoints_xy
        pairs = list(zip(points, points[1:]))
        if self.loop:
            pairs.append((points[-1], points[0]))
        segments = [(a, b, math.dist(a, b)) for a, b in pairs if a != b]
        total = sum(length for _, _, length in segments)
        distance = max(0.0, elapsed_sec) * self.speed_mps
        if self.loop:
            distance %= total
        elif distance >= total:
            x, y = points[-1]
            return x, y, 0.0, 0.0
        for (ax, ay), (bx, by), length in segments:
            if distance <= length:
                fraction = distance / length
                return (
                    ax + (bx - ax) * fraction,
                    ay + (by - ay) * fraction,
                    self.speed_mps * (bx - ax) / length,
                    self.speed_mps * (by - ay) / length,
                )
            distance -= length
        raise RuntimeError("No trajectory segment selected")


def load_obstacles(path):
    """Parse a small trajectory YAML; an empty path disables moving obstacles."""
    if not path:
        return []
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    obstacles = [
        MovingObstacle(
            name=str(item["name"]),
            classification=str(item.get("classification", "simulated_obstacle")),
            radius_m=float(item["radius_m"]),
            speed_mps=float(item["speed_mps"]),
            waypoints_xy=tuple(tuple(map(float, point)) for point in item["waypoints_xy"]),
            loop=bool(item.get("loop", True)),
        )
        for item in data.get("obstacles", [])
    ]
    names = [obstacle.name for obstacle in obstacles]
    if len(names) != len(set(names)):
        raise ValueError("Obstacle names must be unique")
    return obstacles
