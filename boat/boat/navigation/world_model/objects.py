"""Tracked-object prediction and the rule that turns stopped objects into static obstacles."""

import math
from dataclasses import dataclass


def predict(track, observed_at, at_time):
    """Return the constant-velocity (x, y) of a track at `at_time` seconds."""
    dt = at_time - observed_at
    return (
        track.pose.position.x + track.twist.linear.x * dt,
        track.pose.position.y + track.twist.linear.y * dt,
    )


@dataclass(frozen=True)
class StaticObstacle:
    """A stopped object, held in the static map where it stopped."""

    x: float
    y: float
    radius_m: float


class StationaryFilter:
    """Promote objects that stay slow for a hold time; demote them when they move again.

    A promoted object stays static while it is missing from later snapshots,
    since a stopped obstacle outside Perception's view is still there.
    """

    def __init__(self, speed_threshold_mps, hold_time_s):
        """Configure the speed below which an object counts as stopped, and for how long."""
        if speed_threshold_mps < 0 or hold_time_s < 0:
            raise ValueError("Speed threshold and hold time must be non-negative")
        self.speed_threshold = speed_threshold_mps
        self.hold_time = hold_time_s
        self.static = {}
        self._still_since = {}

    def update(self, tracks, stamp):
        """Apply one object snapshot taken at `stamp` seconds; return True if `static` changed."""
        changed = False
        seen = set()
        for track in tracks:
            seen.add(track.id)
            x, y = track.pose.position.x, track.pose.position.y
            speed = math.hypot(track.twist.linear.x, track.twist.linear.y)
            stopped = self.static.get(track.id)
            if stopped is not None:
                moved = math.hypot(x - stopped.x, y - stopped.y) > stopped.radius_m
                if speed > self.speed_threshold or moved:
                    del self.static[track.id]
                    changed = True
                continue
            if speed > self.speed_threshold:
                self._still_since.pop(track.id, None)
                continue
            since = self._still_since.setdefault(track.id, stamp)
            if since > stamp:  # simulation reset
                since = self._still_since[track.id] = stamp
            if stamp - since >= self.hold_time:
                self.static[track.id] = StaticObstacle(x, y, track.radius_m)
                del self._still_since[track.id]
                changed = True
        for track_id in set(self._still_since) - seen:
            del self._still_since[track_id]
        return changed

    def moving(self, tracks):
        """Return the tracks that are not held in the static layer."""
        return [track for track in tracks if track.id not in self.static]
