"""Small geometry helpers for paths given as lists of (x, y) points."""

import math


def distance_to_polyline(point, polyline):
    """Return the shortest distance from a point to a polyline of (x, y) vertices."""
    if len(polyline) == 1:
        return math.dist(point, polyline[0])
    px, py = point
    best = math.inf
    for (ax, ay), (bx, by) in zip(polyline, polyline[1:]):
        dx, dy = bx - ax, by - ay
        length_squared = dx * dx + dy * dy
        t = 0.0
        if length_squared > 0:
            t = min(1.0, max(0.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best
