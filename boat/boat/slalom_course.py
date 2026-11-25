"""Geometry and scoring of a slalom race. Plain Python, no ROS, so it can be unit tested.

Everything is in the course frame: metres, origin where the boat starts, x along the line
of buoys, y to port. The boat weaves through the buoys on the way out, rounds the end
buoy and weaves back, as in the 2025 MEBC AI Slalom.
"""

import math
from dataclasses import dataclass

# WGS84, so that GPS offsets match the simulator's to the centimetre over a race course.
WGS84_A = 6378137.0  # m
WGS84_E2 = 6.69437999014e-3


@dataclass(frozen=True)
class Gate:
    """A line across the course at `x`, to be crossed in `direction` with the buoy on one side.

    direction is +1 outbound and -1 on the way back. side is the side of the buoy the boat
    must pass on: +1 port (y > 0), -1 starboard, 0 anywhere.
    """

    name: str
    x: float
    direction: int
    side: int


@dataclass(frozen=True)
class Course:
    """The buoys, the gates in the order they must be passed, and the line to steer along."""

    buoys: list  # (x, y, is_end_buoy)
    gates: list
    path: list  # (x, y), about 0.5 m apart


def build_course(count, spacing, first, start_line, clearance):
    """Build the slalom: `count` buoys `spacing` apart, then an end buoy to round."""
    if count < 1:
        raise ValueError("a slalom needs at least one buoy")
    if not 0.0 < start_line < first:
        raise ValueError("the start line must lie between the boat and the first buoy")
    if not 0.0 < 2.0 * clearance < spacing:
        raise ValueError("clearance must be positive and less than half the buoy spacing")

    xs = [first + i * spacing for i in range(count)]
    x_end = first + count * spacing
    sides = [1 if i % 2 == 0 else -1 for i in range(count)]
    end_side = -sides[-1]  # keep alternating around the end buoy

    buoys = [(x, 0.0, False) for x in xs] + [(x_end, 0.0, True)]
    gates = [Gate("start", start_line, 1, 0)]
    gates += [Gate(f"buoy {i + 1}", xs[i], 1, sides[i]) for i in range(count)]
    gates += [Gate("end out", x_end, 1, end_side), Gate("end back", x_end, -1, -end_side)]
    gates += [Gate(f"buoy {i + 1} back", xs[i], -1, -sides[i]) for i in reversed(range(count))]
    gates += [Gate("finish", start_line, -1, 0)]

    points = [(0.0, 0.0), (start_line, 0.0)]
    points += [(xs[i], sides[i] * clearance) for i in range(count)]
    points += [(x_end, end_side * clearance), (x_end + clearance, 0.0)]
    points += [(x_end, -end_side * clearance)]
    points += [(xs[i], -sides[i] * clearance) for i in reversed(range(count))]
    # Run on past the finish line so the boat crosses it at speed.
    points += [(start_line, 0.0), (start_line - 4.0, 0.0)]
    return Course(buoys, gates, smooth(points))


def smooth(points, step=0.5):
    """Return a Catmull-Rom curve through `points`, sampled about every `step` metres."""
    padded = [points[0], *points, points[-1]]
    curve = []
    for p0, p1, p2, p3 in zip(padded, padded[1:], padded[2:], padded[3:], strict=False):
        n = max(1, round(math.dist(p1, p2) / step))
        for k in range(n):
            t = k / n
            curve.append(
                tuple(
                    0.5
                    * (
                        2 * b
                        + (c - a) * t
                        + (2 * a - 5 * b + 4 * c - d) * t**2
                        + (3 * b - a - 3 * c + d) * t**3
                    )
                    for a, b, c, d in zip(p0, p1, p2, p3, strict=True)
                )
            )
    curve.append(points[-1])
    return curve


class Referee:
    """Watches where the boat goes and scores the gates, independently of the steering."""

    def __init__(self, gates):
        """Start before the first gate."""
        self.gates = gates
        self.index = 0
        self.passed = 0
        self.missed = []
        self.start_time = None
        self.finish_time = None
        self.last = None

    @property
    def started(self):
        """True once the start line has been crossed."""
        return self.start_time is not None

    @property
    def finished(self):
        """True once the finish line has been crossed."""
        return self.finish_time is not None

    def next_gate(self):
        """Return the gate the boat must cross next, or None after the finish."""
        return self.gates[self.index] if self.index < len(self.gates) else None

    def elapsed(self, now):
        """Return the race time in seconds: 0 before the start, frozen after the finish."""
        if not self.started:
            return 0.0
        return (self.finish_time if self.finished else now) - self.start_time

    def update(self, x, y, now):
        """Take the boat's position at time `now` (s); return the gates crossed since last time."""
        crossed = []
        if self.last is not None:
            x0, y0, t0 = self.last
            while (gate := self.next_gate()) is not None:
                before, after = (x0 - gate.x) * gate.direction, (x - gate.x) * gate.direction
                if not before < 0.0 <= after:
                    break
                f = before / (before - after)
                self.score(gate, y0 + f * (y - y0), t0 + f * (now - t0))
                crossed.append(gate)
                self.index += 1
        self.last = (x, y, now)
        return crossed

    def score(self, gate, y, when):
        """Record one crossing at sideways position `y` and time `when`."""
        if self.index == 0:
            self.start_time = when
        elif self.index == len(self.gates) - 1:
            self.finish_time = when
        elif gate.side * y > 0.0:
            self.passed += 1
        else:
            self.missed.append(gate.name)


def pursue(path, start_index, x, y, lookahead, window=40):
    """Return (nearest index, target point) on the path for a boat at (x, y).

    Only searches `window` points ahead of `start_index`: the way out and the way back run
    close together, and the nearest point overall may be on the wrong one.
    """
    end = min(len(path), start_index + window)
    nearest = min(range(start_index, end), key=lambda i: math.dist(path[i], (x, y)))
    target = nearest
    while target < len(path) - 1 and math.dist(path[target], (x, y)) < lookahead:
        target += 1
    return nearest, path[target]


def hits_boat(x, y, yaw, buoy, radius, length, width):
    """Return True if a round buoy overlaps the boat's rectangle (length by width, centred)."""
    dx, dy = buoy[0] - x, buoy[1] - y
    ahead = math.cos(yaw) * dx + math.sin(yaw) * dy
    left = -math.sin(yaw) * dx + math.cos(yaw) * dy
    gap_x = max(abs(ahead) - length / 2.0, 0.0)
    gap_y = max(abs(left) - width / 2.0, 0.0)
    return math.hypot(gap_x, gap_y) < radius


def wrap(angle):
    """Return the angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def metres_per_degree(lat):
    """Return (east, north) metres per degree of longitude and latitude at `lat` degrees."""
    s = math.sin(math.radians(lat))
    w = 1.0 - WGS84_E2 * s * s
    north = WGS84_A * (1.0 - WGS84_E2) / w**1.5
    east = WGS84_A / math.sqrt(w) * math.cos(math.radians(lat))
    return math.radians(east), math.radians(north)


class Anchor:
    """Where the course sits in the world: converts between GPS, local east/north and course."""

    def __init__(self, lat, lon, course_yaw):
        """Put the course origin at (lat, lon), with x pointing at `course_yaw` (rad, 0 = east)."""
        self.lat, self.lon = lat, lon
        self.yaw = course_yaw
        self.m_east, self.m_north = metres_per_degree(lat)

    def to_course(self, lat, lon):
        """Return the course position (x, y) of a GPS position."""
        e, n = (lon - self.lon) * self.m_east, (lat - self.lat) * self.m_north
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * e + s * n, -s * e + c * n

    def to_enu(self, x, y):
        """Return the east/north offset from the origin of a course position."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * x - s * y, s * x + c * y

    def to_gps(self, x, y):
        """Return the (lat, lon) of a course position."""
        e, n = self.to_enu(x, y)
        return self.lat + n / self.m_north, self.lon + e / self.m_east
