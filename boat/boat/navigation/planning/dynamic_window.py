"""Feasible speed and yaw-rate windows for dynamic-window planners."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class VelocityLimits:
    """Physical velocity and acceleration limits in SI units."""

    min_speed: float
    max_speed: float
    min_yaw_rate: float
    max_yaw_rate: float
    max_acceleration: float
    max_deceleration: float
    max_yaw_acceleration: float

    def __post_init__(self):
        values = (
            self.min_speed,
            self.max_speed,
            self.min_yaw_rate,
            self.max_yaw_rate,
            self.max_acceleration,
            self.max_deceleration,
            self.max_yaw_acceleration,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Velocity limits must all be finite")
        if self.min_speed > self.max_speed:
            raise ValueError("min_speed must not exceed max_speed")
        if self.min_yaw_rate > self.max_yaw_rate:
            raise ValueError("min_yaw_rate must not exceed max_yaw_rate")
        if min(self.max_acceleration, self.max_deceleration, self.max_yaw_acceleration) < 0:
            raise ValueError("Acceleration limits must be non-negative")


@dataclass(frozen=True)
class VelocityWindow:
    """Reachable forward-speed and yaw-rate ranges for the next control step."""

    min_speed: float
    max_speed: float
    min_yaw_rate: float
    max_yaw_rate: float

    def sample(self, speed_samples: int, yaw_rate_samples: int) -> list[tuple[float, float]]:
        """Return evenly spaced (speed, yaw_rate) candidates, including endpoints."""
        for count in (speed_samples, yaw_rate_samples):
            if isinstance(count, bool) or not isinstance(count, int) or count < 1:
                raise ValueError("Sample counts must be positive integers")

        speeds = _linspace(self.min_speed, self.max_speed, speed_samples)
        yaw_rates = _linspace(self.min_yaw_rate, self.max_yaw_rate, yaw_rate_samples)
        return [(speed, yaw_rate) for speed in speeds for yaw_rate in yaw_rates]


def compute_dynamic_window(
    current_speed: float,
    current_yaw_rate: float,
    timestep: float,
    limits: VelocityLimits,
) -> VelocityWindow:
    """Intersect the boat's physical limits with velocities reachable this timestep."""
    state = (current_speed, current_yaw_rate, timestep)
    if not all(math.isfinite(value) for value in state):
        raise ValueError("Current velocities and timestep must be finite")
    if timestep <= 0:
        raise ValueError("timestep must be positive")

    min_speed = max(limits.min_speed, current_speed - limits.max_deceleration * timestep)
    max_speed = min(limits.max_speed, current_speed + limits.max_acceleration * timestep)
    min_yaw_rate = max(
        limits.min_yaw_rate, current_yaw_rate - limits.max_yaw_acceleration * timestep
    )
    max_yaw_rate = min(
        limits.max_yaw_rate, current_yaw_rate + limits.max_yaw_acceleration * timestep
    )
    if min_speed > max_speed or min_yaw_rate > max_yaw_rate:
        raise ValueError("Current velocity is outside the configured limits; window is empty")

    return VelocityWindow(min_speed, max_speed, min_yaw_rate, max_yaw_rate)


def _linspace(start: float, stop: float, count: int) -> list[float]:
    if count == 1:
        return [(start + stop) / 2.0]
    step = (stop - start) / (count - 1)
    return [start + index * step for index in range(count)]
