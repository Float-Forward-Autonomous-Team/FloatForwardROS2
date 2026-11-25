"""Tests for the race scoreboard picture."""

from io import BytesIO

import pytest
from PIL import Image

from boat.scoreboard import BAD, GOOD, SIZE, cells, draw_scoreboard

VALUES = {
    "elapsed": 93.4,
    "speed_kn": 4.6,
    "next_gate_m": 12.0,
    "passed": 10,
    "missed": 0,
    "touched": 0,
}


@pytest.mark.parametrize(
    "state", ["WAITING", "RACING: next buoy 4 back", "STOPPED: no GPS or IMU"]
)
def test_draws_every_state(state):
    """Every state gives a readable JPEG of the right size."""
    image = Image.open(BytesIO(draw_scoreboard(state, VALUES, 10)))
    assert image.size == SIZE


def test_misses_and_touches_turn_red():
    """Mistakes stand out, a clean run is green."""
    clean = {c[0]: c for c in cells("FINISHED", VALUES, 10)}
    assert clean["MISSED"][2] == clean["TOUCHED"][2] == clean["STATE"][2] == GOOD
    bad = {c[0]: c for c in cells("FINISHED", {**VALUES, "missed": 1, "touched": 2}, 10)}
    assert bad["MISSED"][2] == bad["TOUCHED"][2] == BAD
    assert bad["GATES"][1] == "10 / 10"
