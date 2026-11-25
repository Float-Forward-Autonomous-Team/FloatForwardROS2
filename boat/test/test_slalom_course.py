"""Tests for the slalom geometry and scoring."""

import math

import pytest

from boat.slalom_course import Anchor, Referee, build_course, hits_boat, pursue, wrap

ARGS = {"count": 4, "spacing": 22.0, "first": 20.0, "start_line": 8.0, "clearance": 4.0}


def run(course, points, dt=0.1):
    """Feed the points to a referee, one every dt seconds."""
    referee = Referee(course.gates)
    for i, (x, y) in enumerate(points):
        referee.update(x, y, i * dt)
    return referee


def test_following_the_path_passes_every_gate():
    """The path itself is a clean run."""
    course = build_course(**ARGS)
    referee = run(course, course.path)
    assert referee.finished
    assert referee.missed == []
    assert referee.passed == 10  # 4 out, 2 at the end buoy, 4 back


@pytest.mark.parametrize("count", [1, 2, 3, 4, 7])
def test_path_is_clean_and_clears_the_buoys_for_any_count(count):
    """Odd and even buoy counts both work, and the path stays clear of every buoy."""
    course = build_course(**{**ARGS, "count": count})
    assert run(course, course.path).missed == []
    for buoy in course.buoys:
        assert min(math.dist(p, buoy[:2]) for p in course.path) > 3.0


def test_wrong_side_is_missed():
    """The mirrored path passes every buoy on the wrong side."""
    course = build_course(**ARGS)
    referee = run(course, [(x, -y) for x, y in course.path])
    assert referee.finished
    assert referee.passed == 0
    assert len(referee.missed) == 10


def test_straight_out_and_back_is_not_clean():
    """Driving down the middle does not count as a slalom."""
    course = build_course(**ARGS)
    line = [(x / 2, 0.5) for x in range(0, 240)]
    referee = run(course, line + line[::-1])
    assert referee.finished
    assert referee.missed


def test_turning_back_early_does_not_finish():
    """Without rounding the end buoy there is no finish."""
    course = build_course(**ARGS)
    half = [p for p in course.path[: len(course.path) // 3]]
    assert not run(course, half + half[::-1]).finished


def test_time_runs_from_start_line_to_finish_line():
    """The clock starts at the start line, and stops at the finish."""
    course = build_course(**ARGS)
    referee = run(course, course.path)
    assert referee.start_time == pytest.approx(16 * 0.1, abs=0.11)  # 8 m at 0.5 m per step
    assert referee.elapsed(1e9) == referee.finish_time - referee.start_time


def test_several_gates_in_one_step():
    """A long gap between two positions must not lose gates."""
    course = build_course(**ARGS)
    referee = Referee(course.gates)
    referee.update(0.0, 1.0, 0.0)
    assert [g.name for g in referee.update(45.0, 1.0, 1.0)] == ["start", "buoy 1", "buoy 2"]


@pytest.mark.parametrize("bad", [{"count": 0}, {"start_line": 25.0}, {"clearance": 12.0}])
def test_impossible_courses_are_refused(bad):
    """A course that cannot be driven is an error."""
    with pytest.raises(ValueError):
        build_course(**{**ARGS, **bad})


def test_pursue_does_not_jump_to_the_way_back():
    """Where the two directions cross, the search stays on the current one."""
    course = build_course(**ARGS)
    index = 60
    x, y = course.path[index]
    nearest, target = pursue(course.path, index, x, y, 6.0)
    assert nearest == index
    assert target[0] > x


def test_gps_round_trip():
    """Course -> GPS -> course is exact."""
    anchor = Anchor(-33.7227, 150.674, 1.0)
    for point in [(0.0, 0.0), (110.0, 4.0), (20.0, -4.0)]:
        assert anchor.to_course(*anchor.to_gps(*point)) == pytest.approx(point, abs=1e-6)


def test_hits_boat():
    """A buoy alongside is clear; one under the hull is a hit."""
    assert hits_boat(0, 0, 0.0, (0.0, 1.4), 0.35, 4.9, 2.4)
    assert not hits_boat(0, 0, 0.0, (0.0, 1.6), 0.35, 4.9, 2.4)
    assert not hits_boat(0, 0, math.pi / 2, (2.0, 0.0), 0.35, 4.9, 2.4)


def test_wrap():
    """Angles wrap to half a turn."""
    assert wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
