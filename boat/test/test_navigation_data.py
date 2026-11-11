"""Focused checks for simulator data, the world model layers, and global planning."""

import math
from types import SimpleNamespace

import numpy as np

from boat.navigation.planning.grid_astar import astar, cell_center, step_costs, world_to_cell
from boat.navigation.planning.path_geometry import distance_to_polyline
from boat.navigation.planning.path_smoothing import generate_safe_spline, line_of_sight_target
from boat.navigation.simulation.obstacle_trajectory import MovingObstacle
from boat.navigation.simulation.scenario_map import ScenarioMap
from boat.navigation.world_model.layers import clearance_cells, paint_circle, passable_cells
from boat.navigation.world_model.objects import StationaryFilter, predict


def costs_for(data, width, height, radius=0.0, **kwargs):
    """Build A* step costs from a flat occupancy list."""
    cells = np.array(data, dtype=np.int8).reshape(height, width)
    return step_costs(clearance_cells(cells), radius, **kwargs)


def track(track_id, x, y, vx=0.0, vy=0.0, radius=1.0):
    """Build a duck-typed TrackedObject."""
    return SimpleNamespace(
        id=track_id,
        radius_m=radius,
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)),
        twist=SimpleNamespace(linear=SimpleNamespace(x=vx, y=vy)),
    )


def test_waypoint_trajectory_loops_with_expected_velocity():
    """Looping objects report the reverse leg after reaching the last waypoint."""
    obstacle = MovingObstacle("probe", "test", 1.0, 1.0, ((0.0, 0.0), (10.0, 0.0)))
    assert obstacle.position_velocity(5.0) == (5.0, 0.0, 1.0, 0.0)
    assert obstacle.position_velocity(15.0) == (5.0, 0.0, -1.0, 0.0)


def test_map_uses_collision_geometry_ros_row_order_and_records_landmarks(tmp_path):
    """A world box occupies its own cell and becomes a landmark at its pose."""
    world = tmp_path / "world.sdf"
    world.write_text(
        """<sdf version="1.9"><world name="test"><model name="box">
        <pose>3.5 4.5 0 0 0 0</pose><link name="body"><collision name="solid">
        <geometry><box><size>1 1 2</size></box></geometry>
        </collision></link></model></world></sdf>""",
        encoding="utf-8",
    )
    grid = ScenarioMap(0, 0, 10, 10, 1, [])
    grid.load_world(world)
    values = grid.values()
    assert values[4 * 10 + 3] == 100
    assert values[8 * 10 + 8] == 0
    (landmark,) = grid.landmarks
    assert (landmark.name, landmark.model, landmark.x, landmark.y) == ("box", "box", 3.5, 4.5)
    assert math.isclose(landmark.radius_m, math.sqrt(0.5))


def test_clearance_counts_obstacles_unknown_cells_and_map_edge():
    """Clearance is 0 on blocked cells and measures distance to the border."""
    cells = np.zeros((5, 5), dtype=np.int8)
    assert clearance_cells(cells)[2, 2] == 3
    assert clearance_cells(cells)[0, 0] == 1
    cells[2, 2] = -1
    clearance = clearance_cells(cells)
    assert clearance[2, 2] == 0
    assert clearance[2, 3] == 1


def test_paint_circle_marks_centres_within_radius():
    """Painting uses metres and the map origin; cells outside the radius stay free."""
    cells = np.zeros((10, 10), dtype=np.int8)
    paint_circle(cells, -5.0, -5.0, 1.0, 0.0, 0.0, 1.0)
    assert cells[5, 5] == 100 and cells[4, 4] == 100
    assert cells[5, 7] == 0
    paint_circle(cells, -5.0, -5.0, 1.0, 50.0, 50.0, 1.0)
    assert np.count_nonzero(cells) == 4


def test_astar_routes_around_obstacles_without_cutting_diagonal_corners():
    """A route through a gap must avoid occupied cells and blocked corners."""
    width, height = 7, 5
    data = [0] * (width * height)
    for row in (0, 1, 3, 4):
        data[row * width + 3] = 100
    path = astar(costs_for(data, width, height), width, height, (1, 1), (5, 1))
    assert path[0] == (1, 1)
    assert path[-1] == (5, 1)
    assert (3, 2) in path
    assert all(data[row * width + col] == 0 for col, row in path)

    corner = [0, 100, 100, 0]
    assert astar(costs_for(corner, 2, 2), 2, 2, (0, 0), (1, 1)) is None


def test_astar_rejects_blocked_or_outside_goals():
    """Do not draw a route through unknown cells or beyond map bounds."""
    assert astar(costs_for([0, -1, 0], 3, 1), 3, 1, (0, 0), (2, 0)) is None
    assert world_to_cell(-593, 241, -650, -50, 0.5, 700, 800) == (114, 582)
    assert world_to_cell(-300, 241, -650, -50, 0.5, 700, 800) is None
    assert cell_center((114, 582), -650, -50, 0.5) == (-592.75, 241.25)


def test_inflation_keeps_route_clear_of_obstacle_and_map_edge():
    """Inflation changes the route and closes a gap narrower than the radius."""
    width = height = 13
    data = [0] * (width * height)
    data[6 * width + 6] = 100
    path = astar(costs_for(data, width, height, 2), width, height, (2, 6), (10, 6))
    assert path is not None
    assert all((col - 6) ** 2 + (row - 6) ** 2 >= 4 for col, row in path)
    assert all(2 <= col <= 10 and 2 <= row <= 10 for col, row in path)

    data = [0] * (7 * 5)
    for row in (0, 1, 3, 4):
        data[row * 7 + 3] = 100
    assert astar(costs_for(data, 7, 5, 1.5), 7, 5, (1, 2), (5, 2)) is None


def test_proximity_cost_moves_route_away_from_a_wall():
    """With a safe distance, the route leaves the wall even though it is longer."""
    width, height = 40, 15
    data = [0] * (width * height)
    for col in range(width):
        data[col] = 100
    plain = astar(costs_for(data, width, height, 1), width, height, (2, 2), (37, 2))
    kept_away = astar(
        costs_for(data, width, height, 1, safe_distance=8, proximity_weight=3),
        width,
        height,
        (2, 2),
        (37, 2),
    )
    assert max(row for _, row in plain) == 2
    assert max(row for _, row in kept_away) >= 6


def test_stationary_filter_promotes_after_hold_and_demotes_on_motion():
    """Stopped objects become static after the hold time and leave when they move."""
    stationary = StationaryFilter(speed_threshold_mps=0.2, hold_time_s=5.0)
    assert not stationary.update([track("a", 0, 0)], 0.0)
    assert not stationary.update([track("a", 0, 0)], 4.9)
    assert stationary.update([track("a", 0, 0)], 5.0)
    assert set(stationary.static) == {"a"}
    assert stationary.moving([track("a", 0, 0), track("b", 9, 9)])[0].id == "b"

    assert not stationary.update([], 6.0)
    assert set(stationary.static) == {"a"}
    assert stationary.update([track("a", 0, 0, vx=1.0)], 7.0)
    assert not stationary.static


def test_stationary_filter_resets_timer_on_motion_or_lost_track():
    """The hold time must be continuous and observed."""
    stationary = StationaryFilter(speed_threshold_mps=0.2, hold_time_s=5.0)
    stationary.update([track("a", 0, 0)], 0.0)
    stationary.update([track("a", 0, 0, vx=1.0)], 3.0)
    assert not stationary.update([track("a", 3, 0)], 6.0)
    stationary.update([], 7.0)
    assert not stationary.update([track("a", 3, 0)], 11.5)
    assert stationary.update([track("a", 3, 0)], 16.5)


def test_predict_uses_constant_velocity():
    """Prediction extrapolates from the observation time."""
    assert predict(track("a", 1.0, 2.0, vx=0.5, vy=-1.0), 10.0, 14.0) == (3.0, -2.0)


def test_distance_to_polyline():
    """Distance is measured to segments, not only vertices."""
    line = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    assert distance_to_polyline((5.0, 3.0), line) == 3.0
    assert distance_to_polyline((13.0, 5.0), line) == 3.0
    assert distance_to_polyline((-4.0, 3.0), line) == 5.0
    assert distance_to_polyline((1.0, 1.0), [(1.0, 4.0)]) == 3.0


def test_status_reporter_publishes_only_changes_and_matches_goals():
    """Repeated reports are dropped; goal matching uses the tolerance."""
    from boat_interfaces.msg import NavigationStatus

    from boat.navigation.status import StatusReporter, same_goal

    sent = []
    clock = SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: NavigationStatus().header.stamp)
    )
    node = SimpleNamespace(
        get_clock=lambda: clock,
        create_publisher=lambda *args: SimpleNamespace(publish=sent.append),
    )
    reporter = StatusReporter(node, "/test/status")
    assert reporter.report(NavigationStatus.ACTIVE, (1.0, 2.0), "go")
    assert not reporter.report(NavigationStatus.ACTIVE, (1.0, 2.0), "go")
    assert reporter.report(NavigationStatus.SUCCEEDED, (1.0, 2.0), "done")
    assert [message.state for message in sent] == [
        NavigationStatus.ACTIVE,
        NavigationStatus.SUCCEEDED,
    ]
    assert (sent[-1].goal.x, sent[-1].goal.y) == (1.0, 2.0)
    assert same_goal(sent[-1].goal, (1.3, 2.0), 0.5)
    assert not same_goal(sent[-1].goal, (2.0, 2.0), 0.5)
    assert not same_goal(sent[-1].goal, None, 0.5)


def test_line_of_sight_target_is_lookahead_along_route():
    """The target lies lookahead metres past the boat's projection onto the route."""
    route = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    target, heading = line_of_sight_target(route, 2.0, 1.0, 5.0)
    assert target == (7.0, 0.0) and heading == 0.0
    target, heading = line_of_sight_target(route, 9.0, -1.0, 5.0)
    assert target == (10.0, 4.0) and math.isclose(heading, math.pi / 2)
    assert line_of_sight_target(route, 10.0, 9.0, 5.0)[0] == (10.0, 10.0)


def test_safe_spline_shortcuts_staircase_without_entering_obstacles():
    """Smoothing removes grid corners but keeps the margin around obstacles."""
    width = height = 40
    data = [0] * (width * height)
    for row in range(10, 30):
        for col in range(18, 22):
            data[row * width + col] = 100
    route = [(5.0 + i, 5.0) for i in range(30)] + [(34.0, 5.0 + i) for i in range(1, 30)]
    cells = np.array(data, dtype=np.int8).reshape(height, width)
    passable = passable_cells(clearance_cells(cells), 2.0)
    spline = generate_safe_spline(route, passable, 0.0, 0.0, 1.0)
    assert spline and len(spline) < len(route) * 3
    assert spline[0] == route[0] and spline[-1] == route[-1]
    for x, y in spline:
        assert passable[math.floor(y), math.floor(x)]
        assert not (17.0 <= x < 23.0 and 9.0 <= y < 31.0)


def test_planner_route_always_passes_the_guidance_margin_check():
    """Guidance and planner share one clearance rule, so an A* route is never rejected."""
    width, height, radius = 60, 40, 3.0
    cells = np.zeros((height, width), dtype=np.int8)
    cells[0:28, 20:24] = 100
    cells[12:40, 38:42] = 100
    clearance = clearance_cells(cells)
    route = astar(step_costs(clearance, radius), width, height, (5, 5), (55, 35))
    assert route is not None
    points = [cell_center(cell, 0.0, 0.0, 1.0) for cell in route]
    passable = passable_cells(clearance, radius)
    spline = generate_safe_spline(points, passable, 0.0, 0.0, 1.0)
    assert spline[0] == points[0] and spline[-1] == points[-1]
    assert all(clearance[math.floor(y), math.floor(x)] >= radius for x, y in spline)
