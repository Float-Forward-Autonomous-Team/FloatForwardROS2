"""Focused checks for simulator trajectory and occupancy map coordinates."""

import math

from boat.navigation.planning.grid_astar import astar, cell_center, world_to_cell
from boat.navigation.simulation.obstacle_trajectory import MovingObstacle
from boat.navigation.simulation.scenario_map import ScenarioMap


def test_waypoint_trajectory_loops_with_expected_velocity():
    """Looping objects report the reverse leg after reaching the last waypoint."""
    obstacle = MovingObstacle("probe", "test", 1.0, 1.0, ((0.0, 0.0), (10.0, 0.0)))
    assert obstacle.position_velocity(5.0) == (5.0, 0.0, 1.0, 0.0)
    assert obstacle.position_velocity(15.0) == (5.0, 0.0, -1.0, 0.0)


def test_map_uses_collision_geometry_and_ros_row_order(tmp_path):
    """A world box occupies its own cell, and moving overlays leave the base map intact."""
    world = tmp_path / "world.sdf"
    world.write_text(
        """<sdf version="1.9"><world name="test"><model name="box">
        <pose>3 4 0 0 0 0</pose><link name="body"><collision name="solid">
        <geometry><box><size>1 1 2</size></box></geometry>
        </collision></link></model></world></sdf>""",
        encoding="utf-8",
    )
    grid = ScenarioMap(0, 0, 10, 10, 1, [])
    grid.load_world(world)
    values = grid.values()
    assert values[4 * 10 + 3] == 100
    assert values[8 * 10 + 8] == 0

    overlay = grid.image.copy()
    grid.draw_circle(overlay, 8, 8, 0.7)
    assert grid.values(overlay)[8 * 10 + 8] == 100
    assert grid.values()[8 * 10 + 8] == 0
    assert math.isclose(grid.resolution, 1.0)


def test_astar_routes_around_obstacles_without_cutting_diagonal_corners():
    """A route through a gap must avoid occupied cells and blocked corners."""
    width, height = 7, 5
    data = [0] * (width * height)
    for row in (0, 1, 3, 4):
        data[row * width + 3] = 100
    path = astar(data, width, height, (1, 1), (5, 1))
    assert path[0] == (1, 1)
    assert path[-1] == (5, 1)
    assert (3, 2) in path
    assert all(data[row * width + col] == 0 for col, row in path)

    corner = [0, 100, 100, 0]
    assert astar(corner, 2, 2, (0, 0), (1, 1)) is None


def test_astar_rejects_blocked_or_outside_goals():
    """Do not draw a route through unknown cells or beyond map bounds."""
    assert astar([0, -1, 0], 3, 1, (0, 0), (2, 0)) is None
    assert world_to_cell(-593, 241, -650, -50, 0.5, 700, 800) == (114, 582)
    assert world_to_cell(-300, 241, -650, -50, 0.5, 700, 800) is None
    assert cell_center((114, 582), -650, -50, 0.5) == (-592.75, 241.25)


def test_astar_inflation_keeps_route_clear_of_obstacle():
    """Inflation changes the route and closes a gap narrower than the radius."""
    width = height = 13
    data = [0] * (width * height)
    data[6 * width + 6] = 100
    path = astar(data, width, height, (2, 6), (10, 6), inflation_radius_cells=2)
    assert path is not None
    assert all((col - 6) ** 2 + (row - 6) ** 2 >= 4 for col, row in path)

    data = [0] * (7 * 5)
    for row in (0, 1, 3, 4):
        data[row * 7 + 3] = 100
    assert astar(data, 7, 5, (1, 2), (5, 2), inflation_radius_cells=1.5) is None
