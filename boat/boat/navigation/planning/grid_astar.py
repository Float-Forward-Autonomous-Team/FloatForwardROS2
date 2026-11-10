"""Grid A* over per-cell step costs derived from the static map's clearance field."""

import heapq
import math

import numpy as np


def world_to_cell(x, y, origin_x, origin_y, resolution, width, height):
    """Return a grid cell for a map-frame point, or None outside the grid."""
    if not (math.isfinite(x) and math.isfinite(y)) or resolution <= 0:
        return None
    col = math.floor((x - origin_x) / resolution)
    row = math.floor((y - origin_y) / resolution)
    return (col, row) if 0 <= col < width and 0 <= row < height else None


def cell_center(cell, origin_x, origin_y, resolution):
    """Return the map-frame centre of a grid cell."""
    col, row = cell
    return origin_x + (col + 0.5) * resolution, origin_y + (row + 0.5) * resolution


def step_costs(clearance, min_clearance, safe_distance=0.0, proximity_weight=0.0):
    """Return row-major cost multipliers: inf inside the inflation, higher near obstacles.

    `clearance` is the distance in cells to the nearest obstacle, as returned by
    `world_model.layers.clearance_cells`. A cell is passable when it is free and
    its clearance is at least `min_clearance`. Below `safe_distance` the
    multiplier rises linearly towards 1 + `proximity_weight` at zero clearance,
    so routes keep away from obstacles where there is room. All lengths are in
    cells.
    """
    for value in (min_clearance, safe_distance, proximity_weight):
        if not math.isfinite(value) or value < 0:
            raise ValueError("Clearances and weights must be finite and non-negative")
    costs = np.ones(clearance.shape)
    if safe_distance > 0 and proximity_weight > 0:
        costs += proximity_weight * np.clip((safe_distance - clearance) / safe_distance, 0.0, None)
    costs[clearance < max(min_clearance, 1.0)] = math.inf
    return costs.ravel().tolist()


def astar(costs, width, height, start, goal):
    """Find a cheapest eight-connected route through passable cells without cutting corners.

    Moving into a cell costs the step length (1 or sqrt 2) times that cell's
    multiplier from `step_costs`. None means no route or a blocked start/goal.
    """
    if width <= 0 or height <= 0 or len(costs) != width * height:
        return None

    def free(col, row):
        return 0 <= col < width and 0 <= row < height and costs[row * width + col] < math.inf

    if not free(*start) or not free(*goal):
        return None

    start_index = start[1] * width + start[0]
    goal_index = goal[1] * width + goal[0]
    diagonal_cost = math.sqrt(2.0)

    def heuristic(col, row):
        dx, dy = abs(goal[0] - col), abs(goal[1] - row)
        return max(dx, dy) + (diagonal_cost - 1.0) * min(dx, dy)

    frontier = [(heuristic(*start), 0.0, start_index)]
    best_cost = {start_index: 0.0}
    parent = {}

    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if cost > best_cost[current]:
            continue
        if current == goal_index:
            route = [current]
            while route[-1] != start_index:
                route.append(parent[route[-1]])
            return [(index % width, index // width) for index in reversed(route)]

        col, row = current % width, current // width
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            next_col, next_row = col + dx, row + dy
            if not free(next_col, next_row):
                continue
            if dx and dy and (not free(col + dx, row) or not free(col, row + dy)):
                continue
            next_index = next_row * width + next_col
            length = diagonal_cost if dx and dy else 1.0
            next_cost = cost + length * costs[next_index]
            if next_cost < best_cost.get(next_index, math.inf):
                best_cost[next_index] = next_cost
                parent[next_index] = current
                heapq.heappush(
                    frontier,
                    (next_cost + heuristic(next_col, next_row), next_cost, next_index),
                )
    return None
