"""Small grid A* for visualizing routes through the current occupancy map."""

import heapq
import math
from functools import lru_cache


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


def astar(data, width, height, start, goal, inflation_radius_cells=0.0):
    """Find an eight-connected route through free cells without cutting corners.

    Occupied and unknown cells are blocked. None means no route or an invalid
    start/goal. The inflation radius keeps cell centres away from occupied
    cells; dynamic objects enter through their current occupancy cells.
    """
    if width <= 0 or height <= 0 or len(data) != width * height:
        return None
    if not math.isfinite(inflation_radius_cells) or inflation_radius_cells < 0:
        raise ValueError("Inflation radius must be finite and non-negative")

    row_spans = []
    prefix = []
    if inflation_radius_cells > 0:
        radius_squared = inflation_radius_cells**2
        for dy in range(-math.ceil(inflation_radius_cells), math.ceil(inflation_radius_cells) + 1):
            remaining = radius_squared - dy * dy
            if remaining > 0:
                row_spans.append((dy, math.ceil(math.sqrt(remaining)) - 1))
        row_spans.sort(key=lambda item: abs(item[0]))

        # Row prefix sums check each horizontal slice of the clearance circle
        # in constant time, instead of examining every cell in that circle.
        prefix = [0] * (height * (width + 1))
        for row in range(height):
            count = 0
            for col in range(width):
                count += data[row * width + col] != 0
                prefix[row * (width + 1) + col + 1] = count

    def inside(col, row):
        return 0 <= col < width and 0 <= row < height

    @lru_cache(maxsize=None)
    def free(col, row):
        if not inside(col, row):
            return False
        if not row_spans:
            return data[row * width + col] == 0
        for dy, span in row_spans:
            other_row = row + dy
            left, right = col - span, col + span
            if not inside(left, other_row) or not inside(right, other_row):
                return False
            base = other_row * (width + 1)
            if prefix[base + right + 1] != prefix[base + left]:
                return False
        return True

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
            next_cost = cost + (diagonal_cost if dx and dy else 1.0)
            if next_cost < best_cost.get(next_index, math.inf):
                best_cost[next_index] = next_cost
                parent[next_index] = current
                heapq.heappush(
                    frontier,
                    (next_cost + heuristic(next_col, next_row), next_cost, next_index),
                )
    return None
