"""Simple occupancy-checked path simplification, spline sampling, and LOS targeting."""

import math



# so that we avoid moving super close to docks 

def check_safety_margin(grid_data, width, height, row, col, resolution, safety_margin):
    """Return whether the square clearance neighborhood is known and free."""
    if (
        width <= 0
        or height <= 0
        or len(grid_data) != width * height
        or resolution <= 0
        or not math.isfinite(resolution)
    ):
        return False
    if safety_margin < 0 or not math.isfinite(safety_margin):
        return False

    radius_cells = safety_margin / resolution
    col_left = math.floor(col - radius_cells)
    col_right = math.ceil(col + radius_cells)
    row_down = math.floor(row - radius_cells)
    row_up = math.ceil(row + radius_cells)

    if col_left < 0 or col_right >= width or row_down < 0 or row_up >= height:
        return False

    for i in range(row_down, row_up + 1):
        for j in range(col_left, col_right + 1):
            if grid_data[i * width + j] != 0:
                return False
    return True

def _is_free(
    point, grid_data, width, height, origin_x, origin_y, resolution, safety_margin_m=3.0
):
    """Return whether a world point lies in a known free occupancy cell."""
    if resolution <= 0 or not math.isfinite(resolution):
        return False
    x, y = point
    if not (math.isfinite(x) and math.isfinite(y)):
        return False
    col = math.floor((x - origin_x) / resolution)
    row = math.floor((y - origin_y) / resolution)
    return (
        0 <= col < width
        and 0 <= row < height
        and len(grid_data) == width * height
        and check_safety_margin(
            grid_data, width, height, row, col, resolution, safety_margin_m
        )
    )

    


def _segment_is_free(
    start,
    end,
    grid_data,
    width,
    height,
    origin_x,
    origin_y,
    resolution,
    safety_margin_m=3.0,
):
    """Sample a line at quarter-cell spacing and reject occupied or unknown cells."""
    distance = math.dist(start, end)
    steps = max(1, math.ceil(distance / (resolution * 0.25)))
    for index in range(steps + 1):
        fraction = index / steps
        point = (
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]),
        )
        if not _is_free(
            point,
            grid_data,
            width,
            height,
            origin_x,
            origin_y,
            resolution,
            safety_margin_m,
        ):
            return False
    return True


def _path_is_free(
    points, grid_data, width, height, origin_x, origin_y, resolution, safety_margin_m=3.0
):
    """Validate all points and the sampled segments connecting them."""
    if not points or any(
        not _is_free(
            point,
            grid_data,
            width,
            height,
            origin_x,
            origin_y,
            resolution,
            safety_margin_m,
        )
        for point in points
    ):
        return False
    return all(
        _segment_is_free(
            start,
            end,
            grid_data,
            width,
            height,
            origin_x,
            origin_y,
            resolution,
            safety_margin_m,
        )
        for start, end in zip(points, points[1:])
    )


def simplify_path(
    points, grid_data, width, height, origin_x, origin_y, resolution, safety_margin_m=3.0
):
    """Remove route points only when a direct shortcut stays in free cells."""
    route = [(float(point[0]), float(point[1])) for point in points]
    if not _path_is_free(
        route, grid_data, width, height, origin_x, origin_y, resolution, safety_margin_m
    ):
        return []
    if len(route) < 3:
        return route

    simplified = [route[0]]
    anchor = 0
    while anchor < len(route) - 1:
        candidate = len(route) - 1
        while candidate > anchor + 1 and not _segment_is_free(
            route[anchor],
            route[candidate],
            grid_data,
            width,
            height,
            origin_x,
            origin_y,
            resolution,
            safety_margin_m,
        ):
            candidate -= 1
        simplified.append(route[candidate])
        anchor = candidate
    return simplified


def catmull_rom_spline(points, sample_spacing):
    """Sample a Catmull-Rom curve through points at approximately fixed spacing."""
    if sample_spacing <= 0 or not math.isfinite(sample_spacing):
        raise ValueError("Sample spacing must be finite and positive")
    if len(points) < 2:
        return list(points)

    sampled = [points[0]]
    for index in range(len(points) - 1):
        p0 = points[max(0, index - 1)]
        p1 = points[index]
        p2 = points[index + 1]
        p3 = points[min(len(points) - 1, index + 2)]
        steps = max(1, math.ceil(math.dist(p1, p2) / sample_spacing))
        for step in range(1, steps + 1):
            t = step / steps
            t2 = t * t
            t3 = t2 * t
            sample = []
            for axis in (0, 1):
                sample.append(
                    0.5
                    * (
                        2 * p1[axis]
                        + (-p0[axis] + p2[axis]) * t
                        + (2 * p0[axis] - 5 * p1[axis] + 4 * p2[axis] - p3[axis])
                        * t2
                        + (-p0[axis] + 3 * p1[axis] - 3 * p2[axis] + p3[axis])
                        * t3
                    )
                )
            sampled.append(tuple(sample))
    return sampled


def generate_safe_spline(
    points, data, width, height, origin_x, origin_y, resolution, safety_margin_m=3.0
):
    """Smooth a simplified route; fall back to its collision-checked polyline."""
    simplified = simplify_path(
        points, data, width, height, origin_x, origin_y, resolution, safety_margin_m
    )
    if not simplified:
        return []
    if len(simplified) < 3:
        return simplified

    spline = catmull_rom_spline(simplified, resolution * 0.5)
    if _path_is_free(
        spline, data, width, height, origin_x, origin_y, resolution, safety_margin_m
    ):
        return spline
    return simplified


#(x,y) current world coordinates of the boat

def line_of_sight_target(points, x, y, lookahead_m):
    """Return the point and heading at lookahead distance from the nearest route point."""
    if not points:
        return None
    if lookahead_m < 0 or not math.isfinite(lookahead_m):
        raise ValueError("Lookahead distance must be finite and non-negative")
    if len(points) == 1:
        return points[0], 0.0

    lengths = [math.dist(start, end) for start, end in zip(points, points[1:])]
    cumulative = [0.0]
    for length in lengths:
        cumulative.append(cumulative[-1] + length)

    nearest_distance = math.inf
    nearest_route_distance = 0.0
    for index, (start, end) in enumerate(zip(points, points[1:])):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared == 0:
            continue

        # fraction of the segment that is closest to the boat's current position
        fraction = max(        
            0.0,
            min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length_squared),
        )
        projected = (start[0] + fraction * dx, start[1] + fraction * dy)
        distance = math.dist((x, y), projected)
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_route_distance = cumulative[index] + fraction * lengths[index]

    target_distance = min(cumulative[-1], nearest_route_distance + lookahead_m)
    for index, length in enumerate(lengths):
        if target_distance <= cumulative[index + 1] or index == len(lengths) - 1:
            fraction = 0.0 if length == 0 else (target_distance - cumulative[index]) / length
            fraction = max(0.0, min(1.0, fraction))
            start, end = points[index], points[index + 1]
            target = (
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            )
            return target, math.atan2(end[1] - start[1], end[0] - start[0])
    return points[-1], 0.0
