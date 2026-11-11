"""Static map helpers shared by the world model and both planners.

Grids are numpy arrays of shape (height, width) in ROS row order: row 0 is the
smallest y. Cells are 0 (free), 100 (occupied) or -1 (unknown).
"""

import math

import numpy as np
from scipy.ndimage import distance_transform_edt

FREE = 0
OCCUPIED = 100


def paint_circle(cells, origin_x, origin_y, resolution, x, y, radius):
    """Mark cells whose centres lie within `radius` of (x, y), and the cell under it."""
    height, width = cells.shape
    col = (x - origin_x) / resolution
    row = (y - origin_y) / resolution
    reach = radius / resolution
    col_lo, col_hi = max(0, math.floor(col - reach)), min(width, math.ceil(col + reach) + 1)
    row_lo, row_hi = max(0, math.floor(row - reach)), min(height, math.ceil(row + reach) + 1)
    if col_lo < col_hi and row_lo < row_hi:
        centres_x = np.arange(col_lo, col_hi) + 0.5
        centres_y = np.arange(row_lo, row_hi)[:, None] + 0.5
        inside = (centres_x - col) ** 2 + (centres_y - row) ** 2 <= reach**2
        cells[row_lo:row_hi, col_lo:col_hi][inside] = OCCUPIED
    under_col, under_row = math.floor(col), math.floor(row)
    if 0 <= under_col < width and 0 <= under_row < height:
        cells[under_row, under_col] = OCCUPIED


def clearance_cells(cells):
    """Return each cell's distance, in cells, to the nearest non-free cell or map edge.

    Occupied and unknown cells have clearance 0; a free cell has at least 1.
    The map border counts as an obstacle, so routes keep away from the edge of
    the represented area as they would from a wall.
    """
    free = np.pad(cells == FREE, 1, constant_values=False)
    return distance_transform_edt(free)[1:-1, 1:-1]


def passable_cells(clearance, min_clearance):
    """Return a mask of cells the boat may occupy: free and at least `min_clearance` away.

    `min_clearance` is in cells. The global planner and guidance both use this
    rule, so a smoothed path can never enter a zone the planner forbids.
    """
    return clearance >= max(min_clearance, 1.0)
