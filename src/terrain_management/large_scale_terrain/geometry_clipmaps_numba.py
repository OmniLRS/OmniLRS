__author__ = "Antoine Richard"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging

import numba as nb
import numpy as np

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")

# Vertices are addressed by their doubled (x, y) coordinates, so that the midpoints of the cells
# are integers. Within a level, the vertices lie on a regular lattice whose spacing, in doubled
# coordinates, is the step of the level. Each level stores the index of the vertices it adds in a
# dense lattice (-1 where no vertex was added), which replaces hashing the vertex coordinates.
# The level context passed around is: (level, grid, step, radius, prev_grid, prev_step, prev_radius).
# The counters array holds: [number of points, number of indices].


@nb.jit(nopython=True, cache=True)
def _level_params(level, meshBaseLODExtentHeightfieldTexels):
    """
    Compute the tessellation parameters of a level.

    Args:
        level (int): Level of the mesh backbone.
        meshBaseLODExtentHeightfieldTexels (int): Heightfield texels in the mesh backbone.

    Returns:
        Tuple[int, int, float, int, int]: Step of the level, step of the previous level (0 for
        level 0), half extent of the level in steps, radius of the level, and number of cells
        per axis.
    """

    step = 1 << level
    if level == 0:
        prevStep = 0
    else:
        prevStep = max(0, (1 << (level - 1)))
    g = meshBaseLODExtentHeightfieldTexels / 2
    # Pad by one element to hide the gap to the next level
    pad = 0
    radius = int(step * (g + pad))
    num_cells = len(range(-radius, radius, step))
    return step, prevStep, g, radius, num_cells


@nb.jit(nopython=True, cache=True)
def _mesh_buffer_sizes(start_level, num_levels, meshBaseLODExtentHeightfieldTexels):
    """
    Compute the sizes of the buffers needed to build the mesh backbone. The number of triangles is
    exact. The number of points is an upper bound: the total size of the vertex lattices.

    Args:
        start_level (int): Starting level of the mesh backbone.
        num_levels (int): Number of levels in the mesh backbone.
        meshBaseLODExtentHeightfieldTexels (int): Heightfield texels in the mesh backbone.

    Returns:
        Tuple[int, int]: Number of triangles and maximum number of points in the mesh backbone.
    """

    num_triangles = 0
    max_points = 0
    for level in range(start_level, num_levels):
        step, halfStep, g, radius, num_cells = _level_params(level, meshBaseLODExtentHeightfieldTexels)
        max_points += (2 * num_cells + 1) ** 2
        for y in range(-radius, radius, step):
            for x in range(-radius, radius, step):
                if max(abs(x + halfStep), abs(y + halfStep)) >= g * halfStep:
                    # 8 triangles per cell, minus one per side stitched into the next level.
                    num_triangles += 8
                    num_triangles -= int(x == -radius) + int(x == (radius - step))
                    num_triangles -= int(y == -radius) + int(y == (radius - step))
    return num_triangles, max_points


@nb.jit(nopython=True, cache=True)
def _query_point_index(point, ctx, points, counters):
    """
    Querry the index of a point in the mesh. If the point is not in the mesh, add it.
    The point is first looked up in the lattice of the previous level, which holds the points
    added at the previous level. Then, it is looked up in the lattice of the current level.

    Args:
        point (Tuple[int, int]): Doubled (x, y) coordinates of the point.
        ctx (tuple): Level context (level, grid, step, radius, prev_grid, prev_step, prev_radius).
        points (np.ndarray): Buffer of vertices in the mesh (N, 3).
        counters (np.ndarray): Number of points and number of indices in the buffers.

    Returns:
        int: Index of the point.
    """

    level, grid, step, radius, prev_grid, prev_step, prev_radius = ctx
    px, py = point
    if prev_step > 0:
        ox = px + 2 * prev_radius
        oy = py + 2 * prev_radius
        if ox >= 0 and oy >= 0 and ox % prev_step == 0 and oy % prev_step == 0:
            i = ox // prev_step
            j = oy // prev_step
            if i < prev_grid.shape[1] and j < prev_grid.shape[0] and prev_grid[j, i] >= 0:
                return prev_grid[j, i]
    i = (px + 2 * radius) // step
    j = (py + 2 * radius) // step
    index = grid[j, i]
    if index < 0:
        index = counters[0]
        grid[j, i] = index
        points[index, 0] = px * 0.5
        points[index, 1] = py * 0.5
        points[index, 2] = float(level)
        counters[0] += 1
    return index


@nb.jit(nopython=True, cache=True)
def _add_triangle(A, B, C, ctx, points, indices, counters):
    """
    Add a triangle to the mesh. To add the triangle, the indices of the vertices are querried.
    If the vertices are not in the mesh, they are added. Once the vertices are in the mesh, the
    indices are written to the buffer of indices to form the triangle.

    Args:
        A (Tuple[int, int]): Doubled (x, y) coordinates of the first vertex of the triangle.
        B (Tuple[int, int]): Doubled (x, y) coordinates of the second vertex of the triangle.
        C (Tuple[int, int]): Doubled (x, y) coordinates of the third vertex of the triangle.
        ctx (tuple): Level context (level, grid, step, radius, prev_grid, prev_step, prev_radius).
        points (np.ndarray): Buffer of vertices in the mesh (N, 3).
        indices (np.ndarray): Buffer of indices of the vertices in the mesh (M,).
        counters (np.ndarray): Number of points and number of indices in the buffers.
    """

    # Numba does not bounds check: refuse to write past the buffer sized by _mesh_buffer_sizes.
    if counters[1] + 3 > indices.shape[0]:
        raise RuntimeError("Mesh backbone: more triangles than counted by _mesh_buffer_sizes.")
    for vertex in (A, B, C):
        n = counters[1]
        indices[n] = _query_point_index(vertex, ctx, points, counters)
        counters[1] = n + 1


@nb.jit(nopython=True, cache=True)
def _build_mesh(start_level, num_levels, meshBaseLODExtentHeightfieldTexels):
    """
    Build the mesh backbone for the geometry clipmaps. The mesh backbone is a series of triangles
    that tessellate the terrain. The mesh backbone is built from the bottom up, starting at the
    lowest level and working up to the highest level. Geometry clipmaps are used to render the
    terrain at different levels of detail. At the center, the terrain is rendered at the highest
    level of detail. As the distance from the center increases, the level of detail decreases.

    The UV coordinates of the triangle corners are the (x, y) coordinates of their vertices, so they
    are not built here: they can be obtained with points[indices, :2].

    Args:
        start_level (int): Starting level of the mesh backbone.
        num_levels (int): Number of levels in the mesh backbone.
        meshBaseLODExtentHeightfieldTexels (int): Heightfield texels in the mesh backbone.

    Returns:
        Tuple[np.ndarray, np.ndarray]: Vertices (N, 3) and indices (M,) in the mesh backbone.
    """

    print("Building the mesh backbone, this may take time...")
    num_triangles, max_points = _mesh_buffer_sizes(start_level, num_levels, meshBaseLODExtentHeightfieldTexels)
    points = np.empty((max_points, 3), dtype=np.float64)
    indices = np.empty(3 * num_triangles, dtype=np.int64)
    counters = np.zeros(2, dtype=np.int64)
    prev_grid = np.full((0, 0), -1, dtype=np.int64)
    prev_step = 0
    prev_radius = 0
    for level in range(start_level, num_levels):
        print("Generating level " + str(level + 1) + " out of " + str(num_levels) + "...")
        step, halfStep, g, radius, num_cells = _level_params(level, meshBaseLODExtentHeightfieldTexels)
        grid = np.full((2 * num_cells + 1, 2 * num_cells + 1), -1, dtype=np.int64)
        ctx = (level, grid, step, radius, prev_grid, prev_step, prev_radius)

        for y in range(-radius, radius, step):
            for x in range(-radius, radius, step):
                if max(abs(x + halfStep), abs(y + halfStep)) >= g * halfStep:
                    # Cleared the cutout from the previous level. Tessellate the
                    # square.

                    #   A-----B-----C
                    #   | \   |   / |
                    #   |   \ | /   |
                    #   D-----E-----F
                    #   |   / | \   |
                    #   | /   |   \ |
                    #   G-----H-----I

                    A = (2 * x, 2 * y)
                    C = (A[0] + 2 * step, A[1])
                    G = (A[0], A[1] + 2 * step)
                    I = (C[0], G[1])  # noqa: E741  (geometric point label in the A-I grid)

                    B = (A[0] + step, A[1])
                    D = (A[0], A[1] + step)
                    F = (C[0], A[1] + step)
                    H = (A[0] + step, G[1])

                    E = (A[0] + step, A[1] + step)

                    # Stitch the border into the next level

                    if x == -radius:
                        #   A-----B-----C
                        #   | \   |   / |
                        #   |   \ | /   |
                        #   |     E-----F
                        #   |   / | \   |
                        #   | /   |   \ |
                        #   G-----H-----I
                        _add_triangle(E, A, G, ctx, points, indices, counters)
                    else:
                        _add_triangle(E, A, D, ctx, points, indices, counters)
                        _add_triangle(E, D, G, ctx, points, indices, counters)

                    if y == (radius - step):
                        _add_triangle(E, G, I, ctx, points, indices, counters)
                    else:
                        _add_triangle(E, G, H, ctx, points, indices, counters)
                        _add_triangle(E, H, I, ctx, points, indices, counters)

                    if x == (radius - step):
                        _add_triangle(E, I, C, ctx, points, indices, counters)
                    else:
                        _add_triangle(E, I, F, ctx, points, indices, counters)
                        _add_triangle(E, F, C, ctx, points, indices, counters)

                    if y == -radius:
                        _add_triangle(E, C, A, ctx, points, indices, counters)
                    else:
                        _add_triangle(E, C, B, ctx, points, indices, counters)
                        _add_triangle(E, B, A, ctx, points, indices, counters)
        prev_grid = grid
        prev_step = step
        prev_radius = radius
    if counters[1] != indices.shape[0]:
        raise RuntimeError("Mesh backbone: fewer triangles than counted by _mesh_buffer_sizes.")
    # The points buffer was sized with an upper bound, trim it to the points actually added.
    return points[: counters[0]].copy(), indices
