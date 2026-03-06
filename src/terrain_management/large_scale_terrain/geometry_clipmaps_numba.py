__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import numba as nb
import numpy as np
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")

point3 = nb.types.UniTuple(nb.types.float32, 3)
point2 = nb.types.UniTuple(nb.types.float32, 2)
idx_dict_type = nb.types.DictType(point2, nb.types.int32)
point3_list_type = nb.types.ListType(point3)
point2_list_type = nb.types.ListType(point2)
idx_list_type = nb.types.ListType(nb.types.int32)


@nb.jit(point2(point3), nopython=True)
def _point3_to_point2(point):
    """
    Convert a 3D point to a 2D point by dropping the z coordinate.

    Args:
        point (point3): 3D point to convert.

    Returns:
        point2: 2D point with the z coordinate dropped.
    """

    return point[:2]


@nb.jit(
    nb.types.Tuple((nb.types.int32, nb.types.int32))(
        point3,
        point3_list_type,
        idx_dict_type,
        idx_dict_type,
        nb.types.int32,
    ),
    nopython=True,
)
def _query_point_index(point, points, prev_indices, new_indices, index_count):
    """
    Querry the index of a point in the list of points. If the point is not in the list, add it.
    The previous indices are the indices of the points in the list at the previous level.
    The new indices are the indices of the points in the list at the current level.
    The index count is used to give a unique index to the point.

    Args:
        point (point3): Point to querry.
        points (point3_list_type): List of points.
        prev_indices (idx_dict_type): Dictionary of indices of points in the list.
        new_indices (idx_dict_type): Dictionary of indices of points in the list.
        index_count (int): Current index count.
    """
    hash = _point3_to_point2(point)
    if hash in prev_indices:
        index = prev_indices[hash]
    elif hash in new_indices:
        index = new_indices[hash]
    else:
        index = index_count
        points.append(point)
        new_indices[hash] = index
        index_count += 1
    return index, index_count


@nb.jit(
    nb.types.int32(
        point3,
        point3,
        point3,
        idx_list_type,
        point2_list_type,
        point3_list_type,
        idx_dict_type,
        idx_dict_type,
        nb.types.int32,
    ),
    nopython=True,
)
def _add_triangle(
    A,
    B,
    C,
    indices,
    uvs,
    points,
    prev_indices,
    new_indices,
    index_count,
):
    """
    Add a triangle to the mesh. To add the triangle, the indices of the vertices are querried.
    If the vertices are not in the list, they are added. Once the vertices are in the list, the
    indices are appended to the list of indices to form the triangle. The UV coordinates of the
    vertices are also appended to the list of UV coordinates.

    Args:
        A (point3): First vertex of the triangle.
        B (point3): Second vertex of the triangle.
        C (point3): Third vertex of the triangle.
        indices (idx_list_type): List of indices of the vertices in the mesh.
        uvs (point2_list_type): List of UV coordinates of the vertices in the mesh.
        points (point3_list_type): List of vertices in the mesh.
        prev_indices (idx_dict_type): Dictionary of indices of vertices in the mesh.
        new_indices (idx_dict_type): Dictionary of indices of vertices in the mesh.
        index_count (int): Current index count.
    """

    A_idx, index_count = _query_point_index(A, points, prev_indices, new_indices, index_count)
    B_idx, index_count = _query_point_index(B, points, prev_indices, new_indices, index_count)
    C_idx, index_count = _query_point_index(C, points, prev_indices, new_indices, index_count)
    indices.append(A_idx)
    indices.append(B_idx)
    indices.append(C_idx)
    uvs.append((A[0], A[1]))
    uvs.append((B[0], B[1]))
    uvs.append((C[0], C[1]))
    return index_count


@nb.jit(nopython=True)
def _build_mesh(start_level, num_levels, meshBaseLODExtentHeightfieldTexels):
    """
    Build the mesh backbone for the geometry clipmaps. The mesh backbone is a series of triangles
    that tessellate the terrain. The mesh backbone is built from the bottom up, starting at the
    lowest level and working up to the highest level. Geometry clipmaps are used to render the
    terrain at different levels of detail. At the center, the terrain is rendered at the highest
    level of detail. As the distance from the center increases, the level of detail decreases.

    Args:
        start_level (int): Starting level of the mesh backbone.
        num_levels (int): Number of levels in the mesh backbone.
        meshBaseLODExtentHeightfieldTexels (int): Heightfield texels in the mesh backbone.

    Returns:
        Tuple[point3_list_type, point2_list_type, idx_list_type]: List of vertices, list of UV
        coordinates, and list of indices in the mesh backbone.
    """

    print("Building the mesh backbone, this may take time...")
    points = nb.typed.List.empty_list(point3)
    uvs = nb.typed.List.empty_list(point2)
    indices = nb.typed.List.empty_list(nb.types.int32)
    prev_indices = nb.typed.Dict.empty(key_type=point2, value_type=nb.types.int32)
    new_indices = nb.typed.Dict.empty(key_type=point2, value_type=nb.types.int32)
    index_count = 0
    for level in range(start_level, num_levels):
        print("Generating level " + str(level + 1) + " out of " + str(num_levels) + "...")
        step = 1 << level
        if level == 0:
            prevStep = 0
        else:
            prevStep = max(0, (1 << (level - 1)))
        halfStep = prevStep

        g = meshBaseLODExtentHeightfieldTexels / 2
        L = float(level)

        # Pad by one element to hide the gap to the next level
        pad = 0
        radius = int(step * (g + pad))
        for y in range(-radius, radius, step):
            for x in range(-radius, radius, step):
                if max(abs(x + halfStep), abs(y + halfStep)) >= g * prevStep:
                    # Cleared the cutout from the previous level. Tessellate the
                    # square.

                    #   A-----B-----C
                    #   | \   |   / |
                    #   |   \ | /   |
                    #   D-----E-----F
                    #   |   / | \   |
                    #   | /   |   \ |
                    #   G-----H-----I

                    A = (float(x), float(y), L)
                    C = (float(x + step), A[1], L)
                    G = (A[0], float(y + step), L)
                    I = (C[0], G[1], L)

                    B = ((A[0] + C[0]) * 0.5, (A[1] + C[1]) * 0.5, (A[2] + C[2]) * 0.5)
                    D = ((A[0] + G[0]) * 0.5, (A[1] + G[1]) * 0.5, (A[2] + G[2]) * 0.5)
                    F = ((C[0] + I[0]) * 0.5, (C[1] + I[1]) * 0.5, (C[2] + I[2]) * 0.5)
                    H = ((G[0] + I[0]) * 0.5, (G[1] + I[1]) * 0.5, (G[2] + I[2]) * 0.5)

                    E = ((A[0] + I[0]) * 0.5, (A[1] + I[1]) * 0.5, (A[2] + I[2]) * 0.5)

                    # Stitch the border into the next level

                    if x == -radius:
                        #   A-----B-----C
                        #   | \   |   / |
                        #   |   \ | /   |
                        #   |     E-----F
                        #   |   / | \   |
                        #   | /   |   \ |
                        #   G-----H-----I
                        index_count = _add_triangle(
                            E, A, G, indices, uvs, points, prev_indices, new_indices, index_count
                        )
                    else:
                        index_count = _add_triangle(
                            E, A, D, indices, uvs, points, prev_indices, new_indices, index_count
                        )
                        index_count = _add_triangle(
                            E, D, G, indices, uvs, points, prev_indices, new_indices, index_count
                        )

                    if y == (radius - step):
                        index_count = _add_triangle(
                            E, G, I, indices, uvs, points, prev_indices, new_indices, index_count
                        )
                    else:
                        index_count = _add_triangle(
                            E, G, H, indices, uvs, points, prev_indices, new_indices, index_count
                        )
                        index_count = _add_triangle(
                            E, H, I, indices, uvs, points, prev_indices, new_indices, index_count
                        )

                    if x == (radius - step):
                        index_count = _add_triangle(
                            E, I, C, indices, uvs, points, prev_indices, new_indices, index_count
                        )
                    else:
                        index_count = _add_triangle(
                            E, I, F, indices, uvs, points, prev_indices, new_indices, index_count
                        )
                        index_count = _add_triangle(
                            E, F, C, indices, uvs, points, prev_indices, new_indices, index_count
                        )

                    if y == -radius:
                        index_count = _add_triangle(
                            E, C, A, indices, uvs, points, prev_indices, new_indices, index_count
                        )
                    else:
                        index_count = _add_triangle(
                            E, C, B, indices, uvs, points, prev_indices, new_indices, index_count
                        )
                        index_count = _add_triangle(
                            E, B, A, indices, uvs, points, prev_indices, new_indices, index_count
                        )
        prev_indices = new_indices.copy()
        new_indices = nb.typed.Dict.empty(key_type=point2, value_type=nb.types.int32)
    return points, uvs, indices
