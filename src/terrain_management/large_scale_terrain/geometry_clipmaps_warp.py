__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import warp as wp


@wp.kernel
def _preprocess(
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    points: wp.array(dtype=wp.vec2f),
    coord: wp.vec2f,
    mpp: float,
    dem_shape: wp.vec2f,
):
    """
    Pre-process the coordinates of the points to be queried.
    It converts the pixel coordinates to meters and adds the offset.
    It also clamps the coordinates to the DEM shape.

    Args:
        x (wp.array(dtype=float)): x coordinates of the points (this is the output).
        y (wp.array(dtype=float)): y coordinates of the points (this is the output).
        points (wp.array(dtype=wp.vec2f)): points to query.
        coord (wp.vec2f): offset to add to the coordinates.
        mpp (float): meters per pixel.
        dem_shape (wp.vec2f): shape of the DEM.
    """

    tid = wp.tid()
    x[tid] = points[tid][0] / mpp + coord[0]
    y[tid] = points[tid][1] / mpp + coord[1]
    x[tid] = wp.atomic_min(x, tid, dem_shape[0] - 1.0)
    y[tid] = wp.atomic_min(y, tid, dem_shape[1] - 1.0)
    x[tid] = wp.atomic_max(x, tid, 0.0)
    y[tid] = wp.atomic_max(y, tid, 0.0)


@wp.func
def _get_4x4_mat(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: float,
    y: float,
    out: wp.mat44f,
) -> wp.mat44f:
    """
    Gets a patch of 4x4 values from the DEM centered at the given coordinates (x, y).
    The function automatically clamps the coordinates to the DEM shape.

    Args:
        dem (wp.array(dtype=float)): DEM.
        dem_shape (wp.vec2i): shape of the DEM.
        x (float): x coordinate.
        y (float): y coordinate.
        out (wp.mat44f): output matrix.
    """

    x0 = wp.max(int(x) - 1, 0)
    y0 = wp.max(int(y) - 1, 0)
    x1 = wp.min(x0 + 1, dem_shape[0] - 1) * dem_shape[1]
    x2 = wp.min(x0 + 2, dem_shape[0] - 1) * dem_shape[1]
    x3 = wp.min(x0 + 3, dem_shape[0] - 1) * dem_shape[1]
    x0 = x0 * dem_shape[1]
    y1 = wp.min(y0 + 1, dem_shape[1] - 1)
    y2 = wp.min(y0 + 2, dem_shape[1] - 1)
    y3 = wp.min(y0 + 3, dem_shape[1] - 1)
    out[0, 0] = dem[x0 + y0]
    out[1, 0] = dem[x1 + y0]
    out[2, 0] = dem[x2 + y0]
    out[3, 0] = dem[x3 + y0]
    out[0, 1] = dem[x0 + y1]
    out[1, 1] = dem[x1 + y1]
    out[2, 1] = dem[x2 + y1]
    out[3, 1] = dem[x3 + y1]
    out[0, 2] = dem[x0 + y2]
    out[1, 2] = dem[x1 + y2]
    out[2, 2] = dem[x2 + y2]
    out[3, 2] = dem[x3 + y2]
    out[0, 3] = dem[x0 + y3]
    out[1, 3] = dem[x1 + y3]
    out[2, 3] = dem[x2 + y3]
    out[3, 3] = dem[x3 + y3]
    return out


@wp.kernel
def _get_values_wp_4x4(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    out: wp.array(dtype=wp.mat44f),
):
    """
    Gets all the 4x4 DEM patches from the array of x and y coordinates.

    Args:
        dem (wp.array(dtype=float)): DEM.
        dem_shape (wp.vec2i): shape of the DEM.
        x (wp.array(dtype=float)): x coordinates.
        y (wp.array(dtype=float)): y coordinates.
        out (wp.array(dtype=wp.mat44f)): output
    """

    tid = wp.tid()
    out[tid] = _get_4x4_mat(dem, dem_shape, x[tid], y[tid], out[tid])


@wp.func
def _get_2x2_mat(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: float,
    y: float,
    out: wp.mat22f,
) -> wp.mat22f:
    """
    Gets a patch of 2x2 values from the DEM centered at the given coordinates (x, y).
    The function automatically clamps the coordinates to the DEM shape.

    Args:
        dem (wp.array(dtype=float)): DEM.
        dem_shape (wp.vec2i): shape of the DEM.
        x (float): x coordinate.
        y (float): y coordinate.
        out (wp.mat22f): output matrix.
    """

    x0 = int(x)
    y0 = int(y)
    x1 = wp.min(x0 + 1, dem_shape[0] - 1) * dem_shape[1]
    x0 = x0 * dem_shape[1]
    y1 = wp.min(y0 + 1, dem_shape[1] - 1)
    out[0, 0] = dem[x0 + y0]
    out[1, 0] = dem[x1 + y0]
    out[0, 1] = dem[x0 + y1]
    out[1, 1] = dem[x1 + y1]
    return out


@wp.kernel
def _get_values_wp_2x2(
    dem: wp.array(dtype=float),
    dem_shape: wp.vec2i,
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    out: wp.array(dtype=wp.mat22f),
):
    """
    Gets all the 2x2 DEM patches from the array of x and y coordinates.

    Args:
        dem (wp.array(dtype=float)): DEM.
        dem_shape (wp.vec2i): shape of the DEM.
        x (wp.array(dtype=float)): x coordinates.
        y (wp.array(dtype=float)): y coordinates.
        out (wp.array(dtype=wp.mat22f)): output
    """

    tid = wp.tid()
    out[tid] = _get_2x2_mat(dem, dem_shape, x[tid], y[tid], out[tid])


@wp.func
def _bilinear_interpolator(
    x: float,
    y: float,
    q: wp.mat22f,
):
    """
    Bilinear interpolation of a single point.

    Args:
        x (float): x coordinate.
        y (float): y coordinate.
        q (wp.mat22f): 2x2 matrix.
    """

    x2 = x - wp.trunc(x)
    y2 = y - wp.trunc(y)
    return (1.0 - x2) * (1.0 - y2) * q[0, 0] + x2 * (1.0 - y2) * q[1, 0] + (1.0 - x2) * y2 * q[0, 1] + x2 * y2 * q[1, 1]


@wp.kernel
def _bilinear_interpolation(
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    q: wp.array(dtype=wp.mat22f),
    out: wp.array(dtype=float),
):
    """
    Bilinear interpolation of all the points in the array.

    Args:
        x (wp.array(dtype=float)): x coordinates.
        y (wp.array(dtype=float)): y coordinates.
        q (wp.array(dtype=wp.mat22f)): 2x2 matrices.
        out (wp.array(dtype=float)): output.
    """

    tid = wp.tid()
    out[tid] = _bilinear_interpolator(x[tid], y[tid], q[tid])


@wp.func
def _cubic_interpolator(
    x: float,
    coeffs: wp.vec4f,
):
    """
    Cubic interpolation of a single point.

    Args:
        x (float): x coordinate.
        coeffs (wp.vec4f): coefficients.
    """

    x2 = x - wp.trunc(x)
    a = -0.5
    coeffs[0] = a * (x2 * (1.0 - x2 * (2.0 - x2)))
    coeffs[1] = a * (-2.0 + x2 * x2 * (5.0 - 3.0 * x2))
    coeffs[2] = a * (x2 * (-1.0 + x2 * (-4.0 + 3.0 * x2)))
    coeffs[3] = a * (x2 * x2 * (1.0 - x2))
    return coeffs


@wp.kernel
def _bicubic_interpolation(
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    q: wp.array(dtype=wp.mat44f),
    coeffs: wp.array(dtype=wp.vec4f),
    out: wp.array(dtype=float),
):
    """
    Bicubic interpolation of all the points in the array.

    Args:
        x (wp.array(dtype=float)): x coordinates.
        y (wp.array(dtype=float)): y coordinates.
        q (wp.array(dtype=wp.mat44f)): 4x4 matrices.
        coeffs (wp.array(dtype=wp.vec4f)): coefficients.
        out (wp.array(dtype=float)): output.
    """

    tid = wp.tid()
    coeffs[tid] = _cubic_interpolator(x[tid], coeffs[tid])
    a0 = (
        q[tid][0, 0] * coeffs[tid][0]
        + q[tid][1, 0] * coeffs[tid][1]
        + q[tid][2, 0] * coeffs[tid][2]
        + q[tid][3, 0] * coeffs[tid][3]
    )
    a1 = (
        q[tid][0, 1] * coeffs[tid][0]
        + q[tid][1, 1] * coeffs[tid][1]
        + q[tid][2, 1] * coeffs[tid][2]
        + q[tid][3, 1] * coeffs[tid][3]
    )
    a2 = (
        q[tid][0, 2] * coeffs[tid][0]
        + q[tid][1, 2] * coeffs[tid][1]
        + q[tid][2, 2] * coeffs[tid][2]
        + q[tid][3, 2] * coeffs[tid][3]
    )
    a3 = (
        q[tid][0, 3] * coeffs[tid][0]
        + q[tid][1, 3] * coeffs[tid][1]
        + q[tid][2, 3] * coeffs[tid][2]
        + q[tid][3, 3] * coeffs[tid][3]
    )
    coeffs[tid] = _cubic_interpolator(y[tid], coeffs[tid])
    out[tid] = a0 * coeffs[tid][0] + a1 * coeffs[tid][1] + a2 * coeffs[tid][2] + a3 * coeffs[tid][3]


@wp.func
def _normal_on_grid(q: wp.mat22f, grid_size: float) -> wp.vec3f:
    """
    Computes the normal of a quad on a regular grid. The quad can be split into two triangles.
    We thus compute the normal of each triangle and take the average. To do so, we make the
    following assumptions:

    q[0,0] (a) ---- q[0,1] (b)
       |               |
       |       X       | grid_size
       |               |
    q[1,0] (c) ---- q[1,1] (d)

    # Watch out for the flip...
    a = (0, 0, a_z)
    b = (0, grid_size, b_z)
    c = (grid_size, 0, c_z)
    d = (grid_size, grid_size, d_z)

    Compute cross r1 = (b - a, c - a)
    Compute cross r2 = (b - d, c - d)
    normal = (r1 - r2) / 2

    b - a = (0, grid_size, b_z - a_z),  c - a = (grid_size, 0, c_z - a_z)
    b - d = (-grid_size, 0, b_z - d_z), c - d = (0, -grid_size, c_z - d_z)

    u = (u0, u1, u2), v = (v0, v1, v2)
    u x v = (u1v2 - u2v1, u2v0 - u0v2, u0v1 - u1v0)

    r1 = (-(b_z - a_z) * grid_size, -(c_z - a_z) * grid_size, grid_size * grid_size)
    r2 = (-(c_z - d_z) * grid_size, -(b_z - d_z) * grid_size, -grid_size * grid_size)

    Take the average:
    Flip r1 to ensure both vectors are pointing up (they can't point down since the surface is always oriented upwards)
    (-r1 + r2) / 2 = ( grid_size * (-b_z + a_z + c_z - d_z) / 2,
                       grid_size * (-c_z + a_z + b_z - d_z) / 2,
                       grid_size * grid_size)

    Args:
        q (wp.mat22f): 2x2 matrix.
        grid_size (float): grid size.

    Returns:
        wp.vec3f: normal vector.
    """

    vec = wp.vec3f(
        grid_size / 2.0 * (-q[1, 0] + q[0, 0] + q[0, 1] - q[1, 1]),
        grid_size / 2.0 * (-q[0, 1] + q[0, 0] + q[1, 0] - q[1, 1]),
        grid_size * grid_size,
    )
    return vec / wp.length(vec)


@wp.kernel
def _preprocess_multi_points(
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    coord: wp.vec2f,
    mpp: float,
    map_offset: wp.vec2f,
):
    """
    Pre-process the coordinates of the points to be queried.
    It converts meters coordinates to pixel coordinates and adds an offset.
    The offset is used to ensure that the points are centered at the center
    of the DEM and not the origin (top-left corner).

    Args:
        x (wp.array(dtype=float)): x coordinates of the points (this is the output).
        y (wp.array(dtype=float)): y coordinates of the points (this is the output).
        coord (wp.vec2f): offset to remove from the coordinates.
        mpp (float): meters per pixel.
        map_offset (wp.vec2f): offset to add to the coordinates.
    """

    tid = wp.tid()
    x[tid] = (x[tid] - coord[0] + map_offset[0]) / mpp
    y[tid] = (y[tid] - coord[1] + map_offset[1]) / mpp


@wp.kernel
def _bilinear_interpolation_and_random_orientation(
    x: wp.array(dtype=float),
    y: wp.array(dtype=float),
    q: wp.array(dtype=wp.mat22f),
    grid_size: float,
    normal: wp.array(dtype=wp.vec3f),
    quat: wp.array(dtype=wp.quatf),
    height: wp.array(dtype=float),
    seed: wp.int32,
):
    """
    This generates a vector normal to the surface with a random orientation for each querried point.
    It also computes the height of the desired points on the surface.

    Args:
        x (wp.array(dtype=float)): x coordinates.
        y (wp.array(dtype=float)): y coordinates.
        q (wp.array(dtype=wp.mat22f)): 2x2 matrices.
        grid_size (float): grid size.
        normal (wp.array(dtype=wp.vec3f)): output normal.
        quat (wp.array(dtype=wp.quatf)): output quaternion.
        height (wp.array(dtype=float)): output height.
        seed (wp.int32): seed for the random number generator.
    """

    tid = wp.tid()
    normal[tid] = _normal_on_grid(q[tid], grid_size)
    quat[tid] = _get_random_tangent_vector(normal[tid], wp.rand_init(seed, tid * 3))
    height[tid] = _bilinear_interpolator(x[tid], y[tid], q[tid])


@wp.func
def _get_random_tangent_vector(normal: wp.vec3f, state: wp.uint32) -> wp.quatf:
    """
    Generates a random tangent vector to the normal vector.
    Using the normal vector, we build a base (vx, vy, normal) that will provide
    a random rotation around that vector.
    To do that, we first generate a random vector vx, and then get the closest vector
    to vx that is orthogonal to the normal vector. Then we repeat the process to get
    vy a vector that is orthogonal to both the normal and vx.
    From there we can build a rotation matrix and convert it to a quaternion.

    Args:
        normal (wp.vec3f): normal vector.
        state (wp.uint32): random number generator state.

    Returns:
        wp.quatf: random quaternion normal to the surface.
    """
    vx = wp.vec3f(
        wp.randf(state, -1.0, 1.0),
        wp.randf(state + wp.uint32(1), -1.0, 1.0),
        wp.randf(state + wp.uint32(2), -1.0, 1.0),
    )
    vx = wp.cross(normal, vx)
    vx = vx / wp.length(vx)
    vy = wp.cross(normal, vx)
    mat = wp.mat33f(vx, vy, normal)
    return wp.quat_from_matrix(mat)
