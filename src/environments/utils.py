__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import numpy as np
from pxr import Gf, UsdGeom
from scipy.spatial.transform import Rotation as R

MOON_ENVIRONMENT_NAME = ""


def set_moon_env_name(name):
    global MOON_ENVIRONMENT_NAME
    MOON_ENVIRONMENT_NAME = name


def get_moon_env_name():
    return MOON_ENVIRONMENT_NAME


def resolve_position(position, height_above_terrain_override=None, get_height_func=None):
    """
    Returns the position unchanged, or with z = terrain height at (x, y) + height_above_terrain_override when set.

    Args:
        position (List[float]): absolute (x, y, z) position.
        height_above_terrain_override (float): optional height above the terrain, replaces z.
        get_height_func (callable): returns the terrain height at an (x, y) position.

    Returns:
        List[float]: the resolved (x, y, z) position.
    """

    position = list(position)
    if height_above_terrain_override is None:
        return position
    if get_height_func is None:
        raise ValueError("height_above_terrain_override is set but no terrain height query is available.")
    position[2] = float(get_height_func((position[0], position[1]))) + height_above_terrain_override
    return position


def set_xform_pose(xform, position, orientation):
    _set_xform_translate(xform, position)
    _set_xform_orientation(xform, orientation)


def _set_xform_translate(xform, position):
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = xform.AddTranslateOp()

    if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
        translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
    else:
        translate_op.Set(Gf.Vec3f(position[0], position[1], position[2]))


def _set_xform_orientation(xform, orientation):
    orient_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            orient_op = op
            break
    if orient_op is None:
        orient_op = xform.AddOrientOp()

    if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
        q = Gf.Quatd(orientation[3], Gf.Vec3d(orientation[0], orientation[1], orientation[2]))
    else:
        q = Gf.Quatf(orientation[3], Gf.Vec3f(orientation[0], orientation[1], orientation[2]))
    orient_op.Set(q)


def transform_orientation_into_xyz(orientation_wxyz):
    """
    This function is used for transforming prim's orientation, that is retrieved from the simulation,
    from [w, x, y, z] format (used by IsaacSim) into [x, y, z] (Euler transformation, human-readable).
    The purpose of this is for testing, debugging, and human-readbility and understanding.
    The discussion available at the following link may help in understanding: https://forums.developer.nvidia.com/t/obtain-the-position-and-orientation-information-of-the-isaac-sim-simulated-environment-for-jetbot/289362

    The following transformations were also tried:

    1) considering orientation input as [x, y, z, w]
        e_xyz_from_xyzw = R.from_quat(q).as_euler('xyz', degrees=True)
        e_zyx_from_xyzw = R.from_quat(q).as_euler('zyx', degrees=True)
    2) considering orientation input as [w, x, y, z]
        q_xyzw_from_wxyz = np.array([q[1], q[2], q[3], q[0]])
        e_xyz_from_wxyz = R.from_quat(q_xyzw_from_wxyz).as_euler('xyz', degrees=True)
        e_zyx_from_wxyz = R.from_quat(q_xyzw_from_wxyz).as_euler('zyx', degrees=True)

    NOTE:  .as_euler('xyz') gives different results than doing .as_euler('zyx) and then reordering the values

    In case you are not receiving the expected results, make sure to proof check the format of your input, and the expected output.
    Otherwise, try using one of the previously tried approaches.
    """
    q = np.array(orientation_wxyz)
    q_xyzw_from_wxyz = np.array([q[1], q[2], q[3], q[0]])
    e_zyx_from_wxyz = R.from_quat(q_xyzw_from_wxyz).as_euler("zyx", degrees=True)
    xyz_orient = [e_zyx_from_wxyz[2], e_zyx_from_wxyz[1], e_zyx_from_wxyz[0]]

    return np.array(xyz_orient)


def transform_orientation_from_xyzw_into_xyz(orientation_xyzw):
    q_xyzw = np.array(orientation_xyzw)
    e_zyx_from_wxyz = R.from_quat(q_xyzw).as_euler("zyx", degrees=True)
    xyz_orient = [e_zyx_from_wxyz[2], e_zyx_from_wxyz[1], e_zyx_from_wxyz[0]]

    return np.array(xyz_orient)
