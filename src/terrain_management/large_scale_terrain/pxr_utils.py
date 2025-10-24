__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import os
import logging

from pxr import UsdGeom, Gf, Usd, Vt, UsdShade, UsdPhysics

from omni.physx.scripts import utils as physx_utils
import omni

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")

collider_modes = [
    "none",
    "convexHull",
    "convexDecomposition",
    "meshSimplification",
    "convexMeshSimplification",
    "boundingSphere",
    "boundingCube",
    "sphereFill",
]


def set_xform_op(prim: Usd.Prim, value, property: UsdGeom.XformOp.Type) -> None:
    """
    Sets a transform operatios on a prim.

    Args:
        prim (Usd.Prim): The prim to set the transform operation.
        value: The value of the transform operation.
        property (UsdGeom.XformOp.Type): The type of the transform operation.
    """

    xform = UsdGeom.Xformable(prim)
    op = None
    for xformOp in xform.GetOrderedXformOps():
        if xformOp.GetOpType() == property:
            op = xformOp
    if op:
        xform_op = op
    else:
        xform_op = xform.AddXformOp(property, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op.Set(value)


def set_scale(prim: Usd.Prim, value: Gf.Vec3d) -> None:
    """
    Sets the scale of a prim.

    Args:
        prim (Usd.Prim): The prim to set the scale.
        value (Gf.Vec3d): The value of the scale.
    """

    set_xform_op(prim, value, UsdGeom.XformOp.TypeScale)


def set_translate(prim: Usd.Prim, value: Gf.Vec3d) -> None:
    """
    Sets the translation of a prim.

    Args:
        prim (Usd.Prim): The prim to set the translation.
        value (Gf.Vec3d): The value of the translation.
    """

    set_xform_op(prim, value, UsdGeom.XformOp.TypeTranslate)


def set_rotate_xyz(prim: Usd.Prim, value: Gf.Vec3d) -> None:
    """
    Sets the rotation of a prim.

    Args:
        prim (Usd.Prim): The prim to set the rotation.
        value (Gf.Vec3d): The value of the rotation.
    """

    set_xform_op(prim, value, UsdGeom.XformOp.TypeRotateXYZ)


def set_orient(prim: Usd.Prim, value: Gf.Quatd) -> None:
    """
    Sets the rotation of a prim.

    Args:
        prim (Usd.Prim): The prim to set the rotation.
        value (Gf.Quatd): The value of the rotation.
    """

    set_xform_op(prim, value, UsdGeom.XformOp.TypeOrient)


def set_transform(prim, value: Gf.Matrix4d) -> None:
    """
    Sets the transform of a prim.

    Args:
        prim (Usd.Prim): The prim to set the transform.
        value (Gf.Matrix4d): The value of the transform.
    """

    set_xform_op(prim, value, UsdGeom.XformOp.TypeTransform)


def set_xform_ops(
    prim,
    translate: Gf.Vec3d = Gf.Vec3d([0, 0, 0]),
    orient: Gf.Quatd = Gf.Quatd(1, Gf.Vec3d([0, 0, 0])),
    scale: Gf.Vec3d = Gf.Vec3d([1, 1, 1]),
) -> None:
    """
    Sets the transform of a prim.

    Args:
        prim (Usd.Prim): The prim to set the transform.
        translate (Gf.Vec3d): The value of the translation.
        orient (Gf.Quatd): The value of the rotation.
        scale (Gf.Vec3d): The value of the scale.
    """

    set_translate(prim, translate)
    set_orient(prim, orient)
    set_scale(prim, scale)


def get_transform(prim: Usd.Prim, parent: Usd.Prim) -> Gf.Matrix4d:
    """
    Gets the transform of a prim relative to its parent.

    Args:
        prim (Usd.Prim): The prim to get the transform.
        parent (Usd.Prim): The parent of the prim.
    """

    return UsdGeom.XformCache(0).ComputeRelativeTransform(prim, parent)[0]


def add_collider(
    stage: Usd.Stage,
    path: str,
    mode: str = "none",
) -> None:
    """
    Adds a collision to a prim.

    Args:
        stage (Usd.Stage): The stage.
        path (str): The path to the prim.
        mode (str, optional): The mode of the collision. Defaults to "none".
    """

    if mode is None:
        mode = "none"

    # Checks that the mode selected by the user is correct.
    assert mode in collider_modes, "Decimation mode: " + mode + " for colliders unknown."
    # Get the prim and add collisions.
    prim = stage.GetPrimAtPath(path)
    physx_utils.setCollider(prim, approximationShape=mode)


def remove_collider(
    stage: Usd.Stage,
    path: str,
) -> None:
    """
    Removes a collision from a prim.

    Args:
        stage (Usd.Stage): The stage.
        path (str): The path to the prim.
    """

    # Get the prim and remove collisions.
    prim = stage.GetPrimAtPath(path)
    physx_utils.removeCollider(prim)


def delete_prim(
    stage: Usd.Stage,
    path: str,
) -> None:
    """
    Deletes a prim.

    Args:
        stage (Usd.Stage): The stage.
        path (str): The path to the prim.
    """

    # Get the prim and delete it.
    stage.RemovePrim(path)


def enable_smooth_shade(
    prim: Usd.Prim,
    extra_smooth: bool = False,
) -> None:
    """
    Enables smooth shading on a prim.

    Args:
        prim (Usd.Prim): The prim on which to enable smooth shading.
        extra_smooth (bool, optional): Toggles the use of the smooth
        method instead of catmullClark. Defaults to False.
    """

    # Sets the subdivision scheme to smooth.
    prim.GetAttribute("subdivisionScheme").Set(UsdGeom.Tokens.catmullClark)
    # Sets the triangle subdivision rule.
    if extra_smooth:
        prim.GetAttribute("triangleSubdivisionRule").Set(UsdGeom.Tokens.smooth)
    else:
        prim.GetAttribute("triangleSubdivisionRule").Set(UsdGeom.Tokens.catmullClark)


def bind_material(stage: Usd.Stage, mtl_prim_path: str, prim_path: str):
    """
    Binds a material to a prim.

    Args:
        stage (Usd.Stage): The stage.
        mtl_prim_path (str): The path to the material prim.
        prim_path (str): The path to the prim.
    """

    mtl_prim = stage.GetPrimAtPath(mtl_prim_path)
    prim = stage.GetPrimAtPath(prim_path)
    shade = UsdShade.Material(mtl_prim)
    UsdShade.MaterialBindingAPI(prim).Bind(shade, UsdShade.Tokens.strongerThanDescendants)


def load_material(material_name: str, material_path: str):
    """
    Loads a material.

    Args:
        material_name (str): The name that will be given to the material in the scene.
        material_path (str): The path to the material file (on the drive).

    Raises:
        AssertionError: If the material file is not found.
    """

    assert os.path.exists(material_path), "Material file not found at: {}".format(material_path)

    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url=material_path,
        mtl_name=material_name,
        mtl_path=os.path.join("/Looks", material_name),
    )
    return os.path.join("/Looks", material_name)


def make_rigid(stage: Usd.Stage, path: str):
    prim = stage.GetPrimAtPath(path)
    rigid = UsdPhysics.RigidBodyAPI.Apply(prim)


def set_texture_path(stage: Usd.Stage, material_path: str, shader_name: str, texture_path: str):
    """
    Sets the texture path for a shader within a material.

    Args:
        stage (Usd.Stage): The stage.
        material_path (str): The path to the material prim.
        shader_name (str): The name of the shader prim inside the material.
        texture_path (str): The absolute path to the texture file.
    """
    material_prim = stage.GetPrimAtPath(material_path)
    if not material_prim.IsValid():
        logger.error(f"Warning: Material not found at {material_path}")
        return
    # Get shader from material prim
    shader = UsdShade.Shader(material_prim.GetPrimAtPath(shader_name))
    if not shader:
        logger.error(f"Warning: Shader '{shader_name}' not found in material '{material_path}'")
        return
    # Write the texture file to albedo map
    file_attribute = shader.GetInput("diffuse_texture")
    if file_attribute:
        file_attribute.Set(texture_path)
    else:
        logger.error(f"Warning: Shader at '{shader.GetPath()}' has no 'file' input.")