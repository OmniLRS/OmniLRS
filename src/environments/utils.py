from pxr import UsdGeom, Gf

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