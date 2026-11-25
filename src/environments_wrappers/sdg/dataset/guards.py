"""Isaac-free statistics behind the opt-in quality guards of mode=SDG dataset=<preset>."""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import numpy as np
from scipy.spatial.transform import Rotation as SSTR

from src.environments_wrappers.sdg.dataset.sampling import CAM_LOOK_FORWARD_XYZW


def lit_stats(rgb: np.ndarray, depth: np.ndarray, threshold: int = 12) -> tuple:
    """(lit fraction, mean gray of lit pixels) over terrain pixels (finite depth); lit = gray > threshold."""
    depth = np.asarray(depth).reshape(np.asarray(depth).shape[0], -1)
    rgb = np.asarray(rgb)
    if rgb.ndim != 3:
        rgb = np.frombuffer(rgb, dtype=np.uint8).reshape(depth.shape[0], depth.shape[1], -1)
    gray = rgb[..., :3].astype(np.float32).mean(-1)
    terrain = np.isfinite(depth) & (depth > 0) & (depth < 1e4)
    if terrain.sum() < 1000:
        return 1.0, 125.0
    g = gray[terrain]
    lit = g > threshold
    return float(lit.mean()), float(g[lit].mean()) if lit.any() else 0.0


def exposure_gain(mean_gray: float, ae: dict) -> float:
    if float(ae["min_mean"]) <= mean_gray <= float(ae["max_mean"]):
        return None
    return max(0.25, min(4.0, float(ae["target_mean"]) / max(mean_gray, 1.0)))


def probe_error(
    depth, K, rig_rec: dict, rig_offset_y: float, ground_height, pixel_step: int = 8, max_depth: float = 30.0
):
    """p95 of |rendered ground z - terrain manager z| over a pixel grid; None when nothing can be compared."""
    depth = np.asarray(depth, dtype=np.float32)
    depth = depth.reshape(depth.shape[0], depth.shape[1])
    K = np.asarray(K, dtype=np.float64)
    H, W = depth.shape
    Rr = SSTR.from_quat(rig_rec["quat_xyzw"]).as_matrix()
    Rc = Rr @ SSTR.from_quat(CAM_LOOK_FORWARD_XYZW).as_matrix()
    t = np.array(rig_rec["position"]) + Rr @ np.array([0.0, rig_offset_y, 0.0])
    ys, xs = np.mgrid[0:H:pixel_step, 0:W:pixel_step]
    z = depth[ys, xs]
    ok = np.isfinite(z) & (z > 0) & (z < max_depth)
    u = xs[ok].astype(np.float32)
    v = ys[ok].astype(np.float32)
    zz = z[ok]
    pc = np.stack([(u - K[0, 2]) * zz / K[0, 0], -(v - K[1, 2]) * zz / K[1, 1], -zz], -1)
    pw = pc @ Rc.T + t
    errs = []
    for i in range(0, len(pw), max(1, len(pw) // 300)):
        try:
            errs.append(pw[i, 2] - ground_height(float(pw[i, 0]), float(pw[i, 1])))
        except Exception:  # noqa: BLE001 - points outside the terrain manager's window are skipped
            pass
    if not errs:
        return None
    return float(np.percentile(np.abs(np.array(errs)), 95))
