"""Shared I/O, camera and terrain helpers for the SDG dataset offline tools (no Isaac Sim)."""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import json
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import uniform_filter
from scipy.spatial.transform import Rotation as SSTR

SEM = {"space": 0, "ground": 1, "rock_small": 2, "rock_large": 3}
BIT = {"slope_caution": 1, "slope_hazard": 2, "crater": 4, "rock_small": 8, "rock_large": 16, "unlabelled": 32}
CAM_Q = np.array([0.5, -0.5, -0.5, 0.5])  # rig->camera, xyzw; equals sampling.CAM_LOOK_FORWARD_XYZW


class ContractError(ValueError):
    """The shard does not follow the SDG dataset output contract (docs/sdg_dataset.md, 'Output contract')."""


_FRAME_KEYS = ("index", "terrain_index", "terrain_seed", "frame_in_terrain", "render", "sun", "rig")


def load_manifest(shard: Path) -> dict:
    p = Path(shard) / "manifest.json"
    if not p.exists():
        raise ContractError(f"{p}: manifest.json missing")
    man = json.loads(p.read_text())
    for key in ("base_seed", "data_dir", "intrinsics", "frames"):
        if key not in man:
            raise ContractError(f"{p}: required field {key!r} missing")
    for cam, k in man["intrinsics"].items():
        for key in ("K", "width", "height", "rig_offset_y_m"):
            if key not in k:
                raise ContractError(f"{p}: intrinsics.{cam}.{key} missing")
    for i, fr in enumerate(man["frames"]):
        for key in _FRAME_KEYS:
            if key not in fr:
                raise ContractError(f"{p}: frames[{i}].{key} missing")
        for key in ("position", "quat_xyzw"):
            if key not in fr["rig"]:
                raise ContractError(f"{p}: frames[{i}].rig.{key} missing")
    return man


def label_file(data_dir: Path, cam: str, annot: str, idx: int, epf: int) -> Path:
    return data_dir / f"{cam}_{annot}_id_label" / str(idx // epf) / f"{idx % epf:0{len(str(epf))}d}.json"


def cameras(man: dict) -> list:
    """[(camera name, side letter)] in rig order; mono rigs have only L."""
    names = [c for c in ("cam_left", "cam_right") if c in man["intrinsics"]] or sorted(man["intrinsics"])[:2]
    return list(zip(names, ("L", "R")))


# ----------------------------------------------------------------------------- terrain masks
class Terrain:
    def __init__(self, root: Path, k: int, footprint_m: float = 0.30):
        base = root / "terrains" / f"terrain_{k:04d}"
        z = np.load(str(base) + ".npz")
        self.meta = json.load(open(str(base) + ".json"))
        self.dem = z["dem"].astype(np.float32)
        self.g = float(self.meta["grid_m"])
        self.ox, self.oy = [float(v) for v in self.meta.get("origin_xy_m", [0.0, 0.0])]
        H, W = self.dem.shape
        self.H, self.W = H, W
        # slope over the rover footprint (window in cells), degrees
        win = max(3, int(round(float(footprint_m) / self.g)) | 1)
        sm = uniform_filter(self.dem, size=win, mode="nearest")
        gy, gx = np.gradient(sm, self.g)
        self.slope_deg = np.degrees(np.arctan(np.hypot(gx, gy))).astype(np.float32)
        self.crater = self._crater_mask()
        self.rocks = {r["path"]: r for r in self.meta["rocks"]}

    def _crater_mask(self) -> np.ndarray:
        """Crater interior: disk from the generator's centre/size, kept where the DEM is below the rim."""
        m = np.zeros(self.dem.shape, np.uint8)
        frame = self.meta.get("crater_xy_frame", "lunaryard_index")
        for c in self.meta["craters"]:
            if frame == "lunaryard_index":
                r0 = c["coord_m"][0] / self.g  # DEM axis 0 (row), see generateCraters: coord[0] -> row
                c0 = c["coord_m"][1] / self.g
                rad = max(2.0, c["size_px"] / 2.0)
            else:  # local_xy metres + radius_m (LargeScale crater db)
                x, y = c["xy_local_m"]
                r0 = (self.H - 1) - (y - self.oy) / self.g
                c0 = (x - self.ox) / self.g
                rad = max(2.0, c["radius_m"] / self.g)
                if not (0 <= r0 < self.H and 0 <= c0 < self.W):
                    continue
            sx, sy = c["xy_deformation_factor"] if c["xy_deformation_factor"][0] > 0 else (1.0, 1.0)
            # work only inside the crater's bounding window: thousands of craters x a full 1600^2 grid was the
            # builder's bottleneck (minutes per terrain)
            ry, rx = int(np.ceil(rad * sy)) + 1, int(np.ceil(rad * sx)) + 1
            r_lo, r_hi = max(0, int(r0) - ry), min(self.H, int(r0) + ry + 1)
            c_lo, c_hi = max(0, int(c0) - rx), min(self.W, int(c0) + rx + 1)
            if r_hi - r_lo < 3 or c_hi - c_lo < 3:
                continue
            yy, xx = np.mgrid[r_lo:r_hi, c_lo:c_hi]
            d2 = ((yy - r0) / (rad * sy)) ** 2 + ((xx - c0) / (rad * sx)) ** 2
            disk = d2 <= 1.0
            ring = (d2 > 0.81) & (d2 <= 1.0)
            if disk.sum() < 4 or ring.sum() < 4:
                continue
            sub = self.dem[r_lo:r_hi, c_lo:c_hi]
            rim = np.median(sub[ring])
            m[r_lo:r_hi, c_lo:c_hi][disk & (sub < rim - 0.01)] = 1
        return m

    def lookup(self, x_m: np.ndarray, y_m: np.ndarray):
        """World (x,y) -> DEM cell, using the convention recorded by the generator (mesh = flip(dem, 0))."""
        col = np.clip(np.round((x_m - self.ox) / self.g).astype(int), 0, self.W - 1)
        row = np.clip(np.round((y_m - self.oy) / self.g).astype(int), 0, self.H - 1)
        row = self.H - 1 - row
        return row, col

    def lookup_alt(self, x_m: np.ndarray, y_m: np.ndarray):
        """Transposed hypothesis (rows = x, cols = y, no flip). Only the validator uses this."""
        row = np.clip(np.round((x_m - self.ox) / self.g).astype(int), 0, self.H - 1)
        col = np.clip(np.round((y_m - self.oy) / self.g).astype(int), 0, self.W - 1)
        return row, col


def resolve_data_dir(shard: Path, man: dict) -> Path:
    """The writer's <hash> dir lives inside the shard dir; the manifest path is relative to the box's cwd."""
    local = shard / Path(man["data_dir"]).name
    if local.is_dir():
        return local
    p = Path(man["data_dir"])
    if p.is_dir():
        return p
    raise FileNotFoundError(f"data dir for {shard}: tried {local} and {p}")


# ----------------------------------------------------------------------------- io helpers
def frame_file(data_dir: Path, cam: str, annot: str, idx: int, epf: int, ext: str) -> Path:
    return data_dir / f"{cam}_{annot}" / str(idx // epf) / f"{idx % epf:0{len(str(epf))}d}.{ext}"


def read_colorized(path: Path, label_json: Path):
    img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(path)
    rgba = cv2.cvtColor(img, cv2.COLOR_BGRA2RGBA)
    table = json.load(open(label_json))
    key = (
        rgba[..., 0].astype(np.uint32) << 24
        | rgba[..., 1].astype(np.uint32) << 16
        | rgba[..., 2].astype(np.uint32) << 8
        | rgba[..., 3]
    )
    out = {}
    for k, v in table.items():
        r, g, b, a = [int(t) for t in k.strip("()").split(",")]
        out[(r << 24) | (g << 16) | (b << 8) | a] = v
    return key, out


def unproject(depth: np.ndarray, K: np.ndarray, world_T_cam: np.ndarray):
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    z = depth
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    pc = np.stack([x, -y, -z], -1)  # USD camera: looks down -Z, +Y up
    pw = pc @ world_T_cam[:3, :3].T + world_T_cam[:3, 3]
    return pw


def cam_pose(rig: dict, offset_y: float) -> np.ndarray:
    T = np.eye(4)
    Rr = SSTR.from_quat(rig["quat_xyzw"]).as_matrix()
    T[:3, :3] = Rr @ SSTR.from_quat(CAM_Q).as_matrix()
    T[:3, 3] = np.array(rig["position"]) + Rr @ np.array([0.0, offset_y, 0.0])
    return T


# ----------------------------------------------------------------------------- rock measurement
def measure_rock_blobs(sem, depth_m, K, T):
    """[(mask_label_id, px, depth_m, height_m, method)] for every rock blob (sem >= rock_small); returns (labels, list).

    Blobs with depth >= 65 m or non-finite depth carry height None (method "far").
    """
    dep_m = depth_m
    H, W = dep_m.shape
    n, lab, st, _ = cv2.connectedComponentsWithStats((sem >= SEM["rock_small"]).astype(np.uint8))
    out = []
    if n <= 1:
        return lab, out
    v, u = np.mgrid[0:H, 0:W]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    for i in range(1, n):
        x, y, w, h, area = [int(s) for s in st[i]]
        m = lab == i
        d = float(np.median(dep_m[m]))
        if not np.isfinite(d) or d >= 65.0 or d <= 0:
            out.append((i, area, d, None, "far"))
            continue
        proj = h * d / fy
        height, method = proj, "projected"
        if d <= 6.0:
            band = np.zeros_like(m)
            band[max(0, y + h - 2) : min(H, y + h + max(4, h // 3)), max(0, x - 2) : min(W, x + w + 2)] = True
            g = band & ~m & (sem == SEM["ground"]) & (np.abs(dep_m - d) < max(0.3, 0.15 * d))
            if g.sum() >= 8:
                zr = dep_m[m]
                ur = u[m]
                vr = v[m]
                pr = np.stack([(ur - cx) * zr / fx, -(vr - cy) * zr / fy, -zr], -1) @ T[:3, :3].T + T[:3, 3]
                zg = dep_m[g]
                ug = u[g]
                vg = v[g]
                pg = np.stack([(ug - cx) * zg / fx, -(vg - cy) * zg / fy, -zg], -1) @ T[:3, :3].T + T[:3, 3]
                height = float(np.percentile(pr[:, 2], 97) - np.median(pg[:, 2]))
                method = "contact"
        out.append((i, area, d, max(0.0, float(height)), method))
    return lab, out
