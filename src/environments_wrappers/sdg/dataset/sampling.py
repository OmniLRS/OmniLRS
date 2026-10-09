"""Isaac-free sampling and camera math for mode=SDG dataset=<preset>. Draw order matters: a seed must keep meaning the
same frames, so do not reorder the rng calls."""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import math

import numpy as np
from scipy.spatial.transform import Rotation as SSTR

# USD cameras look along local -Z with +Y up; this rotation (x, y, z, w) makes -Z point along the rig's +X
# (forward) and +Y along the rig's +Z (up).
CAM_LOOK_FORWARD_XYZW = (0.5, -0.5, -0.5, 0.5)


def compute_intrinsics(width: int, height: int, rig: dict, focal_mm: float = 24.0) -> dict:
    W, H = int(width), int(height)
    hfov = float(rig["hfov_deg"])
    if rig.get("fx"):
        hfov = math.degrees(2 * math.atan(W / (2 * float(rig["fx"]))))
    hap = 2 * focal_mm * math.tan(math.radians(hfov) / 2)
    vap = hap * H / W
    cx = float(rig.get("cx") or W / 2)
    cy = float(rig.get("cy") or H / 2)
    fx_px = W * focal_mm / hap
    fy_px = H * focal_mm / vap
    if rig.get("fy"):
        fy_px = float(rig["fy"])
        vap = H * focal_mm / fy_px
    return {
        "K": [[fx_px, 0.0, cx], [0.0, fy_px, cy], [0.0, 0.0, 1.0]],
        "width": W,
        "height": H,
        "hfov_deg": hfov,
        "usd": {
            "focal_length_mm": focal_mm,
            "horizontal_aperture_mm": hap,
            "vertical_aperture_mm": vap,
            "horizontal_aperture_offset_mm": (cx - W / 2) * hap / W,
            "vertical_aperture_offset_mm": -(cy - H / 2) * vap / H,
        },
    }


def camera_offsets(camera_names: list, baseline_m: float) -> dict:
    """Rig-frame y offset per camera (rig y points to the image left)."""
    if len(camera_names) == 1:
        return {camera_names[0]: 0.0}
    if len(camera_names) == 2:
        b = float(baseline_m)
        return {camera_names[0]: b / 2, camera_names[1]: -b / 2}
    raise ValueError(f"SDG dataset generation supports 1 (mono) or 2 (stereo) cameras, got {camera_names}")


def sample_sun(rng: np.random.Generator, sun: dict, min_elevation: float = None) -> dict:
    buckets = sun["elevation_buckets"]
    if min_elevation is not None:  # dark-frame retry: only buckets that can clear the minimum
        buckets = [b for b in buckets if b["range"][1] > min_elevation] or buckets[-1:]
    w = np.array([b["weight"] for b in buckets], dtype=float)
    b = buckets[rng.choice(len(buckets), p=w / w.sum())]
    lo = max(float(b["range"][0]), float(min_elevation)) if min_elevation is not None else float(b["range"][0])
    elev = float(rng.uniform(lo, b["range"][1]))
    azim = float(rng.uniform(0.0, 360.0))
    if sun["intensity_mode"] == "constant_ground_brightness":
        # The RGB annotator does not auto-expose like a real camera. Flat-ground irradiance goes as sin(elevation),
        # so scale the sun to keep lit ground at about the same brightness at any elevation (clamped at 5 deg).
        base = float(sun["intensity_base"])
        jit = float(sun["intensity_jitter"])
        inten = (
            base / max(math.sin(math.radians(elev)), math.sin(math.radians(5.0))) * float(rng.uniform(1 - jit, 1 + jit))
        )
    else:
        inten = float(rng.uniform(*sun["intensity"]))
    temp = float(rng.uniform(*sun["temperature_k"]))
    return {
        "elevation_deg": round(elev, 3),
        "azimuth_deg": round(azim, 3),
        "intensity": round(inten, 1),
        "temperature_k": round(temp, 1),
    }


def sun_orientation_wxyz(elevation_deg: float, azimuth_deg: float) -> tuple:
    """Same convention as StellarEngineEnvMixin.create_sun: rotate the sun's parent Xform only."""
    x, y, z, w = SSTR.from_euler("xyz", [0, elevation_deg, azimuth_deg - 90], degrees=True).as_quat()
    return (w, x, y, z)


def sample_rig_pose(rng: np.random.Generator, rig: dict, x: float, y: float, ground_z: float) -> dict:
    h = float(rng.uniform(*rig["height_m"]))
    z = ground_z + h
    yaw = float(rng.uniform(0.0, 360.0))
    pitch = float(rng.uniform(*rig["pitch_deg"]))
    rj = float(rig["roll_jitter_deg"])
    roll = float(rng.uniform(-rj, rj))
    q = SSTR.from_euler("ZYX", [yaw, pitch, roll], degrees=True).as_quat()  # x, y, z, w
    return {
        "position": [round(x, 4), round(y, 4), round(z, 4)],
        "height_above_ground_m": round(h, 4),
        "yaw_deg": round(yaw, 3),
        "pitch_deg": round(pitch, 3),
        "roll_deg": round(roll, 3),
        "quat_xyzw": [round(float(v), 6) for v in q],
    }


def next_walk_location(
    rng: np.random.Generator, walk_xy: np.ndarray, k: int, region_radius_m: float, step_m: list
) -> np.ndarray:
    """Random walk inside a disk around the starting position. 40-80 m steps reuse most streamed terrain blocks."""
    if k == 0:
        return np.zeros(2)
    lo, hi = step_m
    for _ in range(20):
        ang = rng.uniform(0, 2 * np.pi)
        step = rng.uniform(lo, hi)
        cand = walk_xy + step * np.array([np.cos(ang), np.sin(ang)])
        if np.linalg.norm(cand) <= region_radius_m:
            return cand
    return walk_xy * 0.5  # pull back toward the centre if the walk is stuck at the rim
