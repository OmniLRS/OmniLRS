"""Synthetic SDG_Dataset shards for the offline-tool tests: flat ground, optional rocks, optional crater."""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import json
from pathlib import Path

import cv2
import numpy as np

W, H = 64, 48
K = [[40.0, 0.0, 32.0], [0.0, 40.0, 24.0], [0.0, 0.0, 1.0]]
# rig 1 m above z=0, pitched 90 deg down: camera looks along world -Z; image up = world +X
RIG = {"position": [0.0, 0.0, 1.0], "quat_xyzw": [0.0, 0.7071068, 0.0, 0.7071068], "height_above_ground_m": 1.0}


def _write_colorized(png: Path, js: Path, ids: np.ndarray, table: dict) -> None:
    png.parent.mkdir(parents=True, exist_ok=True)
    js.parent.mkdir(parents=True, exist_ok=True)
    rgba = np.zeros((*ids.shape, 4), np.uint8)
    out = {}
    for i, lab in table.items():
        color = (i * 40 % 256, 60, 90, 255)
        rgba[ids == i] = color
        out[str(color)] = lab
    cv2.imwrite(str(png), cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
    js.write_text(json.dumps(out))


def make_shard(
    root: Path,
    rocks=((0.10, 20, 30), (0.03, 40, 12)),
    cameras=("cam_left", "cam_right"),
    crater: bool = False,
    dem_tilt_deg: float = 0.0,
    frames: int = 1,
    drop_files_for: tuple = (),
    table_heights=None,
) -> Path:
    """rocks: (height_m, row, col[, depth_m[, size_px]]) boxes, 6x6 px by default. A depth_m given puts the whole
    box at that depth instead of 1 m - height_m. table_heights: the generator rock table's heights, one per rock;
    defaults to the depth-encoded heights (pass different values to tell measurement from table). Returns the shard dir."""
    rocks = [tuple(r) for r in rocks]
    if table_heights is None:
        table_heights = [r[0] for r in rocks]
    shard = root / "shard_00007"
    data = shard / "AbCdEfGh12345678"
    (shard / "terrains").mkdir(parents=True)
    g = 0.05
    dem = np.zeros((200, 200), np.float32)
    dem += np.tan(np.radians(dem_tilt_deg)) * (np.arange(200, dtype=np.float32) * g)[None, :]
    craters = []
    if crater:
        yy, xx = np.mgrid[0:200, 0:200]
        d2 = ((yy - 100) ** 2 + (xx - 100) ** 2) / 20.0**2
        dem[d2 <= 1] -= 0.3 * (1 - d2[d2 <= 1])
        craters.append(
            {"xy_local_m": [0.0, 0.0], "radius_m": 1.0, "xy_deformation_factor": [1.0, 1.0], "rotation_deg": 0.0}
        )
    rock_table = [
        {"path": f"/World/Rocks/r{i}", "height_above_ground_m": h, "footprint_m": 0.3, "position": [0, 0, 0]}
        for i, h in enumerate(table_heights)
    ]
    np.savez_compressed(shard / "terrains" / "terrain_0000.npz", dem=dem)
    (shard / "terrains" / "terrain_0000.json").write_text(
        json.dumps(
            {
                "grid_m": g,
                "origin_xy_m": [-5.0, -5.0],
                "crater_xy_frame": "local_xy",
                "craters": craters,
                "rocks": rock_table,
            }
        )
    )
    offs = {cameras[0]: 0.0} if len(cameras) == 1 else {cameras[0]: 0.06, cameras[1]: -0.06}
    man = {
        "base_seed": 7,
        "environment": "DatasetLargeScale",
        "data_dir": f"data/sdg_dataset/shard_00007/{data.name}",
        "intrinsics": {
            c: {"K": K, "width": W, "height": H, "baseline_m": 0.12, "rig_offset_y_m": offs[c]} for c in cameras
        },
        "frames": [],
        "partial": False,
    }
    for idx in range(frames):
        man["frames"].append(
            {
                "index": idx,
                "terrain_index": 0,
                "terrain_seed": 7000,
                "frame_in_terrain": idx,
                "render": "rt",
                "sun": {"elevation_deg": 20.0, "azimuth_deg": 0.0, "intensity": 1000.0, "temperature_k": 6000.0},
                "rig": RIG,
            }
        )
        if idx in drop_files_for:
            continue
        for cam in cameras:
            name = f"{idx:04d}"
            depth = np.full((H, W), 1.0, np.float32)
            sem_ids = np.ones((H, W), np.int32)  # 1 = ground
            ins_ids = np.zeros((H, W), np.int32)
            for i, (h, r, c, *extra) in enumerate(rocks):
                d = extra[0] if extra else 1.0 - h
                n = extra[1] if len(extra) > 1 else 6
                depth[r : r + n, c : c + n] = d
                sem_ids[r : r + n, c : c + n] = 2
                ins_ids[r : r + n, c : c + n] = 10 + i
            (data / f"{cam}_rgb" / "0").mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(data / f"{cam}_rgb" / "0" / f"{name}.png"), np.full((H, W, 3), 120, np.uint8))
            (data / f"{cam}_depth" / "0").mkdir(parents=True, exist_ok=True)
            np.savez_compressed(data / f"{cam}_depth" / "0" / f"{name}.npz", depth=depth)
            _write_colorized(
                data / f"{cam}_semantic_segmentation" / "0" / f"{name}.png",
                data / f"{cam}_semantic_segmentation_id_label" / "0" / f"{name}.json",
                sem_ids,
                {1: {"class": "terrain"}, 2: {"class": "rock"}},
            )
            _write_colorized(
                data / f"{cam}_instance_segmentation" / "0" / f"{name}.png",
                data / f"{cam}_instance_segmentation_id_label" / "0" / f"{name}.json",
                ins_ids,
                {10 + i: f"/World/Rocks/r{i}/mesh" for i in range(len(rocks))},
            )
    (shard / "manifest.json").write_text(json.dumps(man))
    return shard
