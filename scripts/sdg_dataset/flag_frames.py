#!/usr/bin/env python3
"""Quality pass over a BUILT shard: find frames whose rocks float above or sink into the rendered mesh.

    python scripts/sdg_dataset/flag_frames.py <built_shard_dir> [--tol 0.06] [--max-range 15] [--min-area 12]

Why: rocks are placed at the full-resolution DEM height, but the rendered terrain is a clipmap whose
texel size grows with distance, so at 20-40 m small craters/rims are smoothed and a rock on a rim
hovers (or one in a bowl is buried). For every rock blob in the left semantic label we unproject the
ground pixels in a band just below the blob, look the same (x, y) up in the DEM crop, and take
median(z_render - z_dem). Below -tol the mesh is under the DEM there (rock floats); above +tol the rock
is buried. Writes quality.json and rejected.txt (fid<TAB>reason) into the built shard dir; pass rejected.txt to
build.py --exclude to drop the flagged frames from the splits. Frames that cannot be analysed carry an "error" entry
in quality.json (counted in summary.errors); exit code 1 when every frame errored.
"""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import argparse
import glob
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.sdg_dataset import _common as B  # noqa: E402


def analyse(built: Path, fid: str, terrains: dict, tol: float, max_range: float, min_area: int):
    meta = json.load(open(built / "meta" / f"{fid}.json"))
    cam = meta["cams"]["L"]
    K = np.array(cam["K"])
    T = np.array(cam["world_T_cam"])
    depth = cv2.imread(str(built / "depth" / f"{fid}_L_mm.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
    sem = cv2.imread(str(built / "labels" / f"{fid}_L_sem.png"), cv2.IMREAD_UNCHANGED)
    k = meta["terrain_index"]
    if k not in terrains:
        terrains[k] = B.Terrain(built, k, 0.30)
    terr = terrains[k]
    valid = depth > 0
    pw = B.unproject(np.where(valid, depth, 1.0), K, T)
    row, col = terr.lookup(pw[..., 0], pw[..., 1])
    inside = (
        (pw[..., 0] > terr.ox + 1)
        & (pw[..., 0] < terr.ox + terr.W * terr.g - 1)
        & (pw[..., 1] > terr.oy + 1)
        & (pw[..., 1] < terr.oy + terr.H * terr.g - 1)
    )
    err = pw[..., 2] - terr.dem[row, col]  # z_render - z_dem
    ground = valid & (sem == 1) & inside & (depth < max_range)
    frame = {
        "mesh_err_p50": None,
        "mesh_err_p90": None,
        "mesh_err_far_p50": None,
        "rocks": [],
        "n_float": 0,
        "n_buried": 0,
    }
    if ground.sum() > 500:
        e = err[ground]
        frame["mesh_err_p50"] = round(float(np.median(e)), 3)
        frame["mesh_err_p90"] = round(float(np.percentile(np.abs(e), 90)), 3)
    far = valid & (sem == 1) & inside & (depth >= max_range) & (depth < 60)
    if far.sum() > 500:
        frame["mesh_err_far_p50"] = round(
            float(np.median(err[far])), 3
        )  # recorded only; far-field geometry masks are not trusted
    H, W = sem.shape
    ground_any = valid & (sem == 1)
    # Objects to test: labelled rocks AND unlabelled geometry blobs (pebble layer; the builder used to leave
    # them as class 0, newer builds map them to ground so they are only visible through the raw label).
    # candidates: labelled rocks, unlabelled-surface pixels (raw class 0 or mask bit 32), and depth islands:
    # compact regions far nearer than their surroundings (a pebble hanging at eye level in front of a far
    # slope is a 2 m island in a 60 m field). Resting rocks are islands too; the bottom-edge tests decide.
    bits_path = built / "masks" / f"{fid}_L_bits.png"
    bits = cv2.imread(str(bits_path), cv2.IMREAD_UNCHANGED) if bits_path.exists() else None
    unlab = (valid & (sem == 0)) | ((bits & 32) > 0) if bits is not None else (valid & (sem == 0))
    # Occluding objects: a pixel far nearer than the pixel a few rows ABOVE it (the background behind the
    # object). Plain ground near the horizon changes depth gradually row to row, so it does not qualify;
    # a pebble at 2 m in front of a 60 m slope does. Grown 2 px, limited to compact blobs below.
    up = np.full_like(depth, np.inf)
    up[6:, :] = depth[:-6, :]
    edge = valid & np.isfinite(up) & (depth < 0.7 * up) & (up - depth > 1.0)
    island = cv2.dilate(edge.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    island &= valid
    objects = ((sem == 2) | (sem == 3) | unlab | island).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(objects, connectivity=8)
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_area or y + h >= H - 3 or area > 4000 or h > 160 or w > 260:
            continue  # compact objects only; big blobs are terrain patches, not rocks
        m = lab == i
        d_rock = float(np.median(depth[m]))
        if not (0 < d_rock < max_range):
            continue
        rec = {
            "bbox": [int(x), int(y), int(w), int(h)],
            "area": int(area),
            "depth_m": round(d_rock, 2),
            "kind": "rock"
            if bool(((sem == 2) | (sem == 3))[m].any())
            else ("unlabelled" if bool(unlab[m].any()) else "island"),
        }
        # Test 1 (no DEM needed): extrapolate the ground depth from the rows just below the blob up to the
        # blob's bottom row. A resting object's bottom depth matches it; a floating object is nearer than
        # the ground would be there, by roughly height_above_ground * distance / camera_height.
        cols = np.where(m.any(0))[0]
        d_bot = float(np.median([depth[np.max(np.where(m[:, c])[0]), c] for c in cols]))
        band = np.zeros_like(ground_any)
        band[y + h + 2 : min(H, y + h + 14), max(0, x - 1) : min(W, x + w + 1)] = True
        g = band & ground_any & (depth < max_range * 1.5)
        if g.sum() >= 6:
            ys, xs = np.where(g)
            dz = depth[ys, xs]
            A = np.vstack([ys, np.ones_like(ys)]).T.astype(np.float32)
            k1, b1 = np.linalg.lstsq(A, dz, rcond=None)[0]
            gap = float(k1 * (y + h) + b1) - d_bot
            rec["gap_m"] = round(gap, 3)
        # Test 2 (needs the DEM crop): mesh-vs-DEM error under the object. Authoritative when available;
        # the gap test alone (objects outside the crop) needs a much larger margin because the row-depth
        # fit is noisy for tiny far blobs (a 17 px rock at 27 m produced a 3 m 'gap' with 1.5 cm true error).
        y0, y1 = min(H - 1, y + h + 1), min(H, y + h + 9)
        band2 = np.zeros_like(ground)
        band2[y0:y1, max(0, x - 2) : min(W, x + w + 2)] = True
        sel = band2 & ground
        if sel.sum() >= 6:
            e = float(np.median(err[sel]))
            rec["ground_err_m"] = round(e, 3)
            if e < -tol:
                rec["issue"] = "floating"
                frame["n_float"] += 1
            elif e > tol:
                rec["issue"] = "buried"
                frame["n_buried"] += 1
        elif rec.get("gap_m") is not None and rec["gap_m"] > max(1.0, 0.15 * d_rock):
            rec["issue"] = "floating"
            frame["n_float"] += 1
        frame["rocks"].append(rec)
    frame["unlabelled_frac"] = round(float((valid & (sem == 0)).sum() / max(1, valid.sum())), 4)
    frame["rock_issue"] = frame["n_float"] > 0 or frame["n_buried"] > 0
    # Near-field darkness: a frame whose only lit terrain is the coarse far mesh (shadowed foreground,
    # striped horizon) passes the generator's whole-frame lit test but is useless. Require that at
    # least half of the terrain within 60 m is lit.
    rgb = cv2.imread(str(built / "images" / f"{fid}_L.png"), cv2.IMREAD_GRAYSCALE)
    near = valid & (depth < 60.0) & (sem != 0)
    if rgb is not None and near.sum() > 2000:
        frame["near_lit_frac"] = round(float((rgb[near] > 12).mean()), 3)
    else:
        frame["near_lit_frac"] = None
    frame["dark_near"] = frame["near_lit_frac"] is not None and frame["near_lit_frac"] < 0.5
    # Stale-mesh locations: the rendered ground sits metres above/below the DEM everywhere in the frame
    # (stock clipmap DEM-buffer bug). |median error| > 0.3 m over near ground -> the frame's geometry is wrong.
    frame["mesh_issue"] = frame["mesh_err_p50"] is not None and abs(frame["mesh_err_p50"]) > 0.3
    # Far-terrain background metrics (coarse 5 m mesh beyond the high-res window): how much of the image is
    # far ground, and how much of that is textureless (flat grey sheet). Recorded; gated by --max-far-flat.
    farm = valid & (depth >= 65.0)  # 16-bit mm depth saturates at 65.535 m: saturated pixels ARE the far terrain
    frame["far_frac"] = round(float(farm.mean()), 4)
    if rgb is not None and farm.sum() > 500:
        g32 = rgb.astype(np.float32)
        mu = cv2.blur(g32, (7, 7))
        var = cv2.blur(g32 * g32, (7, 7)) - mu * mu
        flat = farm & (np.sqrt(np.maximum(var, 0)) < 1.5)
        frame["far_flat_frac"] = round(float(flat.sum() / max(1, valid.sum())), 4)
    else:
        frame["far_flat_frac"] = 0.0
    return frame


_TERR = {}


def _work(args):
    built, fid, tol, max_range, min_area = args
    try:
        return fid, analyse(Path(built), fid, _TERR, tol, max_range, min_area)
    except Exception as e:  # noqa: BLE001
        return fid, {"error": f"{type(e).__name__}: {e}", "rock_issue": False}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("built", help="one built shard dir: <build --out>/<shard_name>")
    ap.add_argument("--tol", type=float, default=0.06)
    ap.add_argument("--max-range", type=float, default=15.0)  # beyond ~20 m the clipmap can sit 1-2 m off the DEM
    ap.add_argument("--min-area", type=int, default=12)
    ap.add_argument("--max-far-flat", type=float, default=0.02)
    ap.add_argument("--workers", type=int, default=0)
    a = ap.parse_args(argv)
    built = Path(a.built)
    fids = sorted(os.path.basename(f)[:-6] for f in glob.glob(str(built / "images" / "*_L.png")))
    jobs = [(str(built), f, a.tol, a.max_range, a.min_area) for f in fids]
    if a.workers == 1:
        results = [_work(j) for j in jobs]
    else:
        from concurrent.futures import ProcessPoolExecutor

        with ProcessPoolExecutor(max_workers=a.workers or max(1, min(16, os.cpu_count() or 1))) as ex:
            results = list(ex.map(_work, jobs, chunksize=4))
    q = dict(results)
    reasons = {}
    for f, r in q.items():
        if r.get("mesh_issue"):
            reasons[f] = "mesh_issue"
        elif r.get("rock_issue"):
            reasons[f] = "rock_issue"
        elif r.get("dark_near"):
            reasons[f] = "dark_near"
        elif r.get("far_flat_frac", 0) > a.max_far_flat:
            reasons[f] = "far_flat"
    (built / "rejected.txt").write_text("".join(f"{f}\t{r}\n" for f, r in sorted(reasons.items())))
    n_err = sum(1 for r in q.values() if "error" in r)
    summary = {
        "frames": len(fids),
        "rejected": len(reasons),
        "errors": n_err,
        **{
            k: sum(1 for v in reasons.values() if v == k) for k in ("mesh_issue", "rock_issue", "dark_near", "far_flat")
        },
        "tol_m": a.tol,
        "max_range_m": a.max_range,
    }
    json.dump({"summary": summary, "frames": q}, open(built / "quality.json", "w"))
    print(json.dumps(summary))
    if fids and n_err == len(fids):
        print(f"ERROR: every frame failed to analyse (first: {next(iter(q.values()))['error']})", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
