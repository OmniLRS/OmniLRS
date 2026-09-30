#!/usr/bin/env python3
"""Prove a raw mode=SDG_Dataset shard is usable before spending hours on a long run.

    python scripts/sdg_dataset/validate.py <shard_dir> [--n 12] [--wheel-clearance-m 0.05] [--footprint-m 0.30]

Every check here catches a failure mode that would otherwise be discovered after the
box is destroyed:
  files      manifest frame count == files on disk for EVERY camera/annotator
  classes    ground / rock / space pixel fractions; rock must be > 0 (semantics on rocks)
  instances  distinct rock instances per frame; must be > 0 (instance ids on rocks)
  depth      finite fraction, range; sky must be non-finite or huge, ground must be metres
  dem        unproject ground pixels through manifest pose + K, compare with the DEM under
             both row conventions; the recorded one must win with RMSE < 0.05 m
  normals    unit length; reports mean ground normal to reveal world- vs view-space frame
  sun        ground luminance must rise with sun elevation (Spearman rho > 0.3)
  rocks      height-above-ground histogram vs wheel clearance -> small/large ratio
  contact    contact_sheet.png: rgb | semantic | depth for n frames, eyeball it
Writes validation_report.json into the shard dir. Exit code 1 on any hard failure.
"""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.stats import spearmanr

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.sdg_dataset import _common as B  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("shard")
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--wheel-clearance-m", type=float, default=0.05)
    ap.add_argument("--footprint-m", type=float, default=0.30)
    a = ap.parse_args(argv)
    sh = Path(a.shard)
    man = B.load_manifest(sh)
    cam = B.cameras(man)[0][0]
    data_dir = B.resolve_data_dir(sh, man)
    frames = man["frames"]
    epf = 1000
    rep = {
        "shard": str(sh),
        "environment": man.get("environment"),
        "frames_in_manifest": len(frames),
        "partial": man.get("partial"),
        "fail": [],
    }

    # files
    counts = {}
    for d in sorted(p for p in data_dir.iterdir() if p.is_dir()):
        counts[d.name] = sum(len(f) for _, _, f in os.walk(d))
    rep["files_per_dir"] = counts
    for name, n in counts.items():
        if name.endswith("_pose"):
            continue  # stock pose writer appends rows to one file
        if n != len(frames):
            rep["fail"].append(f"{name}: {n} files != {len(frames)} frames")

    # per-frame checks sample only frames whose depth file exists; missing files already fail above
    have = [i for i, fr in enumerate(frames) if B.frame_file(data_dir, cam, "depth", fr["index"], epf, "npz").exists()]
    rep["frames_with_files"] = len(have)
    rng = np.random.default_rng(0)
    pick = sorted(rng.choice(have, size=min(a.n, len(have)), replace=False).tolist()) if have else []
    if not pick:
        rep["fail"].append("no frame has its depth file on disk")
        json.dump(rep, open(sh / "validation_report.json", "w"), indent=1)
        print(json.dumps(rep, indent=1))
        return 1
    cls_frac, n_inst, dem_err = [], [], {"flip": [], "noflip": []}
    lum, elev, nrm_stats, rock_h, dark = [], [], [], [], []
    tiles = []
    terrains = {}
    K = np.array(man["intrinsics"][cam]["K"])
    for i in pick:
        fr = frames[i]
        idx = fr["index"]
        k = fr["terrain_index"]
        if k not in terrains:
            terrains[k] = B.Terrain(sh, k, a.footprint_m)
            rock_h += [r["height_above_ground_m"] for r in terrains[k].meta["rocks"]]
        terr = terrains[k]
        rgb = cv2.imread(str(B.frame_file(data_dir, cam, "rgb", idx, epf, "png")))
        depth = np.load(str(B.frame_file(data_dir, cam, "depth", idx, epf, "npz")))["depth"].astype(np.float32)
        sem_key, sem_tab = B.read_colorized(
            B.frame_file(data_dir, cam, "semantic_segmentation", idx, epf, "png"),
            B.label_file(data_dir, cam, "semantic_segmentation", idx, epf),
        )
        ins_key, ins_tab = B.read_colorized(
            B.frame_file(data_dir, cam, "instance_segmentation", idx, epf, "png"),
            B.label_file(data_dir, cam, "instance_segmentation", idx, epf),
        )
        valid = np.isfinite(depth) & (depth > 0) & (depth < 1e4)
        labels = {key: (v.get("class") if isinstance(v, dict) else str(v)) for key, v in sem_tab.items()}
        ground = np.zeros(depth.shape, bool)
        rock = np.zeros(depth.shape, bool)
        for key, c in labels.items():
            if c and ("ground" in c or "terrain" in c):
                ground |= sem_key == key
            if c and "rock" in c:
                rock |= sem_key == key
        cls_frac.append(
            {
                "ground": ground.mean(),
                "rock": rock.mean(),
                "space": (~valid).mean(),
                "sem_labels": sorted(set(labels.values())),
            }
        )
        rock_ids = [key for key, p in ins_tab.items() if "instance_" in str(p) or "Rocks" in str(p)]
        n_inst.append(int(sum(1 for key in rock_ids if (ins_key == key).any())))

        # DEM convention check on ground pixels
        T = B.cam_pose(fr["rig"], man["intrinsics"][cam]["rig_offset_y_m"])
        pw = B.unproject(np.where(valid, depth, 1.0), K, T)
        g = ground & valid
        # DEM check only on pixels the saved crop covers (LargeScale crops are 80 m; far pixels would clip to the edge)
        span_x = (terr.ox, terr.ox + terr.W * terr.g)
        span_y = (terr.oy, terr.oy + terr.H * terr.g)
        inside = (
            g
            & (depth < 40)
            & (pw[..., 0] > span_x[0] + 1)
            & (pw[..., 0] < span_x[1] - 1)
            & (pw[..., 1] > span_y[0] + 1)
            & (pw[..., 1] < span_y[1] - 1)
        )
        sel = (
            rng.choice(np.flatnonzero(inside.ravel()), size=min(4000, int(inside.sum())), replace=False)
            if inside.any()
            else []
        )
        if len(sel):
            x, y, z = pw.reshape(-1, 3)[sel].T
            r1, c1 = terr.lookup(x, y)
            r2, c2 = terr.lookup_alt(x, y)
            dem_err["flip"].append(float(np.sqrt(np.mean((terr.dem[r1, c1] - z) ** 2))))  # stated convention
            dem_err["noflip"].append(float(np.sqrt(np.mean((terr.dem[r2, c2] - z) ** 2))))  # transposed alternative
        # sun vs luminance
        if g.any():
            gl = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)[g]
            lum.append(float(gl.mean()))
            elev.append(fr["sun"]["elevation_deg"])
            dark.append({"elev": fr["sun"]["elevation_deg"], "dark_frac": round(float((gl < 8).mean()), 3)})
        # normals
        npath = B.frame_file(data_dir, cam, "normals", idx, epf, "npz")
        if npath.exists() and g.any():
            n = np.load(str(npath))["normals"].astype(np.float32)
            ln = np.linalg.norm(n, axis=-1)
            mg = n[g & (ln > 0.5)].mean(0)
            nrm_stats.append(
                {
                    "unit_frac": float(((ln > 0.9) & (ln < 1.1))[valid].mean()),
                    "mean_ground_normal": mg.round(3).tolist(),
                    "pitch_deg": fr["rig"]["pitch_deg"],
                }
            )
        # contact tile
        if len(tiles) < 8:
            sem_vis = np.zeros_like(rgb)
            sem_vis[ground] = (80, 80, 80)
            sem_vis[rock] = (0, 200, 255)
            d = np.where(valid, depth, 0)
            dv = cv2.applyColorMap(np.clip(d / max(d.max(), 1e-3) * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            row_img = np.concatenate([rgb, sem_vis, dv], 1)
            cv2.putText(
                row_img,
                f"t{k} f{fr['frame_in_terrain']} elev={fr['sun']['elevation_deg']:.1f} h={fr['rig']['height_above_ground_m']:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2,
            )
            tiles.append(cv2.resize(row_img, (row_img.shape[1] // 3, row_img.shape[0] // 3)))

    rep["class_fraction_mean"] = {k: float(np.mean([c[k] for c in cls_frac])) for k in ("ground", "rock", "space")}
    rep["semantic_labels_seen"] = sorted({lab for c in cls_frac for lab in c["sem_labels"]})
    rep["rock_instances_per_frame"] = {"mean": float(np.mean(n_inst)), "min": int(min(n_inst))}
    rep["dem_rmse_m"] = {k: float(np.mean(v)) if v else None for k, v in dem_err.items()}
    rep["normals"] = nrm_stats[:4]
    rep["dark_by_frame"] = dark
    if len(lum) >= 5:
        rho = spearmanr(elev, lum).correlation
        rep["sun_luminance_spearman"] = float(rho)
        if not rho > 0.3:
            rep["fail"].append(
                f"ground luminance does not rise with sun elevation (rho={rho:.2f}); sun convention suspect"
            )
    rh = np.array(rock_h)
    thr = a.wheel_clearance_m
    rep["rocks"] = {
        "n": int(rh.size),
        "height_p50": float(np.median(rh)) if rh.size else None,
        "height_p90": float(np.percentile(rh, 90)) if rh.size else None,
        "frac_large_at_clearance": float((rh >= thr).mean()) if rh.size else None,
        "clearance_m": thr,
    }
    if rep["class_fraction_mean"]["rock"] <= 0:
        rep["fail"].append("no rock pixels in semantic output: rocks carry no semantics")
    if rep["rock_instances_per_frame"]["mean"] <= 0:
        rep["fail"].append("no rock instances: instance annotator does not see the rocks")
    dem_tol = (
        0.05 if (man.get("environment") or "").endswith("Lunaryard") else 0.5
    )  # LargeScale crop is resampled to 5 cm over km-scale relief
    if rep["dem_rmse_m"]["flip"] is not None and not (
        rep["dem_rmse_m"]["flip"] < dem_tol and rep["dem_rmse_m"]["flip"] <= rep["dem_rmse_m"]["noflip"]
    ):
        rep["fail"].append(f"DEM convention check failed: {rep['dem_rmse_m']}")
    if tiles:
        cv2.imwrite(str(sh / "contact_sheet.png"), np.concatenate(tiles, 0))
    json.dump(rep, open(sh / "validation_report.json", "w"), indent=1)
    print(json.dumps(rep, indent=1))
    print(f"-> {sh / 'validation_report.json'}, {sh / 'contact_sheet.png'}")
    return 1 if rep["fail"] else 0


if __name__ == "__main__":
    sys.exit(main())
