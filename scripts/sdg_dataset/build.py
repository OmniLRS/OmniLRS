#!/usr/bin/env python3
"""Turn raw mode=SDG_Dataset shards into the training dataset (S / M / L, split by terrain seed).

    python scripts/sdg_dataset/build.py --shards data/sdg_dataset/shard_00000 [...] --out data/sdg_dataset_built \
        [--wheel-clearance-m 0.05] [--slope-caution-deg 15] [--slope-hazard-deg 25] [--footprint-m 0.30] \
        [--sizes S=2000,M=6000,L=10000] [--val-frac 0.15] [--test-frac 0] [--exclude rejected.txt ...]
    python scripts/sdg_dataset/build.py --out data/sdg_dataset_built --splits-only --exclude */rejected.txt

Per frame it writes, under <out>/<shard name>/:
  images/<id>_{L,R}.png          RGB
  labels/<id>_{L,R}_sem.png      uint8: 0 space, 1 ground, 2 rock_small, 3 rock_large
  labels/<id>_{L,R}_trav.png     uint8: 0 free, 1 caution, 2 hazard   (policy in traversability())
  masks/<id>_{L,R}_bits.png      uint8 bitfield: 1 slope_caution 2 slope_hazard 4 crater 8 rock_small 16 rock_large
  depth/<id>_{L,R}_mm.png        uint16 millimetres, 0 = no surface (space)
  normals/<id>_{L,R}.png         uint8 (n+1)/2*255, frame as recorded
  meta/<id>.json                 sun, rig pose, intrinsics, terrain seed, guard outcomes, rock table, measured rocks
  terrains/                      copy of the shard's per-terrain DEM + tables
and <out>/splits/<size>_<split>.txt (frame ids; sizes are nested S c M c L and split by terrain seed),
<out>/index.json, <out>/summary.json.

Rock size: each rock blob's height above the local ground is measured from depth + camera pose (contact band
under the blob within 6 m, projected extent beyond) and thresholded at --wheel-clearance-m; blobs that cannot be
measured keep the class from the generator's rock table. Craters and slopes come from the DEM, not from pixels:
each depth pixel is unprojected to world xy and looked up in the terrain's DEM-derived masks.
"""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import argparse
import dataclasses
import hashlib
import json
import os
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.sdg_dataset import _common as C  # noqa: E402


# ----------------------------------------------------------------------------- policy
@dataclasses.dataclass
class BuildOptions:
    wheel_clearance_m: float = 0.05
    slope_caution_deg: float = 15.0
    slope_hazard_deg: float = 25.0
    footprint_m: float = 0.30
    far_terrain_m: float = 150.0


def rock_class(height_m: float, clearance_m: float) -> str:
    """Small vs large rock: a rock the wheel can roll over is small. Height above the local ground is what counts."""
    return "rock_large" if height_m >= clearance_m else "rock_small"


def guard_field(fr: dict, key: str, default=None):
    """A generator guard outcome: under fr["guards"] (SDG_Dataset manager), else top-level (older raw shards)."""
    g = fr.get("guards") or {}
    return g[key] if key in g else fr.get(key, default)


def traversability(sem: np.ndarray, bits: np.ndarray) -> np.ndarray:
    """Collapse semantics + geometry into free / caution / hazard.

    Rule:
      hazard  = rock_large | slope_hazard | space
      caution = rock_small | slope_caution | crater
      free    = everything else
    """
    trav = np.zeros(sem.shape, np.uint8)
    caution = (sem == C.SEM["rock_small"]) | (bits & C.BIT["slope_caution"] > 0) | (bits & C.BIT["crater"] > 0)
    hazard = (sem == C.SEM["rock_large"]) | (bits & C.BIT["slope_hazard"] > 0) | (sem == C.SEM["space"])
    trav[caution] = 1
    trav[hazard] = 2
    return trav


# ----------------------------------------------------------------------------- per frame
def process_frame(
    shard: Path, man: dict, fr: dict, terr: C.Terrain, opts: BuildOptions, out: Path, fid: str, epf: int
) -> dict:
    data_dir = C.resolve_data_dir(shard, man)
    idx = fr["index"]
    meta = {
        "id": fid,
        **{k: fr[k] for k in ("terrain_index", "terrain_seed", "frame_in_terrain", "render", "sun", "rig")},
        "base_seed": man["base_seed"],
        "lit_fraction": guard_field(fr, "lit_fraction"),
        "dark_retries": guard_field(fr, "dark_retries", 0),
        "guards": dict(fr.get("guards") or {}),  # every guard outcome the generator recorded for this frame
        "cams": {},
    }
    for cam, side in C.cameras(man):
        K = np.array(man["intrinsics"][cam]["K"], dtype=np.float64)
        rgb_path = C.frame_file(data_dir, cam, "rgb", idx, epf, "png")
        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        if rgb is None:
            raise FileNotFoundError(rgb_path)
        depth = np.load(str(C.frame_file(data_dir, cam, "depth", idx, epf, "npz")))["depth"].astype(np.float32)
        sem_key, sem_tab = C.read_colorized(
            C.frame_file(data_dir, cam, "semantic_segmentation", idx, epf, "png"),
            C.label_file(data_dir, cam, "semantic_segmentation", idx, epf),
        )
        ins_key, ins_tab = C.read_colorized(
            C.frame_file(data_dir, cam, "instance_segmentation", idx, epf, "png"),
            C.label_file(data_dir, cam, "instance_segmentation", idx, epf),
        )
        H, W = depth.shape
        if rgb.shape[:2] != (H, W):
            rgb = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_AREA)
        valid = np.isfinite(depth) & (depth > 0) & (depth < 1e4)

        sem = np.zeros((H, W), np.uint8)
        for key, lab in sem_tab.items():
            cls = (lab.get("class") if isinstance(lab, dict) else str(lab)) or ""
            if "ground" in cls or "terrain" in cls:  # Lunaryard labels the mesh "ground", LargeScale "terrain"
                sem[sem_key == key] = C.SEM["ground"]
            elif "rock" in cls:
                sem[sem_key == key] = C.SEM["rock_small"]  # default; instance join below upgrades to rock_large
        # rocks: instance id -> prim path -> generator's rock table (fallback class for unmeasurable blobs)
        rocks_seen = {}
        for key, path in ins_tab.items():
            p = path if isinstance(path, str) else str(path)
            # the label is the rock prim itself or a child mesh; a bare prefix would join instance_10 to instance_1
            base = next((rp for rp in terr.rocks if p == rp or p.startswith(rp + "/")), None)
            if base is None:
                continue
            m = ins_key == key
            if not m.any():
                continue
            cls = rock_class(terr.rocks[base]["height_above_ground_m"], opts.wheel_clearance_m)
            sem[m] = C.SEM[cls]
            rocks_seen[base] = {
                "class": cls,
                "px": int(m.sum()),
                **{k: terr.rocks[base][k] for k in ("height_above_ground_m", "footprint_m")},
            }
        # geometry the renderer left unlabelled (the pebble PointInstancer layer, Earth) must not share the
        # sky's class 0: pebbles are terrain for every practical purpose
        unlab = valid & (sem == 0)  # kept as mask bit 32 so the quality checker can still find pebbles
        sem[unlab] = C.SEM["ground"]
        sem[~valid] = C.SEM["space"]

        # geometry masks via DEM lookup
        T = C.cam_pose(fr["rig"], man["intrinsics"][cam]["rig_offset_y_m"])
        pw = C.unproject(np.where(valid, depth, 1.0), K, T)
        row, col = terr.lookup(pw[..., 0], pw[..., 1])
        slope = terr.slope_deg[row, col]
        crater = terr.crater[row, col] > 0
        bits = np.zeros((H, W), np.uint8)
        gnd = valid & (sem == C.SEM["ground"])
        bits[gnd & (slope >= opts.slope_caution_deg)] |= C.BIT["slope_caution"]
        bits[gnd & (slope >= opts.slope_hazard_deg)] |= C.BIT["slope_hazard"]
        bits[gnd & crater] |= C.BIT["crater"]
        bits[sem == C.SEM["rock_small"]] |= C.BIT["rock_small"]
        bits[sem == C.SEM["rock_large"]] |= C.BIT["rock_large"]
        bits[unlab] |= C.BIT["unlabelled"]

        depth_mm = np.where(valid, np.clip(depth * 1000.0, 0, 65535), 0).astype(np.uint16)
        # rock size from the rendered geometry, measured on the mm-quantized depth
        dep_q = depth_mm.astype(np.float32) / 1000.0
        lab, blobs = C.measure_rock_blobs(sem, dep_q, K, T)
        measured = []
        for i, area, d, height, method in blobs:
            m = lab == i
            if height is None:
                # unmeasurable (far / no depth): the whole blob takes its majority class from the rock table
                sem[m] = int(np.bincount(sem[m], minlength=4)[2:].argmax()) + 2
            else:
                sem[m] = C.SEM[rock_class(height, opts.wheel_clearance_m)]
            cls_now = "rock_large" if int(np.bincount(sem[m], minlength=4)[2:].argmax()) + 2 == 3 else "rock_small"
            measured.append(
                {
                    "px": int(area),
                    "depth_m": round(d, 2),
                    "method": method,
                    "class": cls_now,
                    "height_m": None if height is None else round(height, 3),
                }
            )
        bits &= ~np.uint8(C.BIT["rock_small"] | C.BIT["rock_large"])
        bits[sem == C.SEM["rock_small"]] |= C.BIT["rock_small"]
        bits[sem == C.SEM["rock_large"]] |= C.BIT["rock_large"]
        trav = traversability(sem, bits)

        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        surf = valid & (sem != C.SEM["space"])
        dark_frac = float((gray[surf] < 8).mean()) if surf.any() else 1.0
        # terrain beyond the streamed high-res window (~175 m radius) is the coarse 5 m mesh and can show
        # striping/streaking under grazing light; recorded so such frames can be filtered or down-weighted
        far_frac = float((depth[surf] > opts.far_terrain_m).mean()) if surf.any() else 0.0
        nrm_path = C.frame_file(data_dir, cam, "normals", idx, epf, "npz")
        if nrm_path.exists():
            n = np.load(str(nrm_path))["normals"].astype(np.float32)
            n8 = np.clip((n + 1.0) * 127.5, 0, 255).astype(np.uint8)
            cv2.imwrite(str(out / "normals" / f"{fid}_{side}.png"), cv2.cvtColor(n8, cv2.COLOR_RGB2BGR))

        cv2.imwrite(str(out / "images" / f"{fid}_{side}.png"), rgb)
        cv2.imwrite(str(out / "labels" / f"{fid}_{side}_sem.png"), sem)
        cv2.imwrite(str(out / "labels" / f"{fid}_{side}_trav.png"), trav)
        cv2.imwrite(str(out / "masks" / f"{fid}_{side}_bits.png"), bits)
        cv2.imwrite(str(out / "depth" / f"{fid}_{side}_mm.png"), depth_mm)
        meta["cams"][side] = {
            "K": K.tolist(),
            "world_T_cam": T.tolist(),
            "width": W,
            "height": H,
            "class_px": {k: int((sem == v).sum()) for k, v in C.SEM.items()},
            "bits_px": {k: int((bits & v > 0).sum()) for k, v in C.BIT.items()},
            "rocks": rocks_seen,
            "rocks_measured": measured,
            "dark_frac": round(dark_frac, 4),
            "far_frac": round(far_frac, 4),
        }
    meta["dark_frac"] = max(c["dark_frac"] for c in meta["cams"].values())
    meta["far_frac"] = max(c["far_frac"] for c in meta["cams"].values())
    meta["baseline_m"] = man["intrinsics"][C.cameras(man)[0][0]].get("baseline_m", 0.0)
    json.dump(meta, open(out / "meta" / f"{fid}.json", "w"))
    return meta


_TERRAIN_CACHE = {}


def _work(args):
    """Worker: build one frame. Terrain objects are cached per process (shard, terrain index)."""
    sh, man, fr, opts, out, fid, epf = args
    key = (str(sh), fr["terrain_index"], opts.footprint_m)
    if key not in _TERRAIN_CACHE:
        _TERRAIN_CACHE[key] = C.Terrain(Path(sh), fr["terrain_index"], opts.footprint_m)
    try:
        meta = process_frame(Path(sh), man, fr, _TERRAIN_CACHE[key], opts, Path(out), fid, epf)
    except FileNotFoundError as e:
        return (fid, None, str(e))
    return (fid, meta, None)


def split_of(terrain_seed: int, val_frac: float, test_frac: float) -> str:
    """Split by terrain seed, so no terrain is in two splits (the published reference dataset used val_frac 0.15)."""
    h = int(hashlib.md5(str(terrain_seed).encode()).hexdigest(), 16) % 1000 % 100
    if h < val_frac * 100:
        return "val"
    if h < (val_frac + test_frac) * 100:
        return "test"
    return "train"


# ----------------------------------------------------------------------------- splits
def write_splits(out: Path, a, sizes: dict, excluded: set, missing: list) -> dict:
    index = json.loads((out / "index.json").read_text()) if (out / "index.json").exists() else {}
    index = {f: v for f, v in index.items() if f not in excluded}
    all_frames = []  # (order_key, fid, split, entry)
    for fid, e in index.items():
        split = split_of(e["terrain_seed"], a.val_frac, a.test_frac)
        if e["dark"]:
            split = "dark"
        elif e["far"]:
            split = "far"
        all_frames.append(((e["base_seed"], e["terrain_index"], e["frame_in_terrain"]), fid, split, e))
    all_frames.sort(key=lambda t: t[0])
    fpt = max(1, max(t[0][2] for t in all_frames) + 1) if all_frames else 25
    terrain_order = []
    for t in all_frames:
        key = t[0][:2]
        if key not in terrain_order:
            terrain_order.append(key)
    (out / "splits").mkdir(parents=True, exist_ok=True)
    for old in (out / "splits").glob("*.txt"):
        old.unlink()
    summary = {"frames": len(all_frames), "terrains": len(terrain_order), "sizes": {}}
    for name, target in sizes.items():
        n_terr = int(np.ceil(target / fpt))
        chosen = set(terrain_order[:n_terr])
        per_split = {}
        for key, fid, split, _ in all_frames:
            if key[:2] in chosen:
                per_split.setdefault(split, []).append(fid)
        for split, ids in per_split.items():
            (out / "splits" / f"{name}_{split}.txt").write_text("\n".join(ids) + "\n")
        summary["sizes"][name] = {"terrains": len(chosen), **{s: len(v) for s, v in per_split.items()}}
    # class balance over everything, for the record
    px = {}
    for _, fid, _, e in all_frames:
        mp = out / e["shard"] / "meta" / f"{fid}.json"
        if not mp.exists():
            continue
        for c in json.loads(mp.read_text())["cams"].values():
            for k, v in c["class_px"].items():
                px[k] = px.get(k, 0) + v
    tot = sum(px.values()) or 1
    summary["class_pixel_fraction"] = {k: round(v / tot, 4) for k, v in px.items()}
    summary["missing"] = sorted(missing)
    summary["excluded"] = len(excluded)
    return summary


def main(argv=None) -> int:
    d = BuildOptions()
    ap = argparse.ArgumentParser()
    ap.add_argument("--shards", nargs="*", default=[])
    ap.add_argument("--out", required=True)
    ap.add_argument("--wheel-clearance-m", type=float, default=d.wheel_clearance_m)
    ap.add_argument("--slope-caution-deg", type=float, default=d.slope_caution_deg)
    ap.add_argument("--slope-hazard-deg", type=float, default=d.slope_hazard_deg)
    ap.add_argument("--footprint-m", type=float, default=d.footprint_m)
    ap.add_argument("--sizes", default="S=2000,M=6000,L=10000")
    ap.add_argument("--val-frac", type=float, default=0.15)
    ap.add_argument("--test-frac", type=float, default=0.0)
    ap.add_argument("--exclude", nargs="*", default=[], help="files of 'frame id<TAB>reason' lines; first column used")
    ap.add_argument("--splits-only", action="store_true", help="rebuild splits/summary from out/index.json only")
    ap.add_argument("--limit", type=int, default=0, help="debug: stop after N frames")
    ap.add_argument(
        "--max-dark", type=float, default=0.9, help="frames with dark_frac above this go to the 'dark' list"
    )
    ap.add_argument(
        "--min-lit", type=float, default=0.7, help="frames whose generator lit_fraction is below this go to 'dark'"
    )
    ap.add_argument(
        "--max-far", type=float, default=1.0, help="frames whose far-terrain fraction exceeds this go to 'far'"
    )
    ap.add_argument("--workers", type=int, default=0, help="parallel frame builders (0 = min(16, cpu count))")
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    opts = BuildOptions(a.wheel_clearance_m, a.slope_caution_deg, a.slope_hazard_deg, a.footprint_m, d.far_terrain_m)
    sizes = {k: int(v) for k, v in (s.split("=") for s in a.sizes.split(","))}
    excluded = set()
    for p in a.exclude:
        for line in Path(p).read_text().splitlines():
            if line.strip():
                excluded.add(line.split("\t")[0].strip())

    missing = []
    n_tasks = n_done = 0
    if a.splits_only:
        prev = out / "summary.json"
        if prev.exists():
            missing = json.loads(prev.read_text()).get("missing", [])
    else:
        tasks, info = [], {}
        for sh in sorted(Path(s) for s in a.shards):
            man = C.load_manifest(sh)
            for sub in ("images", "labels", "masks", "depth", "normals", "meta"):
                (out / sh.name / sub).mkdir(parents=True, exist_ok=True)
            shutil.copytree(sh / "terrains", out / sh.name / "terrains", dirs_exist_ok=True)
            epf = int(man.get("element_per_folder", 1000))  # the writer's folder size; older shards used 1000
            for fr in man["frames"]:
                k = fr["terrain_index"]
                fid = f"s{man['base_seed']:05d}_t{k:04d}_f{fr['frame_in_terrain']:03d}"
                tasks.append((str(sh), man, fr, opts, str(out / sh.name), fid, epf))
                info[fid] = (sh.name, man["base_seed"], fr)
                if a.limit and len(tasks) >= a.limit:
                    break
            if a.limit and len(tasks) >= a.limit:
                break
        n_tasks = len(tasks)
        workers = a.workers or max(1, min(16, os.cpu_count() or 1))
        print(f"building {len(tasks)} frames with {workers} workers")
        index = json.loads((out / "index.json").read_text()) if (out / "index.json").exists() else {}
        if workers == 1:
            results = map(_work, tasks)
            pool = None
        else:
            from concurrent.futures import ProcessPoolExecutor

            pool = ProcessPoolExecutor(max_workers=workers)
            results = pool.map(_work, tasks, chunksize=4)
        try:
            for fid, meta, err in results:
                if meta is None:
                    print(f"  skip {fid}: missing {err}", file=sys.stderr)
                    missing.append(fid)
                    continue
                shard_name, base_seed, fr = info[fid]
                lit = guard_field(fr, "lit_fraction")  # written by the generator's dark-frame guard
                index[fid] = {
                    "shard": shard_name,
                    "terrain_seed": fr["terrain_seed"],
                    "base_seed": base_seed,
                    "terrain_index": fr["terrain_index"],
                    "frame_in_terrain": fr["frame_in_terrain"],
                    "dark": bool(meta["dark_frac"] > a.max_dark or (lit is not None and lit < a.min_lit)),
                    "far": bool(meta["far_frac"] > a.max_far),
                }
                n_done += 1
                if n_done % 200 == 0:
                    print(f"  {n_done} frames", flush=True)
        finally:
            if pool is not None:
                pool.shutdown()
        (out / "index.json").write_text(json.dumps(index))

    summary = write_splits(out, a, sizes, excluded, missing)
    json.dump(summary, open(out / "summary.json", "w"), indent=1)
    print(json.dumps(summary, indent=1))
    if n_tasks and not n_done:
        print(f"ERROR: built 0 of {n_tasks} manifest frames (every frame's files missing)", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
