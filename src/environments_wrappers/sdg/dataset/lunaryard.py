"""Lunaryard controller specialised for mode=SDG dataset=<preset>.

What differs from src/environments_wrappers/sdg/lunaryard_sdg.py and why:

  * Stereo or mono rig under /Lunaryard/Rig; stereo cameras sit at +-baseline/2 on the
    rig Y axis (rig X forward, Z up -> Y is image-left), a mono camera at y=0.
  * Per-terrain reseeding. Every numpy Generator inside the terrain generator and
    the rock manager is replaced with default_rng(terrain_seed) before a terrain is
    built, so terrain k of shard S is a pure function of (S, k). Stock SDG advances
    one RNG stream, so terrain k depends on everything rendered before it.
  * Sun elevation from a low-angle-heavy mixture instead of uniform(20, 90).
  * Per-terrain ground truth dumped for the offline builder: the DEM, crater
    centres/sizes, and every rock instance with its world AABB. Craters and slopes
    are baked into the mesh and get no labels from the renderer; the DEM is how
    the builder recovers them exactly.
"""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import json
import os
import time

import numpy as np
from pxr import Usd, UsdGeom

from src.environments.lunaryard import LunaryardController
from src.environments_wrappers.sdg.dataset.rig import RigMixin


def _reseed_generators(obj, seed: int, _depth=0, _seen=None) -> int:
    """Replace every np.random.Generator reachable from obj (attrs, dicts, lists) with default_rng(seed+i)."""
    if _seen is None:
        _seen = set()
    if _depth > 5 or id(obj) in _seen:
        return 0
    _seen.add(id(obj))
    n = 0
    items = []
    if isinstance(obj, dict):
        items = list(obj.items())
    elif isinstance(obj, (list, tuple)):
        items = list(enumerate(obj))
    elif hasattr(obj, "__dict__"):
        items = list(vars(obj).items())
    for key, val in items:
        if isinstance(val, np.random.Generator):
            new = np.random.default_rng(seed + n)
            if isinstance(obj, dict):
                obj[key] = new
            elif isinstance(obj, list):
                obj[key] = new
            else:
                setattr(obj, key, new)
            n += 1
        elif isinstance(val, (dict, list, tuple)) or (hasattr(val, "__dict__") and not isinstance(val, type)):
            if val.__class__.__module__.startswith(("numpy", "pxr", "omni", "builtins", "scipy", "warp")):
                continue
            n += _reseed_generators(val, seed + 100 * (n + 1), _depth + 1, _seen)
    return n


class DatasetLunaryard(LunaryardController, RigMixin):
    def __init__(
        self,
        lunaryard_settings=None,
        rocks_settings=None,
        flares_settings=None,
        terrain_manager=None,
        camera_settings=None,
        dataset=None,
        camera_names=None,
        resolution=None,
        **kwargs,
    ):
        super().__init__(
            lunaryard_settings=lunaryard_settings,
            rocks_settings=rocks_settings,
            flares_settings=flares_settings,
            terrain_manager=terrain_manager,
            **kwargs,
        )
        self.terrain_settings = terrain_manager
        self.init_dataset(dataset, camera_names, resolution)
        self.grid = float(self.terrain_settings.resolution)

    # ------------------------------------------------------------------ scene
    def load(self) -> None:
        self.createRig()
        super().load()
        mesh = self.stage.GetPrimAtPath(self.T._mesh_path)
        rel = mesh.GetRelationship("material:binding") if mesh else None
        bound = [str(t) for t in rel.GetTargets()] if rel and rel.IsValid() else []
        print(f"[sdg_dataset] terrain mesh {self.T._mesh_path} material binding: {bound}", flush=True)
        if not any(self.terrain_material_match in b for b in bound):
            print(
                f"[sdg_dataset] WARNING: terrain is not bound to {self.terrain_material_match} (got {bound}); "
                "check terrain_manager.texture_path",
                flush=True,
            )
        if self.ds.terrain_material:
            self.apply_terrain_material()
        if self.ds.guards["hide_far_mesh"]:
            print("[sdg_dataset] WARNING: hide_far_mesh has no effect on Lunaryard", flush=True)

    # ------------------------------------------------------------------ DEM helpers
    def ground_height(self, x_m: float, y_m: float) -> float:
        """Terrain height at world (x, y). Mesh vertex (x*g, y*g) carries flip(DEM,0)[y, x]."""
        H, W = self.dem.shape
        col = int(np.clip(round(x_m / self.grid), 0, W - 1))
        row = int(np.clip(round(y_m / self.grid), 0, H - 1))
        return float(self.dem[H - 1 - row, col])

    dem_z = ground_height

    # ------------------------------------------------------------------ terrain
    def new_terrain(self, k: int) -> None:
        self.terrain_index = k
        self.terrain_seed = self.base_seed * 1000 + k
        self.rng = np.random.default_rng(self.terrain_seed)
        t0 = time.time()
        n_t = _reseed_generators(self.T._G, self.terrain_seed)
        n_r = _reseed_generators(self.RM, self.terrain_seed + 7)
        if n_t == 0:
            raise RuntimeError("reseed found no RNG inside the terrain generator; per-terrain seeding is broken")
        self.switch_terrain(-1)
        t_terrain = time.time() - t0
        self.terrain_meta = self._dump_terrain(k, n_t, n_r, t_terrain)
        print(
            f"[sdg_dataset] terrain {k:04d} seed={self.terrain_seed} rngs(terrain={n_t}, rocks={n_r}) "
            f"craters={len(self.terrain_meta['craters'])} rocks={len(self.terrain_meta['rocks'])} "
            f"in {t_terrain:.1f}s",
            flush=True,
        )

    def sample_rig(self) -> dict:
        L = float(self.terrain_settings.sim_length)
        Wd = float(self.terrain_settings.sim_width)
        m = float(self.ds.rig["terrain_margin_m"])
        x = float(self.rng.uniform(m, L - m))
        y = float(self.rng.uniform(m, Wd - m))
        return self.place_rig(x, y)

    def _rock_instances(self):
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            useExtentsHint=True,
            ignoreVisibility=True,
        )
        rocks = []
        for group, inst in self.RM.instancers.items():
            paths = getattr(inst, "instance_paths", None)
            if not paths:
                continue
            for i, p in enumerate(paths):
                prim = self.stage.GetPrimAtPath(p)
                if not prim.IsValid():
                    continue
                r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                mn, mx = np.array(r.GetMin()), np.array(r.GetMax())
                pos = np.asarray(inst.position[i], dtype=float)
                ground = self.dem_z(pos[0], pos[1])
                rocks.append(
                    {
                        "path": p,
                        "group": group,
                        "prototype": int(inst.ids[i]) if inst.ids is not None else -1,
                        "position": pos.round(4).tolist(),
                        "scale": np.asarray(inst.scale[i], dtype=float).round(4).tolist(),
                        "aabb_min": mn.round(4).tolist(),
                        "aabb_max": mx.round(4).tolist(),
                        "height_above_ground_m": round(float(mx[2] - ground), 4),
                        "footprint_m": round(float(max(mx[0] - mn[0], mx[1] - mn[1])), 4),
                    }
                )
        return rocks

    def _dump_terrain(self, k, n_t, n_r, t_terrain):
        craters = []
        for c in self.T._craters_data or []:
            craters.append(
                {
                    "coord_m": [float(c.coord[0]), float(c.coord[1])],
                    "size_px": int(c.size),
                    "xy_deformation_factor": [float(v) for v in c.xy_deformation_factor],
                    "rotation_deg": float(c.rotation),
                    "profile_id": int(c.crater_profile_id),
                }
            )
        rocks = self._rock_instances()
        meta = {
            "terrain_index": k,
            "terrain_seed": self.terrain_seed,
            "base_seed": self.base_seed,
            "dem_shape": list(self.dem.shape),
            "grid_m": self.grid,
            "dem_convention": "z(x,y) = dem[H-1-round(y/g), round(x/g)]  (mesh uses flip(dem,0))",
            "origin_xy_m": [0.0, 0.0],
            "crater_xy_frame": "lunaryard_index",
            "rng_reseeded": {"terrain": n_t, "rocks": n_r},
            "build_seconds": round(t_terrain, 2),
            "craters": craters,
            "rocks": rocks,
        }
        base = os.path.join(self.out_dir, "terrains", f"terrain_{k:04d}")
        np.savez_compressed(base + ".npz", dem=self.dem.astype(np.float32), mask=self.mask.astype(np.uint8))
        with open(base + ".json", "w") as f:
            json.dump(meta, f)
        return meta
