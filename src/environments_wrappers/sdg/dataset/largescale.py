"""Real Moon surface: OmniLRS LargeScale environment (NASA LOLA 5 mpp South Pole DEM, Site20)
with procedural high-resolution craters and rocks streamed around the rig.

Why this is not the stock SDG path: OmniLRS SDG mode only knows Lunalab/Lunaryard. The
LargeScale controller streams a 0.025 m high-res DEM window (hr_dem_num_blocks blocks of
block_size m on each side) around whatever position you hand to update_visual_mesh(). We
walk the rig across the map, ask for a terrain update at every new location (blocking
until the HR DEM, clipmaps, rocks and colliders are built), then shoot frames.

"Terrain seed" here = (shard base_seed -> crater/rock generator seeds, set in the env yaml
via environment.seed) + location index. The underlying LOLA DEM is fixed; the procedural
crater field and rocks are what the seed controls.
"""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import json
import os
import time

import numpy as np
from pxr import Usd, UsdGeom

from src.environments.large_scale_lunar import LargeScaleController
from src.environments_wrappers.sdg.dataset import sampling
from src.environments_wrappers.sdg.dataset.rig import RigMixin
from src.terrain_management.large_scale_terrain.utils import BoundingBox


class DatasetLargeScale(LargeScaleController, RigMixin):
    def __init__(
        self,
        large_scale_terrain=None,
        stellar_engine_settings=None,
        sun_settings=None,
        flares_settings=None,
        camera_settings=None,
        dataset=None,
        camera_names=None,
        resolution=None,
        is_simulation_alive=lambda: True,
        **kwargs,
    ):
        super().__init__(
            large_scale_terrain=large_scale_terrain,
            stellar_engine_settings=stellar_engine_settings,
            sun_settings=sun_settings,
            is_simulation_alive=is_simulation_alive,
            flares_settings=flares_settings,
            **kwargs,
        )
        self.init_dataset(dataset, camera_names, resolution)
        self.walk_xy = np.zeros(2)
        self.loc_meta = None

    # ------------------------------------------------------------------ scene
    def load(self) -> None:
        self.createRig()
        # Replicates LargeScaleController.load() so the rock height sampler can be replaced BEFORE the
        # first terrain update samples any rocks (LSTM.build() would otherwise sample with the clipmap
        # kernel, which put whole rock fields ~7 m above the mesh at many locations).
        from src.terrain_management.large_scale_terrain_manager import LargeScaleTerrainManager

        self.build_scene()
        self.LSTM = LargeScaleTerrainManager(self.stage_settings, is_simulation_alive=self.is_simulation_alive)
        self.LSTM.build_configs()
        self.LSTM.build_managers()
        self._patch_rock_heights()
        self.LSTM.mesh_position = (0, 0)
        self.LSTM.update_visual_mesh((0, 0))
        if self.enable_stellar_engine:
            self.SE.set_lat_lon(*self.LSTM.get_lat_lon())
        if self.SAM:
            self.SAM.spawn(get_height_func=self.LSTM.get_height_local)
        if self.MCM:
            self.MCM.spawn()
        self.hr_res = float(self.stage_settings.hr_dem_resolution)
        self._instrument_streaming()

    def _instrument_streaming(self) -> None:
        """Wrap the four streaming sub-steps with timers (a location took ~40 s with no breakdown).
        Printed once per location as [sdg_dataset] stream: ..."""
        import functools

        self._stream_t = {}

        def wrap(obj, name, label):
            fn = getattr(obj, name, None)
            if fn is None:
                return

            @functools.wraps(fn)
            def timed(*a, **k):
                t0 = time.time()
                try:
                    return fn(*a, **k)
                finally:
                    self._stream_t[label] = self._stream_t.get(label, 0.0) + time.time() - t0

            setattr(obj, name, timed)

        wrap(self.LSTM.map_manager.hr_dem_gen, "update_terrain_data_blocking", "hr_dem")
        # Workaround for the stale fine-clipmap DEM buffer after a block shift; remove once
        # LargeScaleTerrainManager.update_visual_mesh refreshes it.
        # update_visual_mesh never refreshes the fine clipmap's GPU DEM buffer after the HR DEM shifts (the call is
        # commented out upstream). Any move that shifts the block grid then renders stale terrain at the new
        # position, offset by metres from the DEM the rocks and rig are placed on (every location after the 3rd,
        # +-4-11 m). Refresh the buffer right after the blocking terrain update, before update_clipmaps runs.
        gen = self.LSTM.map_manager.hr_dem_gen
        fine = self.LSTM.nested_clipmap_manager.fine_clipmap_manager
        timed_update = gen.update_terrain_data_blocking

        def update_then_refresh(*a, **k):
            r = timed_update(*a, **k)
            t0 = time.time()
            sampler = getattr(fine._geo_clipmap, "DEM_sampler", None)
            if sampler is not None:
                # The generator may re-instantiate high_res_dem (new array) on a large jump; the clipmap
                # sampler would then keep sampling the OLD array forever. Always re-point it first.
                if sampler.dem is not gen.high_res_dem:
                    sampler.dem = gen.high_res_dem
                    self._stream_t["dem_repointed"] = self._stream_t.get("dem_repointed", 0.0) + 1
                if getattr(sampler, "acceleration_mode", "") == "hybrid":
                    import warp as wp

                    sampler.dem_wp = wp.array(sampler.dem, dtype=float, device="cpu", copy=False)
                else:
                    fine.update_DEM_buffer()  # gpu mode: re-upload the (current) DEM
            self._stream_t["dem_buffer"] = self._stream_t.get("dem_buffer", 0.0) + time.time() - t0
            return r

        gen.update_terrain_data_blocking = update_then_refresh
        wrap(self.LSTM.nested_clipmap_manager, "update_clipmaps", "clipmaps")
        wrap(self.LSTM.rock_manager, "sample", "rocks")
        wrap(self.LSTM.collider_manager, "update_shifting_map", "colliders")
        bound = []
        for p in self.stage.Traverse():
            rel = p.GetRelationship("material:binding")
            if rel and rel.IsValid():
                bound += [str(t) for t in rel.GetTargets() if self.terrain_material_match in str(t)]
        print(
            f"[sdg_dataset] prims bound to {self.terrain_material_match}: {len(bound)} "
            f"(texture={self.stage_settings.geo_cm_texture_name})",
            flush=True,
        )
        if not bound:
            print(
                f"[sdg_dataset] WARNING: no prim is bound to {self.terrain_material_match}; "
                "check large_scale_terrain.geo_cm_texture_name",
                flush=True,
            )
        if self.ds.terrain_material:
            self.apply_terrain_material()
        self._hide_far_mesh()

    def _hide_far_mesh(self) -> None:
        """Hide the COARSE (5 m LR DEM) clipmap. Its long triangles smear the 4 m regolith tiling into radial
        streaks and a flat bright sheet at the horizon, and giving it its own material failed (texture did not
        bind, rendered solid red). Near-field segmentation does not need it, so beyond the fine clipmap the frame
        shows space: those pixels are labelled space and depth saturates.
        Called after load and after every terrain move in case the clipmap update touches visibility."""
        if not self.ds.guards["hide_far_mesh"]:
            return
        coarse = self.LSTM.nested_clipmap_manager.coarse_clipmap_manager
        prim = self.stage.GetPrimAtPath(coarse._mesh_path)
        if not prim or not prim.IsValid():
            raise RuntimeError(f"coarse clipmap prim missing at {coarse._mesh_path}")
        img = UsdGeom.Imageable(prim)
        img.MakeInvisible()
        print(
            f"[sdg_dataset] far (coarse) mesh hidden: {coarse._mesh_path} visibility={img.ComputeVisibility()}",
            flush=True,
        )

    def _patch_rock_heights(self) -> None:
        """Rock z from hr_dem_gen.get_height (same query that places the rig, verified to 1 cm against the
        render at every location) instead of the clipmap kernel. Orientation: +Z aligned to the DEM normal
        with a random yaw, seeded. Falls back to the original sampler for points the HR DEM cannot answer."""
        from scipy.spatial.transform import Rotation as R

        gen = self.LSTM.map_manager.hr_dem_gen
        replaced = []

        def make(orig):
            def height_fn(x, y, map_coordinates, seed=0):
                x = np.asarray(x, dtype=float)
                y = np.asarray(y, dtype=float)
                n = len(x)
                z = np.full(n, np.nan)
                quats = np.zeros((n, 4))
                quats[:, 3] = 1.0
                rng = np.random.default_rng(int(seed) + 12345)
                for i in range(n):
                    try:
                        z[i] = float(gen.get_height((x[i], y[i])))
                        nrm = np.asarray(gen.get_normal((x[i], y[i])), dtype=float)
                    except Exception:  # noqa: BLE001
                        continue
                    nrm = nrm / max(np.linalg.norm(nrm), 1e-9)
                    axis = np.cross([0.0, 0.0, 1.0], nrm)
                    s_ = np.linalg.norm(axis)
                    c_ = float(np.clip(nrm[2], -1, 1))
                    r_align = R.from_rotvec(axis / s_ * np.arctan2(s_, c_)) if s_ > 1e-6 else R.identity()
                    quats[i] = (R.from_rotvec(nrm * rng.uniform(0, 2 * np.pi)) * r_align).as_quat()
                bad = np.isnan(z)
                if bad.any():
                    z2, q2 = orig(x[bad], y[bad], map_coordinates, seed)
                    z[bad] = z2
                    quats[bad] = q2
                    print(f"[sdg_dataset] rock height fallback for {int(bad.sum())}/{n} points", flush=True)
                return z, quats

            return height_fn

        for rg in self.LSTM.rock_manager.rock_generators:
            sampler = getattr(rg, "rock_sampler", None)
            for holder in [sampler] + list(vars(sampler).values()) if sampler is not None else []:
                if hasattr(holder, "sampling_func") and not isinstance(holder, (str, int, float, list, dict)):
                    holder.sampling_func = make(holder.sampling_func)
                    replaced.append(type(holder).__name__)
        print(f"[sdg_dataset] rock height sampler replaced in: {replaced}", flush=True)
        if not replaced:
            raise RuntimeError("could not find the rock sampling_func to patch; rocks would float")

    def ground_height(self, x: float, y: float) -> float:
        return float(self.LSTM.get_height_local((x, y)))

    # ------------------------------------------------------------------ locations
    def new_terrain(self, k: int) -> None:
        self.terrain_index = k
        self.terrain_seed = self.base_seed * 1000 + k
        self.rng = np.random.default_rng(self.terrain_seed)
        t0 = time.time()
        self.walk_xy = sampling.next_walk_location(
            self.rng,
            self.walk_xy,
            k,
            float(self.ds.largescale["region_radius_m"]),
            self.ds.largescale["step_m"],
        )
        xy = self.walk_xy.copy()
        gen = self.LSTM.map_manager.hr_dem_gen
        self.LSTM.update_visual_mesh((float(xy[0]), float(xy[1])))
        for _ in range(600):  # update_terrain_data_blocking should already block; belt and braces
            if gen.is_map_done():
                break
            time.sleep(0.1)
        # Re-sample the clipmap AFTER the DEM is certainly complete. The stock path samples the mesh
        # elevation immediately after the blocking update; the CPU DEM used by rig/rocks/crops was
        # correct at every location (matches LOLA within 0.5 m) while the rendered mesh was metres off
        # from the first block shift on, so the mesh must have been sampled from a not-yet-final array.
        t1 = time.time()
        time.sleep(1.0)
        self.loc_xy = xy
        self.resample_clipmap()
        self._hide_far_mesh()
        self._stream_t["clipmaps_resample"] = self._stream_t.get("clipmaps_resample", 0.0) + time.time() - t1
        t_terrain = time.time() - t0
        self.loc_xy = xy
        self.terrain_meta = self._dump_location(k, t_terrain)
        print(
            f"[sdg_dataset] location {k:04d} seed={self.terrain_seed} local_xy=({xy[0]:.1f},{xy[1]:.1f}) "
            f"map_done={gen.is_map_done()} craters={len(self.terrain_meta['craters'])} "
            f"rocks={len(self.terrain_meta['rocks'])} in {t_terrain:.1f}s",
            flush=True,
        )
        st = getattr(self, "_stream_t", {})
        print(
            "[sdg_dataset] stream: "
            + " ".join(f"{k2}={v:.1f}s" for k2, v in st.items())
            + f" dump={time.time() - t0 - t_terrain:.1f}s",
            flush=True,
        )
        self._stream_t = {}

    def resample_clipmap(self) -> None:
        """Re-sample the clipmap elevation at the current location with the stock-equivalent arguments."""
        xy = self.loc_xy
        n_m = float(self.stage_settings.update_every_n_meters)
        corrected = ((xy[0] // n_m) * n_m, (xy[1] // n_m) * n_m)
        sp = self.stage_settings.starting_position
        gc = (corrected[0] + sp[0], corrected[1] + sp[1])
        mm = self.LSTM.map_manager
        self.LSTM.nested_clipmap_manager.update_clipmaps(
            mm.get_hr_coordinates(gc), mm.get_lr_coordinates(gc), corrected
        )

    def sample_rig(self) -> dict:
        """Jitter around the location, rejecting spots inside or next to a labelled rock: without the check
        the rig ended up inside a boulder about once in six frames (black frame, all-rock label)."""
        j = float(self.ds.largescale["frame_jitter_m"])
        clear = float(self.ds.largescale["rock_clearance_m"])
        rocks = self.terrain_meta["rocks"] if self.terrain_meta else []
        rx = np.array([r["position"][0] for r in rocks]) if rocks else np.zeros(0)
        ry = np.array([r["position"][1] for r in rocks]) if rocks else np.zeros(0)
        rr = np.array([r["footprint_m"] / 2 + clear for r in rocks]) if rocks else np.zeros(0)
        for attempt in range(40):
            x = float(self.loc_xy[0] + self.rng.uniform(-j, j))
            y = float(self.loc_xy[1] + self.rng.uniform(-j, j))
            if rr.size == 0 or not np.any(np.hypot(rx - x, ry - y) < rr):
                break
        else:
            print(
                f"[sdg_dataset] WARNING: no rock-free rig spot after 40 tries at location {self.terrain_index}",
                flush=True,
            )
        rec = self.place_rig(x, y)
        rec["placement_attempts"] = attempt + 1
        return rec

    # ------------------------------------------------------------------ ground truth dumps
    def _rock_instances(self, center, radius_m):
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            useExtentsHint=True,
            ignoreVisibility=True,
        )
        rocks = []
        for gen in self.LSTM.rock_manager.rock_generators:
            inst = gen.rock_instancer
            paths = getattr(inst, "instance_paths", None)
            if not paths:  # PointInstancer group (unlabelled): no per-rock prims
                continue
            pos = np.asarray(inst.position, dtype=float)
            near = np.flatnonzero(np.hypot(pos[:, 0] - center[0], pos[:, 1] - center[1]) <= radius_m)
            for i in near:
                prim = self.stage.GetPrimAtPath(paths[i])
                if not prim.IsValid():
                    continue
                r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                mn, mx = np.array(r.GetMin()), np.array(r.GetMax())
                ground = self.ground_height(pos[i, 0], pos[i, 1])
                rocks.append(
                    {
                        "path": paths[i],
                        "group": gen.settings.instancer_name,
                        "prototype": int(inst.ids[i]) if inst.ids is not None else -1,
                        "position": pos[i].round(4).tolist(),
                        "scale": np.asarray(inst.scale[i], dtype=float).round(4).tolist(),
                        "aabb_min": mn.round(4).tolist(),
                        "aabb_max": mx.round(4).tolist(),
                        "height_above_ground_m": round(float(mx[2] - ground), 4),
                        "footprint_m": round(float(max(mx[0] - mn[0], mx[1] - mn[1])), 4),
                    }
                )
        return rocks

    def _craters(self, center_global, half):
        db = getattr(self.LSTM.map_manager.hr_dem_gen, "crater_db", None)
        out = []
        if db is None:
            return out, "none"
        try:
            bs = int(self.stage_settings.block_size)  # the DBs index by block; bounds must be int block multiples
            region = BoundingBox(
                x_min=int(np.floor((center_global[0] - half) / bs) * bs),
                x_max=int(np.ceil((center_global[0] + half) / bs) * bs),
                y_min=int(np.floor((center_global[1] - half) / bs) * bs),
                y_max=int(np.ceil((center_global[1] + half) / bs) * bs),
            )
            res = db.get_blocks_within_region(region)
            metas = res[0] if isinstance(res, tuple) else res
            sp = self.stage_settings.starting_position
            for c in metas:
                if abs(float(c.coordinates[0]) - center_global[0]) > half + float(c.radius) or abs(
                    float(c.coordinates[1]) - center_global[1]
                ) > half + float(c.radius):
                    continue  # keep only craters that can touch the saved DEM crop
                out.append(
                    {
                        "xy_local_m": [float(c.coordinates[0]) - sp[0], float(c.coordinates[1]) - sp[1]],
                        "xy_global_m": [float(c.coordinates[0]), float(c.coordinates[1])],
                        "radius_m": float(c.radius),
                        "xy_deformation_factor": [float(v) for v in c.xy_deformation_factor],
                        "rotation_deg": float(c.rotation),
                    }
                )
            return out, "crater_db(global_m, assumed)"
        except Exception as e:  # noqa: BLE001 - bookkeeping must never kill a long render run
            print(f"[sdg_dataset] crater db query failed: {type(e).__name__}: {e}", flush=True)
            return out, f"failed:{type(e).__name__}"

    def _dem_crop(self, center_local, half_m, out_res):
        """HR DEM crop around the location, re-laid in the builder convention:
        z(x, y) = dem[H-1-round((y-oy)/g), round((x-ox)/g)] with origin (ox, oy) in LOCAL metres."""
        gen = self.LSTM.map_manager.hr_dem_gen
        sp = self.stage_settings.starting_position
        g = self.hr_res
        step = max(1, int(round(out_res / g)))
        gx, gy = center_local[0] + sp[0], center_local[1] + sp[1]
        hx, hy = gen.get_coordinates((gx - half_m, gy - half_m))  # HR frame metres of the crop corner
        ix0, iy0 = int(hx / g), int(hy / g)
        n = int(2 * half_m / g)
        hr = gen.high_res_dem
        ix0, iy0 = max(ix0, 0), max(iy0, 0)
        crop = np.asarray(hr[ix0 : ix0 + n : step, iy0 : iy0 + n : step], dtype=np.float32)  # [x, y]
        dem_std = np.flip(crop.T, 0)  # rows = y flipped, cols = x
        origin = [float(center_local[0] - half_m + (ix0 * g - hx)), float(center_local[1] - half_m + (iy0 * g - hy))]
        return dem_std, origin, g * step, (int(hr.shape[0]), int(hr.shape[1]))

    def _dump_location(self, k, t_terrain):
        c = self.loc_xy
        sp = self.stage_settings.starting_position
        half = float(self.ds.largescale["dem_crop_half_m"])
        dem_std, origin, g, hr_shape = self._dem_crop(c, half, float(self.ds.largescale["dem_crop_res_m"]))
        craters, crater_src = self._craters((c[0] + sp[0], c[1] + sp[1]), half)
        rocks = self._rock_instances(c, half)
        meta = {
            "terrain_index": k,
            "terrain_seed": self.terrain_seed,
            "base_seed": self.base_seed,
            "environment": "LargeScale",
            "lr_dem": self.stage_settings.lr_dem_name,
            "starting_position_global_m": [float(sp[0]), float(sp[1])],
            "location_local_m": [float(c[0]), float(c[1])],
            "location_global_m": [float(c[0] + sp[0]), float(c[1] + sp[1])],
            "dem_shape": list(dem_std.shape),
            "grid_m": g,
            "origin_xy_m": origin,
            "hr_dem_shape": hr_shape,
            "dem_convention": "z(x,y) = dem[H-1-round((y-oy)/g), round((x-ox)/g)], (ox,oy)=origin_xy_m, local frame",
            "crater_source": crater_src,
            "crater_xy_frame": "local_xy",
            "build_seconds": round(t_terrain, 2),
            "craters": craters,
            "rocks": rocks,
        }
        base = os.path.join(self.out_dir, "terrains", f"terrain_{k:04d}")
        np.savez_compressed(
            base + ".npz", dem=dem_std.astype(np.float32), origin_xy=np.array(origin, np.float32), grid=np.float32(g)
        )
        with open(base + ".json", "w") as f:
            json.dump(meta, f)
        return meta

    def shutdown(self) -> None:
        try:
            self.LSTM.map_manager.hr_dem_gen.shutdown()
            print("[sdg_dataset] hr dem workers shut down", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[sdg_dataset] shutdown warning: {e}", flush=True)
