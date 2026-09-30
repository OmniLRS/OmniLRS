"""Shared rig, sun and manifest code for the SDG_Dataset environments. The host class provides self.stage,
self.scene_name, StellarEngineEnvMixin's set_sun_* and ground_height(x, y)."""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import json
import os

from pxr import Gf, UsdGeom
from WorldBuilders.pxr_utils import addDefaultOps, setDefaultOps

from src.environments_wrappers.sdg.dataset import sampling


class RigMixin:
    def init_dataset(self, ds, camera_names: list, resolution: tuple) -> None:
        self.ds = ds
        self.camera_names = list(camera_names)
        self.resolution = tuple(resolution)
        self.base_seed = int(ds.base_seed)
        self.terrain_material_match = "LunarRegolith8k"
        self.terrain_index = -1
        self.terrain_seed = None
        self.rng = None
        self.frame_records = []
        self.terrain_meta = None
        self.intrinsics = {}
        self.out_dir = os.path.join(ds.out_dir, f"shard_{self.base_seed:05d}")
        os.makedirs(os.path.join(self.out_dir, "terrains"), exist_ok=True)

    # ------------------------------------------------------------------ rig
    def createRig(self) -> None:
        k = sampling.compute_intrinsics(*self.resolution, self.ds.rig)
        usd = k["usd"]
        W, H = k["width"], k["height"]

        rig = self.stage.DefinePrim(self.scene_name + "/Rig", "Xform")
        addDefaultOps(UsdGeom.Xformable(rig))
        setDefaultOps(UsdGeom.Xformable(rig), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (1.0, 1.0, 1.0))
        self._rig_prim = rig
        b = float(self.ds.rig["baseline_m"])
        self.cam_paths = {}
        for name, y in sampling.camera_offsets(self.camera_names, self.ds.rig["baseline_m"]).items():
            path = f"{self.scene_name}/Rig/{name}"
            cam = UsdGeom.Camera.Define(self.stage, path)
            cam.CreateFocalLengthAttr().Set(usd["focal_length_mm"])
            cam.CreateHorizontalApertureAttr().Set(usd["horizontal_aperture_mm"])
            cam.CreateVerticalApertureAttr().Set(usd["vertical_aperture_mm"])
            cam.CreateHorizontalApertureOffsetAttr().Set(usd["horizontal_aperture_offset_mm"])
            cam.CreateVerticalApertureOffsetAttr().Set(usd["vertical_aperture_offset_mm"])
            cam.CreateFStopAttr().Set(0.0)
            cam.CreateFocusDistanceAttr().Set(10.0)
            cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.05, 1.0e6))
            addDefaultOps(UsdGeom.Xformable(cam.GetPrim()))
            setDefaultOps(
                UsdGeom.Xformable(cam.GetPrim()), (0.0, y, 0.0), sampling.CAM_LOOK_FORWARD_XYZW, (1.0, 1.0, 1.0)
            )
            self.cam_paths[name] = path
            self.intrinsics[name] = {
                **{kk: k[kk] for kk in ("K", "width", "height", "hfov_deg", "usd")},
                "baseline_m": b,
                "rig_offset_y_m": y,
            }
        print(
            f"[sdg_dataset] rig: {W}x{H} hfov={k['hfov_deg']:.2f} fx={k['K'][0][0]:.1f} fy={k['K'][1][1]:.1f} "
            f"baseline={b} m",
            flush=True,
        )

    def place_rig(self, x: float, y: float) -> dict:
        rec = sampling.sample_rig_pose(self.rng, self.ds.rig, x, y, self.ground_height(x, y))
        setDefaultOps(
            UsdGeom.Xformable(self._rig_prim), tuple(rec["position"]), tuple(rec["quat_xyzw"]), (1.0, 1.0, 1.0)
        )
        return rec

    # ------------------------------------------------------------------ terrain material
    def apply_terrain_material(self) -> int:
        """The MDL tiles the 8k regolith once per ~1 m of clipmap UV (texture_scale 0.5 on UVs that are 2x metres),
        so at rover distances every pixel averages dozens of texels and the map mips to flat grey.
        terrain_material.texture_scale re-tiles it (0.125 -> one tile per 8 m, ~1 mm/texel); bump_factor is exposed
        too. Applies to every Shader prim whose path contains self.terrain_material_match. Returns the count."""
        from pxr import Sdf

        tm = self.ds.terrain_material or {}
        scale = tm.get("texture_scale")
        bump = tm.get("bump_factor")
        n = 0
        for p in self.stage.Traverse():
            if self.terrain_material_match not in str(p.GetPath()) or p.GetTypeName() != "Shader":
                continue
            if scale is not None:
                a = p.GetAttribute("inputs:texture_scale") or p.CreateAttribute(
                    "inputs:texture_scale", Sdf.ValueTypeNames.Float2
                )
                a.Set((float(scale), float(scale)))
            if bump is not None:
                a = p.GetAttribute("inputs:bump_factor") or p.CreateAttribute(
                    "inputs:bump_factor", Sdf.ValueTypeNames.Float
                )
                a.Set(float(bump))
            got = p.GetAttribute("inputs:texture_scale").Get() if p.GetAttribute("inputs:texture_scale") else None
            got_bump = p.GetAttribute("inputs:bump_factor").Get() if p.GetAttribute("inputs:bump_factor") else None
            print(
                f"[sdg_dataset] material {p.GetPath()}: texture_scale now {got}, bump_factor {got_bump}",
                flush=True,
            )
            n += 1
        if n == 0:
            print(
                f"[sdg_dataset] WARNING: no {self.terrain_material_match} shader prim found; "
                "texture scale NOT overridden",
                flush=True,
            )
        return n

    # ------------------------------------------------------------------ sun
    def apply_sun(self, sun: dict) -> None:
        self.set_sun_pose(orientation=sampling.sun_orientation_wxyz(sun["elevation_deg"], sun["azimuth_deg"]))
        self.set_sun_intensity(sun["intensity"])
        self.set_sun_color_temperature(sun["temperature_k"])

    def sample_and_apply_sun(self, min_elevation=None) -> dict:
        s = sampling.sample_sun(self.rng, self.ds.sun, min_elevation)
        self.apply_sun(s)
        return s

    # ------------------------------------------------------------------ records
    def randomize_frame(self, frame_in_terrain: int, global_index: int, render_tag: str) -> dict:
        sun = self.sample_and_apply_sun()
        rig = self.sample_rig()
        rec = {
            "index": global_index,
            "terrain_index": self.terrain_index,
            "terrain_seed": self.terrain_seed,
            "frame_in_terrain": frame_in_terrain,
            "render": render_tag,
            "sun": sun,
            "rig": rig,
        }
        self.frame_records.append(rec)
        return rec

    def flush(self, data_dir: str, extra: dict = None) -> None:
        man = {
            "base_seed": self.base_seed,
            "environment": self.__class__.__name__,
            "dataset_settings": self.ds.to_dict(),
            "intrinsics": self.intrinsics,
            "data_dir": data_dir,
            "frames": self.frame_records,
            **(extra or {}),
        }
        path = os.path.join(self.out_dir, "manifest.json")
        with open(path, "w") as f:
            json.dump(man, f)
        print(
            f"[sdg_dataset] manifest: {path} frames={len(self.frame_records)} bytes={os.path.getsize(path)}",
            flush=True,
        )
