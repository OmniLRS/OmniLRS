"""Simulation manager for mode=SDG dataset=<preset>: terrains x frames loop with settle steps and opt-in quality guards.

Stock SDG_SimulationManager records one step after each randomize(); under RTX real-time that frame still carries
temporal-AA history from the previous camera pose. settle_steps renders a few frames after every re-roll before
recording.
"""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import json
import os
import threading
import time

import numpy as np
import omni
from isaacsim.core.api.world import World

from src.environments_wrappers.sdg.dataset import guards
from src.labeling.auto_label import AutonomousLabeling


def save_rig_intrinsics(data_dir: str, intrinsics: dict, camera_names: list) -> None:
    """Writes <data_dir>/<cam>_intrinsics.json from the rig (the stock csv/npy use the USD camera attributes)."""
    for cam in camera_names:
        with open(os.path.join(data_dir, f"{cam}_intrinsics.json"), "w") as f:
            json.dump(intrinsics[cam], f, indent=1)


def start_exit_watchdog(timeout_s: float, exit_code: int = 0) -> None:
    """Isaac's close() has hung after the HR DEM workers shut down, and the HR DEM worker shutdown itself has hung;
    never leave a GPU-holding zombie. Forces os._exit(exit_code) timeout_s seconds from now."""

    def _watchdog():
        time.sleep(timeout_s)
        print(f"[sdg_dataset] watchdog: forcing process exit (code {exit_code})", flush=True)
        os._exit(exit_code)

    threading.Thread(target=_watchdog, daemon=True).start()


class SDGDataset_SimulationManager:
    def __init__(self, cfg, simulation_app) -> None:
        self.simulation_app = simulation_app
        self.cfg = cfg
        self.gen = cfg["mode"]["generation_settings"]
        self.ds = cfg["mode"]["dataset_settings"]
        self.cam = self.gen.camera_names[0]

        res = [list(r) for r in self.gen.camera_resolutions]
        assert all(r == res[0] for r in res), (
            f"SDG dataset renders every rig camera at one resolution; generation_settings.camera_resolutions "
            f"must be identical, got {res}"
        )
        for name in ("mesh_probe", "dark_frame", "auto_exposure"):
            if self.ds.guards[name]["enabled"]:
                annots = self.gen.annotators_list[0]
                if "depth" not in annots or "rgb" not in annots:
                    raise ValueError(f"guards.{name} needs the depth and rgb annotators on {self.cam}")

        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.world.get_physics_context().set_solver_type("PGS")
        self.world.reset()

        env_cfg = dict(cfg["environment"])
        env_kwargs = dict(
            flares_settings=cfg["rendering"]["lens_flares"],
            camera_settings=cfg["mode"]["camera_settings"],
            dataset=self.ds,
            camera_names=self.gen.camera_names,
            resolution=tuple(self.gen.camera_resolutions[0]),
        )
        if env_cfg["name"] == "Lunaryard":
            from src.environments_wrappers.sdg.dataset.lunaryard import DatasetLunaryard

            self.LC = DatasetLunaryard(**env_cfg, **env_kwargs)
        elif env_cfg["name"] == "LargeScale":
            from src.environments_wrappers.sdg.dataset.largescale import DatasetLargeScale

            self.LC = DatasetLargeScale(**env_cfg, **env_kwargs, is_simulation_alive=simulation_app.is_running)
        else:
            raise ValueError(
                f"SDG dataset generation supports environment Lunaryard or LargeScale, got {env_cfg['name']}"
            )
        self.LC.load()
        for _ in range(60):
            self.world.step(render=True)

        self.gen.prim_path = self.LC.scene_name + "/" + self.gen.prim_path
        self.gen.data_dir = self.LC.out_dir
        self.AL = AutonomousLabeling(self.gen)
        self.AL.load()
        save_rig_intrinsics(self.AL.data_dir, self.LC.intrinsics, self.gen.camera_names)
        # Proof of effect: list what was attached, so the log shows a missing annotator immediately.
        print("[sdg_dataset] annotators attached:", sorted(self.AL.annotators.keys()))
        print("[sdg_dataset] data_dir:", self.AL.data_dir)

        if self.ds.guards["pt_runtime_switch"]:
            # Selecting "PathTracing" in the SimulationApp config crashes librtx.raytracing on the first
            # real frame in this install (Isaac 5.0 / L40S / 580.65, headless). Boot RT and flip the
            # render mode at runtime instead.
            import carb.settings

            cs = carb.settings.get_settings()
            cs.set("/rtx/rendermode", "PathTracing")
            spp = int(cfg["rendering"]["renderer"].samples_per_pixel_per_frame or 32)
            cs.set("/rtx/pathtracing/spp", spp)
            cs.set("/rtx/pathtracing/totalSpp", 0)
            cs.set("/rtx/pathtracing/clampSpp", 0)
            for _ in range(3):
                self.world.step(render=True)
            print(
                f"[sdg_dataset] render mode switched at runtime: {cs.get('/rtx/rendermode')} "
                f"spp={cs.get('/rtx/pathtracing/spp')}",
                flush=True,
            )
        path_traced = self.ds.guards["pt_runtime_switch"] or cfg["rendering"]["renderer"].renderer == "PathTracing"
        self.render_tag = "pt" if path_traced else "rt"
        self.settle = int(self.ds.settle_steps)
        self.num_terrains = int(self.ds.num_terrains)
        self.fpt = int(self.ds.frames_per_terrain)
        self.count = 0
        self.skipped = []
        self._error = None
        print(
            f"[sdg_dataset] env={env_cfg['name']} plan: {self.num_terrains} terrains x {self.fpt} frames, "
            f"settle={self.settle}, render={self.render_tag}, renderer={cfg['rendering']['renderer'].renderer}",
            flush=True,
        )

    def _settle(self) -> None:
        for _ in range(self.settle):
            self.world.step(render=True)

    def run_simulation(self) -> None:
        # run.py never calls finish(): flush the manifest here, whether the loop ended or raised
        try:
            self._run()
        except BaseException as e:
            self._error = e
            raise
        finally:
            self.finish()

    def _run(self) -> None:
        g_cfg = self.ds.guards
        probe_on = bool(g_cfg["mesh_probe"]["enabled"])
        dark = g_cfg["dark_frame"]
        ae = g_cfg["auto_exposure"]
        self.timeline.play()
        t_start = time.time()
        for k in range(self.num_terrains):
            if not self.simulation_app.is_running():
                break
            self.LC.new_terrain(k)
            self._settle()
            t_k = time.time()
            n_settle = n_resample = n_reroll = n_skip = 0
            tol = float(g_cfg["mesh_probe"]["tol_m"])
            for f in range(self.fpt):
                # Per-frame mesh probe: every frame moves the rig and (LargeScale) re-centres the clipmap, and
                # a stale clipmap leaves the rendered mesh 0.3-2 m off the DEM on some frames while their
                # neighbours are at 2 mm. Remedies in order: more settle steps (render/pose lag), clipmap
                # re-sample (stale clipmap), then re-roll the rig spot; a location is skipped only on frame 0.
                rec = None
                for rig_try in range(3):
                    if rec is not None:
                        self.LC.frame_records.pop()
                        n_reroll += 1
                    rec = self.LC.randomize_frame(f, self.count, self.render_tag)
                    rec["guards"] = {}
                    self._settle()
                    err = self._mesh_probe(rec, k, quiet=f > 0) if probe_on else None
                    remedy = None
                    if err is not None and abs(err) > tol:
                        # up to 3 extra settle rounds: the clipmap/annotators catch up progressively, and a
                        # partially stale mesh (outer ring) still fails the p95 criterion after one round
                        for extra in range(3):
                            self._settle()
                            e2 = self._mesh_probe(rec, k, quiet=True)
                            remedy = f"settle{extra + 1}"
                            if e2 is None or abs(e2) <= tol:
                                break
                        if e2 is not None and abs(e2) > tol and hasattr(self.LC, "resample_clipmap"):
                            time.sleep(1.0)
                            self.LC.resample_clipmap()
                            self._settle()
                            e2 = self._mesh_probe(rec, k, quiet=True)
                            remedy = "resample"
                        print(
                            f"[sdg_dataset] PROBE loc {k} frame {f}: {err:+.2f} m -> {remedy} -> "
                            f"{e2 if e2 is None else round(e2, 3)} m",
                            flush=True,
                        )
                        err = e2
                        if remedy.startswith("settle"):
                            n_settle += 1
                        else:
                            n_resample += 1
                    if probe_on:
                        rec["guards"]["probe_err_m"] = None if err is None else round(float(err), 4)
                        rec["guards"]["probe_remedy"] = remedy
                    if err is None or abs(err) <= tol:
                        break
                else:
                    self.LC.frame_records.pop()
                    n_skip += 1
                    if f == 0:
                        print(
                            f"[sdg_dataset] PROBE loc {k}: still {err:+.2f} m after re-rolls -> SKIPPING location",
                            flush=True,
                        )
                        self.skipped.append(k)
                        break
                    print(
                        f"[sdg_dataset] PROBE loc {k} frame {f}: still {err:+.2f} m after 3 rig spots -> frame skipped",
                        flush=True,
                    )
                    continue
                g = rec["guards"]
                # Dark-frame guard: on real relief a low sun often leaves the whole view in terrain shadow
                # (black image, useless for training). Re-roll the sun and re-render, up to retries times,
                # before recording.
                if dark["enabled"]:
                    for attempt in range(int(dark["retries"])):
                        lit = self._measure_lit(g)
                        if lit >= float(dark["min_lit_fraction"]):
                            break
                        rec["sun"] = self.LC.sample_and_apply_sun(min_elevation=float(dark["retry_min_elevation_deg"]))
                        g["dark_retries"] = attempt + 1
                        self._settle()
                # Auto-exposure: the sin(elevation) rule assumes flat ground; sun-facing slopes blow out and
                # shaded ones go murky. Measure the lit terrain and rescale the sun once if it is off target.
                if ae["enabled"]:
                    if "terrain_mean_gray" not in g:
                        self._measure_lit(g)
                    gain = guards.exposure_gain(g["terrain_mean_gray"], ae)
                    if gain is not None:
                        new_i = rec["sun"]["intensity"] * gain
                        self.LC.set_sun_intensity(new_i)
                        rec["sun"]["intensity"] = round(new_i, 1)
                        g["ae_gain"] = round(gain, 3)
                        self._settle()
                        self._measure_lit(g)
                self.AL.record()  # let exceptions surface: a silent skip desyncs manifest vs files
                self.count += 1
            per = (time.time() - t_k) / self.fpt
            print(
                f"[sdg_dataset] terrain {k:04d} done: {self.fpt} frames, {per:.2f} s/frame, total {self.count} | "
                f"probe remedies: settle={n_settle} resample={n_resample} reroll={n_reroll} skipped={n_skip}",
                flush=True,
            )
            if k % 5 == 4:
                self.LC.flush(self.AL.data_dir, {"partial": True, **self._manifest_extra()})
        self.timeline.stop()
        print(f"[sdg_dataset] finished {self.count} frames in {(time.time() - t_start) / 60:.1f} min")

    def _mesh_probe(self, rec: dict, k: int, quiet: bool = False):
        """In-sim truth: unproject the first camera's depth image and compare the rendered ground height with the
        terrain manager's CPU height query at the same (x, y). Returns p95 |error| in metres, or None."""
        try:
            depth = np.asarray(self.AL.annotators[f"{self.cam}_depth"][2].get_data()).astype(np.float32)
            depth = depth.reshape(depth.shape[0], depth.shape[1])
        except Exception as e:  # noqa: BLE001
            print(f"[sdg_dataset] probe unavailable: {e}", flush=True)
            return None
        intr = self.LC.intrinsics[self.cam]
        # criterion = p95 of |err|: frames whose median was 0.00 m still had a peripheral region of the mesh
        # 2-5 m stale (rocks "buried" by metres at the image edge)
        p95 = guards.probe_error(depth, intr["K"], rec["rig"], intr["rig_offset_y_m"], self.LC.ground_height)
        if p95 is not None and not quiet:
            print(f"[sdg_dataset] PROBE loc {k}: p95 |z_render - z_dem| = {p95:.2f} m", flush=True)
        return p95

    def _lit_stats(self) -> tuple:
        """(lit fraction, mean gray) over terrain pixels (finite depth) in the first camera's RGB."""
        try:
            rgb = np.asarray(self.AL.annotators[f"{self.cam}_rgb"][2].get_data())
            depth = np.asarray(self.AL.annotators[f"{self.cam}_depth"][2].get_data())
        except Exception as e:  # noqa: BLE001
            print(f"[sdg_dataset] lit check unavailable: {e}", flush=True)
            return 1.0, 125.0
        return guards.lit_stats(rgb, depth)

    def _measure_lit(self, g: dict) -> float:
        """Stores lit_fraction and terrain_mean_gray in the frame's guard record; returns the raw lit fraction."""
        lit, mean = self._lit_stats()
        g["lit_fraction"] = round(lit, 3)
        g["terrain_mean_gray"] = round(mean, 1)
        return lit

    def _manifest_extra(self) -> dict:
        """Writer layout the offline tools need to find the files: <cam>_<annot>/<idx // epf>/<idx % epf>.<ext>."""
        return {"element_per_folder": int(self.gen.element_per_folder)}

    def finish(self) -> None:
        failed = self._error is not None
        # first, so it also covers a flush or LC.shutdown() that hangs or raises
        if self.ds.exit_watchdog_s:
            start_exit_watchdog(float(self.ds.exit_watchdog_s), exit_code=1 if failed else 0)
        extra = {
            "partial": failed,
            "frames_recorded": self.count,
            "skipped_locations": self.skipped,
            **self._manifest_extra(),
        }
        if failed:
            # the frame in flight has a record but no (complete) files: keep only the recorded frames
            del self.LC.frame_records[self.count :]
            extra["error"] = repr(self._error)
        self.LC.flush(self.AL.data_dir, extra)
        if hasattr(self.LC, "shutdown"):
            self.LC.shutdown()
        # Proof of effect: count files on disk for the first camera's RGB and compare.
        d = os.path.join(self.AL.data_dir, f"{self.cam}_rgb")
        n = sum(len(fs) for _, _, fs in os.walk(d)) if os.path.isdir(d) else 0
        print(f"[sdg_dataset] {self.cam}_rgb files on disk: {n} (manifest frames: {self.count})", flush=True)
