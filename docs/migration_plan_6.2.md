# OmniLRS Migration Plan: Isaac Sim 5.0.0 → 6.2.0

**Status:** Implemented on branch `migrate/isaacsim-6.2` (see [§0](#0-implementation-status)). Decisions agreed (see [§9](#9-decisions-agreed)): direct upgrade, compat-first, Jazzy everywhere, CI on 24.04, RTPT renderer, driver 595.x, pixi-only for users.

## 0. Implementation status

| Gate | Result |
|---|---|
| Lock/env | `pixi lock` resolves; `ci` + `test-sim` installed. numpy 2.3.1, numba 0.63.1, llvmlite 0.46, scipy 1.17.0, warp 1.17.0, torch 2.11 (cu130), a single cv2 distribution (`opencv-python-headless-noffmpeg`), gdal 3.13.3, zfpy and numpy-quaternion import and work |
| Static | `ruff check` + `ruff format --check` (py312) green, `compileall` green, no imports of removed symbols, all Warp kernels compile (CPU+CUDA) under Warp 1.17 |
| `test-ros2` (ROS2, lunalab) | ✅ 6 passed. 8 topic checks skipped (no system ROS CLI on the dev host). The topics and all TF frames were verified separately with the bundled Jazzy rclpy. |
| `test-ros2-largescale` | ✅ 8 passed, 9 skipped (same reason) |
| `test-yamcs` (lunaryard_40m_workshop, pragyaan) | ✅ 17 passed |
| `test-yamcs-largescale` (largescale_workshop, pragyaan) | ✅ 21 passed |
| Analysis-only paths (§8.1) | Done. No further changes needed (details in §8.1) |
| WorldBuilders | Submodule bumped to `c68be67` (WorldBuilders #7 deprecated imports + #8 ruff setup, incl. the `add_update_semantics` fix); all 4 canonical scenarios re-pass |
| Open | Host driver upgrade to 595.x (shared host, needs reboot). The CI base image `omni-ci-0.4.0` must be built from `ci.docker/ci.Dockerfile` and pushed. |
**Scope:** `OmniLRS/` (pixi envs, Docker, CI, `src/`, `WorldBuilders` submodule, tests, docs). Isaac Sim 7.x is out of scope.
**Sources:** Isaac Sim release notes for [5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html), [6.0.0 (EDR + GA)](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/overview/release_notes.html), [6.0.1](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/overview/release_notes.html), [6.1.0](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/overview/release_notes.html), [6.2.0](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/overview/release_notes.html), the [6.0](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/migration_guides/isaac_sim_6_0/index.html) and [6.1](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/migration_guides/isaac_sim_6_1/index.html) migration guides, the 6.2 install/ROS/container docs, and **files read directly from the published 6.2.0.0 pip wheels and the `nvcr.io/nvidia/isaac-sim:6.2.0` image config**. The wheel and image checks matter because several deps (numpy, Python, container OS) don't appear in the release notes.

> `docs/Migration_5.0_to_5.1.md` and `docs/Migration_5.0_to_5.1_Checklist.md` are superseded by this document. They were never executed (`pixi.toml` still pins 5.0.0) and they reference files that no longer exist (`src/sensors/*`, `src/configurations/mode/*`).

---

## 1. Executive summary

| | |
|---|---|
| **Recommendation** | **Upgrade directly from 5.0 to 6.2**, on a branch, in phases. Don't ship intermediate versions. Use 6.0/6.1 wheels only to bisect if a 6.2 regression shows up (see [§2](#2-direct-upgrade-or-intermediate-versions)). |
| **Biggest jump** | Almost all breakage lands at **6.0**: Python 3.11 → **3.12**, numpy 1.26 → **2.x** (6.2 pins `numpy==2.3.1`), Kit 107 → **110**, `omni.isaac.*` shims removed, core/sensor APIs deprecated, ROS 2 bridge split, mandatory multi-tick rendering, and a container based on **Ubuntu 24.04** that runs rootless. 6.1 and 6.2 are incremental. |
| **Hard blockers found** | 8 (see [§4.1](#41--blockers-will-not-start--will-not-resolve)): py3.12 env, numpy/packaging pins, numba/llvmlite, OpenCV wheel clash, **removed `add_update_semantics` (breaks lunalab/lunaryard in every mode)**, ROS lib path, Docker base OS and rootless user, custom `SimulationApp.close()` |
| **Good news** | The deprecated APIs OmniLRS depends on still ship in 6.2 under `isaacsim/extsDeprecated/`, which is on the default extension search path: `World`, `PhysicsContext`, `SingleArticulation`, `RigidPrim`, `SingleXFormPrim`, `isaacsim.core.utils.*`, `isaacsim.sensors.camera.Camera`, and `isaacsim.sensors.physics._sensor`. I checked this against the 6.2 sources. All PhysX collision schemas OmniLRS uses are still present, as are `semantics.schema.editor` (which now writes `UsdSemantics.LabelsAPI`), `omni.kit.viewport.actions` (with the same action names), `omni.physx.scripts.utils`, and the bundled ROS 2 Humble/Jazzy `rclpy` (cp312). A "compat-first" port is therefore feasible. |

---

## 2. Direct upgrade or intermediate versions?

| Option | Pros | Cons |
|---|---|---|
| **A. Direct 5.0 → 6.2 (recommended)** | One env/lockfile/container rebuild. NVIDIA's 6.0 guides are written as "5.x → 6.x" and don't require stepping through. 6.0 → 6.2 adds few OmniLRS-relevant breaks. Avoids fixing issues that 6.1/6.2 already fixed (shutdown hangs, Fabric pose fixes, Replicator stability). | A regression is harder to attribute to a specific version. **Mitigation:** keep 6.0.1/6.1 wheels available for bisecting. |
| B. 5.0 → 5.1 → 6.0 → 6.1 → 6.2 | Smaller diffs per step. 5.1 is an easy step (still py3.11/Kit 107). | Four env/lock/Docker/CI cycles. The 5.1 step teaches little: 5.0 already logs the same deprecations. The 6.0 step carries ~90% of the work either way. 6.0 and 6.1 have known bugs that 6.2 fixes. |
| C. 5.0 → 5.1 (checkpoint) → 6.2 | Low-risk checkpoint. | It adds a full cycle but doesn't lower the 6.x risk. |

**Verdict:** go with Option A. Treat 6.0.1 and 6.1.0 as **diagnostic bisect targets** only. All three install the same way (`isaacsim[all,extscache]==6.x`, py3.12).

---

## 3. Version-by-version digest (OmniLRS-relevant only)

Legend: 🔴 blocker · 🟠 breaking/behavior · 🟡 deprecation/verify · ⚪ info/positive

### 3.1 Platform/dependency matrix

| | **5.0.0 (current)** | 5.1.0 | 6.0.0 EDR | 6.0.0 GA | 6.0.1 | 6.1.0 | **6.2.0 (target)** |
|---|---|---|---|---|---|---|---|
| Kit SDK | 107.3.1 | 107.3.3 | 110.0.0 | 110.1.1 | 110.1.2 | 110.3.0 | **110.4.0** |
| Python | 3.11 | 3.11.13 | 3.12 | **3.12.13** | 3.12 | 3.12 | **3.12 (`==3.12.*` in wheel)** |
| numpy | 1.26 (pinned by us) | – | – | – | – | – | **2.3.1 (hard pin, `isaacsim-kernel`)** ✔ verified |
| Warp (`omni.warp.core`) | 1.7.x | 1.8.2 | 1.12.0 (`omni.warp` removed) | 1.13.0 | – | 1.16.0 | **1.17.0** |
| `omni.replicator.core` | 1.12.x | 1.12.27 | 1.13.4 | 1.13.25 | 1.13.27 | 1.13.36 | **1.13.38** |
| PhysX/`omni.usd.schema.physx` | 107.3.18 | 107.3.26 | 110.0.7 | 110.1.11 | 110.1.13 | 110.3.2 | **110.4.4** |
| torch (pulled by `isaacsim-core`) | – | – | 2.10 | 2.11 | – | – | **2.11.0 (+torchvision 0.26)** ✔ |
| scipy / OpenCV (pinned by `isaacsim-core`) | – | – | 1.17 / headless 4.13 | | | headless→`-noffmpeg` | **scipy==1.17.0, `opencv-python-headless-noffmpeg==4.14.0.94rc3`, `llvmlite==0.46.0`, `packaging>=23.2,<=26`** ✔ |
| Container base | Ubuntu 22.04, root | rootless | | | | | **Ubuntu 24.04, user `isaac-sim` (UID 1234), `MIN_DRIVER_VERSION=570.169`** ✔ |
| Tested driver | | | | | | | 595.58.03 (host has 580.178 ≥ min 570.169) |
| ROS 2 bundled libs | `isaacsim.ros2.bridge/humble/lib` | humble+jazzy, `system_default` | **moved to `isaacsim.ros2.core/{humble,jazzy}`** | | | auto ROS env in `python.sh` | **`isaacsim.ros2.core/{humble,jazzy}/{lib,rclpy}` (cp312)** ✔ |

✔ = read directly from 6.2.0.0 wheels or the image manifest.

### 3.2 5.1.0
- 🟡 "All deprecated extensions will be fully removed in the next major 6.0 release."
- 🟠 ROS 2 bridge default library = `system_default`: **Ubuntu 22.04 → Humble internal libs, Ubuntu 24.04 → Jazzy internal libs** (this also applies to the 24.04 host and the 6.2 container).
- 🟠 IMU sensor moved to device-side (GPU) processing. Re-validate IMU values.
- 🟠 `XFormPrim.get/set_world_poses` Fabric implementation rewritten (FSD/IFabricHierarchy) and affects `SingleXFormPrim` base pose.
- 🟠 `isaacsim.core.nodes` uses `SimulationManager` instead of the deprecated CoreNodes time APIs.
- 🟠 `SimulationApp.close()` gains `skip_cleanup`. Shutdown hang fixed. Linux without `DISPLAY` forces headless.
- 🟠 Docker container runs rootless.
- ⚪ CameraInfo fy fix, ROS timestamp duplicates fixed, Warp 1.8.2, Replicator 1.12.27.

### 3.3 6.0.0 Early Developer Release (Kit 110.0)
- 🔴 ROS 2: monolithic `isaacsim.ros2.bridge` split into `isaacsim.ros2.core/nodes/ui/examples`. **Internal libs path moved to `isaacsim.ros2.core`.** Python 3.12, so Jazzy system installs can be sourced directly. The `isaacsim.ros2.bridge` meta-extension still exists in 6.2 and depends on the four new extensions.
- 🟠 Multi-tick rendering. RTX sensors use Hydra time (`omni.timeline`).
- 🟠 `isaacsim.sensors.camera` deprecated (Camera now uses `_fast` annotator variants). `isaacsim.sensors.physics` deprecated: sensors rewritten on experimental prims with legacy interface shims, sensor period/frequency params removed (physics rate), creation commands renamed `*Experimental*`.
- 🟠 `omni.physx` start_simulation/event stream replaced by `omni.physics`. Timeline API replaced by app utils inside Isaac extensions.
- 🟠 SimulationApp: separate default render settings for PathTracing / RealTimePathTracing. `close()` waits for Replicator even with `rep.orchestrator.step()`. Auto-close at interpreter exit. Uses `close_stage_async`.
- 🟠 PhysX residual-reporting APIs removed (not used by OmniLRS).
- 🟠 `omni.warp` extension removed (only `omni.warp.core`). Warp 1.12.
- ⚪ Torch 2.10, scipy 1.17, opencv-headless 4.13.

### 3.4 6.0.0 GA (Kit 110.1.1)
- 🔴 **Removed `omni.isaac.*` compatibility shims.** OmniLRS `src/` is clean. `WorldBuilders/test_*.py` still use `omni.isaac.kit`/`omni.isaac.core`.
- 🟡 **Deprecated `isaacsim.core.api`, `isaacsim.core.prims`, `isaacsim.core.utils`** (replacements: `isaacsim.core.experimental.*`, `SimulationManager`, `RenderingManager`). They still ship in 6.2 under `extsDeprecated/`.
- 🔴 Per the Core migration guide, the old semantics helpers `add_update_semantics`, `remove_all_semantics`, `get_semantics`, `check_*_semantics`, and `count_semantics_in_scene` are removed. Use `add_labels`/`get_labels`/`remove_labels`/`upgrade_prim_semantics_to_labels` (`UsdSemantics.LabelsAPI`). **I confirmed in the 6.2 source** that `isaacsim.core.utils.semantics` only exports the `*_labels` helpers.
- 🟡 Deprecated `isaacsim.sensors.camera` (→ `isaacsim.sensors.experimental.rtx`: `RtxCamera` + `CameraSensor`) and `isaacsim.sensors.physics` / `isaacsim.sensors.physx` (→ `isaacsim.sensors.experimental.physics`: `IMU` + `IMUSensor`).
- 🔴 (for RTX Lidar users only) Removed Camera prims as RTX sensors via JSON configs.
- 🟠 **Non-multitick code paths removed.** `/rtx/hydra/supportMultiTickRate=true` is mandatory. Simulation time goes through the `/ExternalSimulationTime` Fabric prim (`set_next_simulation_time` removed).
- 🟠 `SimulationContext.add_physics_callback` now registers **pre-step** (was post-step). OmniLRS doesn't use it, but check before adopting it.
- 🟠 `SimulationContext.get_physics_context()` raises `RuntimeError` before init (previously returned `None`).
- 🟠 Euler helpers (experimental transform utils) expect `[roll, pitch, yaw]` XYZ order. This only matters if you migrate `quat_to_rot_matrix` users to experimental utils.
- 🟠 Importers switched to codeless PhysX schema usage. `pxr.PhysxSchema` still ships in 6.2 (verified) and all OmniLRS collision APIs exist in the 6.2 `generatedSchema.usda`.
- 🟠 PhysX removed legacy cloth/deformable bodies (OmniLRS terrain deformation is pure Python/Warp, so it isn't affected). `GpuMaxDeformableVolumeContacts` rename (OmniLRS sets `gpu_max_soft_body_contacts`, so verify).
- 🟠 Robot assets in `Isaac/Robots/` are migrating to `Isaac/Robots_Multiphysics/` (OmniLRS uses local assets, so not affected).
- ⚪ Bundled Fast-RTPS for Humble downgraded 2.6.11 → 2.6.10. Simulation-interfaces v1.4 (Humble) / v1.5 (Jazzy).
- ⚪ ROS workspaces: `use_internal_libs` default False on Jazzy, True on Humble.

### 3.5 6.0.1 (Kit 110.1.2)
- 🟠 Replicator: `_is_replicator_capturing` replaced by an annotator-ownership check. Async rendering no longer re-enables on stop/pause while a capture pipeline is attached.
- 🟠 SimulationApp: `shutdown_watchdog_timeout` (default 120 s), and `sync_loads` really forces synchronous loads.
- ⚪ ROS 2 dynamic-message leak fixed.

### 3.6 6.1.0 (Kit 110.3.0)
- 🟠 SimulationApp: **`limit_cpu_threads` default 32 → 16**, new `active_cuda_gpus` (mutually exclusive with `active_gpu`), `close(exit_code=…)` runs the same cleanup regardless of exit code.
- 🟡 **`read_gravity` deprecated** on `IMUSensor.get_sensor_reading/get_data` (OmniLRS passes `read_gravity=True`).
- 🟠 IMU acceleration now includes the ω×v transport term and normalizes `gravityDirection`. This changes IMU values for rotating bodies (rover on slopes).
- 🟠 **`python.sh` auto-configures bundled ROS 2 env** (`--no-ros-env` to opt out). This affects the Docker workflow. Pixi doesn't use `python.sh`.
- 🟠 Replicator 1.13.36 "annotator output stability". Camera `CameraSensor` errors on unexpected element counts.
- 🟠 `omni.physx.scripts.utils` replaced *inside Isaac extensions* by `isaacsim.core.experimental.utils.physics`. The module still ships in 6.2 (verified), so `WorldBuilders/pxr_utils.py` keeps working.
- 🟠 OpenCV in the Isaac pip set is now `opencv-python-headless-noffmpeg`.
- ⚪ Warp 1.16, Newton 1.5 (PhysX remains default).

### 3.7 6.2.0 (Kit 110.4.0)
- 🟡 **Ubuntu 22.04 support deprecated** (still supported in 6.2). This affects the current CI base image (`artefacts/ros2:humble-harmonic-nvidia`, 22.04).
- 🟠 `get_sensor_reading()` in experimental physics sensors returns an **owned snapshot** (`get_sensor_reading_view()` gives live, read-only data). Call it every step. OmniLRS already does.
- 🟠 Default asset root moved to the 6.2 catalog. Asset-root check timeout 5 → 10 s (startup slower when offline; `/persistent/isaac/asset_root/timeout`).
- 🟠 `NonVisualMaterial` no longer authors a `UsdPreviewSurface` for shaderless materials (relevant only to WorldBuilders `PreviewSurface` tests).
- 🟠 ROS 2 sim control prefers robot-schema joint names (legacy fallback deprecated). Auto-generated names use absolute prim paths. OmniLRS uses raw `rclpy`, so this doesn't apply.
- 🟠 URDF/MJCF importers come from the `isaacsim-asset` wheel. Controllers/cuMotion come from `isaacsim-robot-motion`. Both are covered by `[all]`.
- ⚪ Kit 110.4, Warp 1.17, Replicator 1.13.38, PhysX 110.4.4, Newton 1.6.1 (PhysX still default), `aioboto3` removed, Cortex removed.

---

## 4. Impact on OmniLRS (verified against the code)

### 4.1 🔴 Blockers (will not start / will not resolve)

| # | Area | Current | Problem in 6.2 | Required change | Introduced |
|---|---|---|---|---|---|
| B1 | Python | `pixi.toml` `python = "3.11.*"` (core + test-sim), `scripts/activate_isaac_ros.sh` hard-codes `python3.11` | Wheels are `cp312` only, `Requires-Python ==3.12.*` | `python = "3.12.*"`. Fix the path in the activation script. `pyproject.toml` `target-version = "py312"`. | 6.0 |
| B2 | numpy / packaging | `numpy = "==1.26.0"`, `packaging = "==23.0"` (pixi), `numpy==1.26.4`/`packaging==23.0` (Dockerfile) | `isaacsim-kernel` pins `numpy==2.3.1`. `isaacsim-core` needs `packaging>=23.2,<=26`. The lock can't resolve. | Remove both pins. Let Isaac drive numpy 2.3.1. Audit numpy-2 API use (scan: no removed aliases found; Warp `copy=False` is fine). | 6.x |
| B3 | numba | `numba = "*"` | `isaacsim-core` pins `llvmlite==0.46.0`, so only **numba 0.63.x** works (`numpy<2.4`) | Pin `numba = "==0.63.*"` | 6.2 |
| B4 | OpenCV | `opencv-python = "*"` (pixi core), `--no-deps opencv-python` (Docker) | Isaac installs `opencv-python-headless-noffmpeg==4.14.0.94rc3`. Two distributions overwrite the same `cv2` package (classic broken-cv2 scenario). | Drop `opencv-python` from Isaac envs (`src/` only uses headless-safe cv2). Keep it in `zenoh-client` (uses `cv2.imshow`). | 6.1 |
| B5 | **Semantics** | `src/labeling/instancer.py`: `from isaacsim.core.utils.semantics import add_update_semantics` | **Removed.** `instancer` is imported by `src/environments/rock_manager.py`, so **lunalab and lunaryard fail with ImportError in every mode**. | Replace with `add_labels(prim, [label], instance_name="class")` (6.2 helper) or `semantics.schema.editor.PrimSemanticData.add_entry` (already used elsewhere and writes `LabelsAPI`). | 6.0 |
| B6 | ROS 2 libs | `activate_isaac_ros.sh` → `.../python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib`. Dockerfile `LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib` | Path no longer exists. Libs are at `exts/isaacsim.ros2.core/<distro>/lib` (verified in wheel) | Update both paths and make the distro configurable. Verify `rclpy` import after `enable_extension("isaacsim.ros2.bridge")` (meta-ext still exists → `ros2.core`). | 6.0 |
| B7 | Docker | `FROM isaac-sim:5.0.0`, apt installs `ros-${ROS_DISTRO=humble}-*` from `packages.ros.org/$UBUNTU_CODENAME`, runs as root, writes `/root/.bashrc` | 6.2 image is **Ubuntu 24.04 (noble)** with **user `isaac-sim`**. Humble packages don't exist for noble, and apt as non-root fails. | `USER root` for apt, then back to `isaac-sim`. Pick ROS distro (Jazzy on noble). Fix `.bashrc` location, `run_docker.sh` volume ownership (UID 1234), and `LD_LIBRARY_PATH`. Remove the numpy pin. | 5.1 / 6.x |
| B8 | App shutdown | `SimulationApp_wait.close()` re-implements close using private `_app`, `_exiting`, `_framework`. It calls `_framework.unload_all_plugins()` after `_app.shutdown()`. | In 6.2, `fast_shutdown=True` by default makes `_app.shutdown()` exit the process. Upstream deliberately removed framework unload (GIL deadlock, NVBug 5948099) and added a watchdog plus stdio flush. The override bypasses both. | Keep only the `wait_for_threads` hook and then call `super().close(wait_for_replicator=...)`. Drop the copied body. | 5.1–6.1 |

### 4.1.1 🔴 Additional blockers found during implementation (runtime)

| # | Area | Problem in 6.2 | Fix applied |
|---|---|---|---|
| B9 | IMU sensor | The deprecated `isaacsim.sensors.physics` isn't enabled by the default app. Enabling it afterwards doesn't help: `isaacsim.sensors.physics.nodes` (enabled by default) has already registered `isaacsim.sensors.physics` as a namespace package, so `from isaacsim.sensors.physics import _sensor` fails (`unknown location`). | `src/robots/robot.py` reads the IMU through `isaacsim.sensors.experimental.physics.IMUSensor` (lazy, raw reading). The output dict and the quaternion order (x, y, z, w) are unchanged for consumers. This is a deliberate small exception to compat-first. |
| B10 | Deprecated core exts | Not guaranteed to be enabled by the 6.x app | `startSim()` enables `isaacsim.core.api/prims/utils` and `isaacsim.sensors.camera` explicitly via the Kit extension manager (`LEGACY_EXTENSIONS`). |
| B11 | `numpy-quaternion` | Imported by `WorldBuilders/Clippers.py`. It used to be bundled with Isaac Sim and was removed in 5.1 ("Removed unused python packages: numpy-quaternion…"). | Added `numpy-quaternion` to `[feature.core.pypi-dependencies]` |
| B12 | **Crater spline profiles** (`assets/Terrains/crater_spline_profiles.pkl`) | SciPy 1.17 (pinned by Isaac) changed `PPoly`/`CubicSpline` to slots `_c/_x/_asarray`. Profiles pickled with older SciPy unpickle **without coefficients** (`'CubicSpline' object has no attribute '_asarray'`). The large-scale DEM workers swallow the exception (`except Exception: pass`), so the startup **hangs forever** at `HR DEM … collecting...`. `terrain_generation.py` (lunaryard procedural craters) is affected too. | Asset regenerated once: the 29 splines were rebuilt from their pickled breakpoints and coefficients with SciPy 1.17 and re-pickled. The curves are identical. The code still uses plain `pickle.load`. ⚠ The new file can't be unpickled by SciPy < 1.17 (i.e. Isaac Sim 5.x envs). |
| B13 | ROS 2 shutdown | Jazzy `rclpy` handles SIGINT/SIGTERM itself: the executors raise `ExternalShutdownException` and the sim loop kept running (it had to be SIGKILLed). | ROS2 loop also checks `rclpy.ok()`. The executor threads exit quietly on `ExternalShutdownException`. SIGINT now exits in about 4 s. |
| B14 | Warp 1.17 | Constructing matrices from vectors (`wp.mat33f(vx, vy, normal)`) was removed in Warp 1.12. Rock placement on largescale terrain failed to compile (`geometry_clipmaps_warp.py::_get_random_tangent_vector`). | `wp.matrix_from_cols(vx, vy, normal)`. Inside kernels the old constructor treated the vectors as columns (Warp GH-278), so rock orientations are unchanged. |
| B15 | numpy scalars in `Gf` | 6.2's pxr rejects numpy scalars other than float64 (`np.float32`, `np.int64`, 0-d arrays) and whole arrays in `Gf.Vec*`/`Gf.Quat*` constructors (`ArgumentError`); 5.0 accepted them. The canonical runs pass because config values arrive as Python floats (and `resolve_position` casts), but numpy-derived poses would crash. Found by #143. | Explicit casts at the numpy-fed sites: `robot.load` (ported from #143), `labeling/instancer.py`, `environments/utils.py` xform helpers, large-scale `rock_manager`, `collider_builder` and `geometry_clipmaps_manager`. Config-sourced and stellar-engine (float64) sites need no change. |

### 4.1.2 Observations (not blocking)
- **Log spam (fixed):** `[PoseTree] target getObjectType eInvalid` about 90 times per second, from `ros2_husky_PhysX_vlp16.usd` → `/husky_robot/Graphs/ROS_Odometry/TFRobot`. That `ROS2PublishTransformTree` used the deprecated `targetPrims` input on non-rigid prims (vlp16, Camera_Pseudo_Depth, Imu_Sensor). The graph was migrated per NVIDIA's "ROS 2 OmniGraph Nodes" guide: a new `TFRobot_ComputeTransformTree` (`isaacsim.core.nodes.IsaacComputeTransformTree`) takes the targets and parent, and feeds `parentFrames/childFrames/translations/orientations` into the publisher. Only that graph prim changed in the asset. Result: 0 PoseTree warnings, no deprecation warning, the same 8 TF pairs, and identical transform values (checked against the original). The same migration was applied to `ros2_husky_PhysX_vlp16_mono_depth_imu.usd`, `ros2_jackal_PhysX_vlp16.usd`, `ros2_jackal_PhysX_vlp16_stereo.usd` and `ex1/ex1_spheres.usd`. Each was run in lunalab ROS2 mode, original vs migrated: PoseTree warnings 3580/6349/3036/0 → 0, no deprecation warning, identical TF frame pairs (9/12/8/7), and sensor frame transforms unchanged (wheel and suspension links differ ≤ 0.002, which is run-to-run physics settling).
- Verified ROS topics on lunalab with Jazzy: `/clock /cmd_vel /odom /pointcloud /tf /rosout /parameter_events` plus all `/OmniLRS/*` control topics.
- `torch` resolves to the `cu130` build, which needs NVIDIA driver ≥ 580 (the host's 580.178 is OK).
- `__pycache__` dirs in the repo owned by root (left by the old Docker workflow) block bytecode writes for the user. Cosmetic.

### 4.2 🟠 Behavior changes to validate (code may need tuning)

| # | Area | OmniLRS location | What changes | Action |
|---|---|---|---|---|
| H1 | Renderer mode | `cfg/rendering/ray_tracing.yaml` `renderer: "RayTracedLighting"`, `src/environments/rendering.py` (`set_renderer_rtx_realtime` action → `RaytracedLighting`) | 6.x default is **`RealTimePathTracing`** (RTX Real-Time 2.0). `RaytracedLighting` (legacy RT) is still accepted. `RENDERER_DEFAULTS` only cover PT/RTPT. | Decide the renderer (see D5). Re-tune `max_bounces` etc. Eyeball lunar lighting in the canonical runs. |
| H2 | RTX post settings | `rendering.py`: `/rtx/post/lensFlares/*`, `/rtx/post/motionblur/*`, `/rtx/post/chromaticAberration/*`, `/rtx/hydra/TBNFrameMode` | Not mentioned in the notes. Kit 110 RTX may have renamed or ignored keys, so keys can silently no-op. | Runtime check: read back each setting after set and look for visual effect. |
| H3 | Multi-tick rendering + loop runner | `World(physics_dt, rendering_dt)`, `world.step(render=True)`, `src/environments_wrappers/rate.py`. `largescale.yaml` uses 60 Hz physics / 30 Hz render. | Non-multitick path removed. `isaacsim.exp.base.kit` sets `runLoops.main.manualModeEnabled=true` and `useFixedTimeStepping=false`. Render cadence vs physics substeps may differ. | Measure physics/render step counts, sim-time vs wall-time, and camera frame cadence per mode. |
| H4 | CPU threads | SimulationApp launch config | `limit_cpu_threads` 32 → 16 | Set it explicitly in `cfg/rendering/*.yaml` if throughput drops (decision D-minor). |
| H5 | IMU values | `src/robots/robot.py::get_imu_readings` (`_sensor.acquire_imu_sensor_interface().get_sensor_reading(..., read_gravity=True)`), used by Yamcs/Zenoh/ROS | Legacy interface now shims the experimental implementation: device-side processing, ω×v term, gravity-direction normalization, physics-rate reads. `read_gravity` deprecated. | Plausibility check in the canonical runs (stationary rover ≈ lunar g). Remove `read_gravity` or migrate to `IMUSensor` (D2). |
| H6 | Cameras | `robot.py` (RGB + depth `Camera` per resolution on the same prim → 2×N render products), `monitoring_cameras_manager.py`, `pragyaan/tmtc/camera_handler.py` | Deprecated but working. Uses `_fast` annotators. `attach_annotator` before `initialize` now raises. Multi-tick frame timing. | Validate shapes/dtypes/latency of `get_rgba`/`get_rgb`/`get_depth`. Optionally consolidate into one `CameraSensor(annotators=["rgb","distance_to_image_plane"])` (D2). |
| H7 | Replicator SDG | `src/labeling/auto_label.py`, `rep_utils.py`, `SimulationApp_wait.close` (orchestrator) | Replicator 1.12 → 1.13.38. Segmentation/instance annotator output and ID mapping, colorize. Orchestrator ownership semantics (6.0.1). | Run SDG mode on lunalab/lunaryard and diff the outputs (`test/test_labeling_writers.py`). |
| H8 | Semantic labels on assets | Rock/terrain USDs, `PrimSemanticData.add_entry` | Annotators read `UsdSemantics.LabelsAPI`. Assets authored with legacy `Semantics.SemanticsAPI` may lose labels. | Check rock/lander USDs. Run `upgrade_prim_semantics_to_labels` once and re-save if needed. |
| H9 | ROS distro selection | pixi `ROS_DISTRO=humble`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, host Ubuntu 24.04 | `ros_distro = "system_default"` picks **Jazzy internal libs on 24.04** and Humble on 22.04. The docs say "Humble on Ubuntu 24.04 is not supported." Cross-distro DDS between Jazzy (sim) and Humble (external `omnilrs_ros2_demo/humble_ws`) works for basic msgs but isn't officially supported. | Decide the distro (D3). Force via `--/exts/isaacsim.ros2.bridge/ros_distro=` and the env vars. Test topics end-to-end with external nodes. |
| H10 | World/physics | `src/physics/physics_scene.py` (PhysicsContext sim_params incl. `gpu_max_soft_body_contacts`, broadphase via `SimulationManager`), `robot.py` (`RigidPrim(track_contact_forces=True)`, `SingleArticulation` measured forces) | Deprecated wrappers run on the new Simulation Manager. Tensor-view invalidation (the code already has a workaround comment). Fabric pose rewrite. | Covered by the canonical runs (articulation telemetry, terramechanics); analysis for the rest. |
| H11 | Startup/asset root | tests (`test/test_*_sanity*.py` watch log lines incl. `carb.crashreporter-breakpad`), offline runs | Asset-root check timeout 10 s. Log text changed. Python launcher auto-ROS env. | Update log-match patterns and timeouts in tests. Optionally set `/persistent/isaac/asset_root/timeout`. |
| H12 | Env size | pixi envs | `isaacsim-core` pulls **torch 2.11 + torchvision/torchaudio**, newton, mujoco (multi-GB) | Expect bigger envs and CI images. Keep `dedup-envs`. |
| H13 | conda libstdc++ | pixi (conda-forge) | 6.2 needs `libstdc++` with `CXXABI_1.3.15` in conda envs | Add `libstdcxx-ng >=13` (or `libstdcxx`) to `[feature.core.dependencies]`. |
| H14 | zfpy | `src/terrain_management/large_scale_terrain/utils.py` | Only `zfpy 1.0.1` cp312 wheels exist. numpy-2 ABI is unverified. | Smoke-import with numpy 2.3.1. Fall back to the conda-forge `zfpy` build if it fails. |

### 4.3 🟡 Deprecated-but-working (decide whether to modernize now; see D2)

| Deprecated (works in 6.2 via `extsDeprecated`) | Replacement | OmniLRS files |
|---|---|---|
| `isaacsim.core.api.World`, `PhysicsContext` | `isaacsim.core.experimental.utils.stage` + `SimulationManager` + `RenderingManager` | `environments_wrappers/{ros2,yamcs,zenoh,sdg}/simulation_manager_*.py`, `physics/physics_scene.py`, `robots/robot.py`, `terrain_manager.py` (`__main__`) |
| `isaacsim.core.prims.SingleArticulation`, `RigidPrim`, `SingleRigidPrim`, `SingleXFormPrim`, `isaacsim.core.utils.types.ArticulationAction` | `isaacsim.core.experimental.prims.Articulation/RigidPrim/XformPrim` (array-based) | `robots/articulation_control.py`, `articulation_telemetry.py`, `robot.py` |
| `isaacsim.core.utils.{prims,stage,xforms,rotations,extensions}` | `isaacsim.core.experimental.utils.{prim,stage,transform,app}` | `mission_specific/*/subsystems/*`, `labeling/instancer.py`, `environments/lunalab.py`, `ros2/__init__.py`, `robot.py` |
| `isaacsim.sensors.camera.Camera` | `isaacsim.sensors.experimental.rtx.RtxCamera` + `CameraSensor` | `robot.py`, `monitoring_cameras_manager.py`, `pragyaan/tmtc/camera_handler.py` |
| `isaacsim.sensors.physics._sensor` IMU interface, `read_gravity` | `isaacsim.sensors.experimental.physics.IMUSensor(path).get_data()` (keys `linear_acceleration`, `angular_velocity`, `orientation`) | `robot.py`, `zenoh/telemetry/imu_bridge.py` (consumer) |

Note: NVIDIA states these will be removed in a future major release. Version 7 is out of scope here, but every deprecated call is a future break.

### 4.4 ⚪ Not affected / already OK (verified)
- `src/` has no `omni.isaac.*` imports. The only remnants are `WorldBuilders/test_*.py` (dev scripts, not imported at runtime).
- PhysX schemas used (`PhysxRigidBodyAPI`, `PhysxCollisionAPI`, `PhysxTriangleMeshCollisionAPI`, `PhysxTriangleMeshSimplificationCollisionAPI`, `PhysxConvexHull/ConvexDecomposition/SphereFillCollisionAPI`) are all present in the 6.2 `PhysxSchema`.
- `omni.physx.scripts.utils` still ships (`WorldBuilders/pxr_utils.py`).
- `semantics.schema.editor` 2.0.2 ships and `PrimSemanticData.add_entry` writes `UsdSemantics.LabelsAPI`.
- `omni.kit.viewport.actions` 110.1.1 still has `set_renderer_rtx_realtime` / `set_renderer_rtx_pathtracing`.
- ROS messages used (`std_msgs`, `geometry_msgs`, plus executors) are bundled in Isaac's cp312 Humble/Jazzy `rclpy`.
- No Python-3.12-removed stdlib usage (`distutils`, `imp`, `asyncore`, `pkg_resources`) in OmniLRS code.
- No NVIDIA asset-root / Nucleus references in `src/` or `cfg/` (assets are local), so the asset catalog move doesn't apply.
- Warp kernels use only stable APIs (`wp.array/zeros/launch/kernel/func/vec*/mat*/quatf/quat_from_matrix/tid/synchronize/init`). Re-test them anyway under Warp 1.17.

---

## 5. Necessary changes by workstream

### WS1: Environment and dependencies (`pixi.toml`, `pyproject.toml`, `pixi.lock`)
1. `python = "3.12.*"` in `core` and `test-sim`. `glibc = "2.35"` platform is OK.
2. `isaacsim = { version = "==6.2.0.0", extras = ["all", "extscache"] }`.
3. Remove `numpy==1.26.0` and `packaging==23.0` from `feature.isaac`.
4. `numba = "==0.63.*"`. Drop `opencv-python` from `core` (move to `zenoh-client`). Add `libstdcxx-ng >=13` (conda).
5. `pyproject.toml`: pin `warp-lang==1.17.0` (matches `omni.warp.core`/newton pin) or remove it; `ruff target-version = "py312"`.
6. Regenerate `pixi.lock` (CI `pixi-lock.yaml` will enforce). Re-check `gdal`/`libgdal` (conda-forge py312 + numpy 2), `zfpy`, `av`, `skyfield`, `hydra-core`, `omegaconf`, `eclipse-zenoh==1.10.1` (cp312 wheels), `yamcs-client`.
7. `feature.ros2.activation.env`: set `ROS_DISTRO` per decision D3. Update `scripts/activate_isaac_ros.sh` to `${CONDA_PREFIX}/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/${ROS_DISTRO}/lib`.

### WS2: App launch and shutdown (`src/environments_wrappers/__init__.py`, `cfg/rendering/*.yaml`)
1. Replace the body of `SimulationApp_wait.close()` with: run `wait_for_threads`, then `super().close(wait_for_replicator=wait_for_replicator)`. Drop the `_framework.unload_all_plugins()` and private-state handling.
2. Renderer value per D5. Optionally set `limit_cpu_threads` and (on multi-GPU hosts) `active_cuda_gpus`.
3. Verify the `/rtx/post/*` and `/rtx/hydra/TBNFrameMode` keys still take effect (H2).

### WS3: Semantics and SDG (`src/labeling/*`, `src/environments/rock_manager.py`)
1. `instancer.py`: `add_update_semantics` → `add_labels` (B5).
2. Validate Replicator annotators and writers (H7). Upgrade legacy semantic schemas in assets if needed (H8).

### WS4: Sensors (`src/robots/robot.py`, monitoring/lander cameras, Zenoh/Yamcs/ROS consumers)
- **Minimum:** drop `read_gravity` (deprecated). Plausibility-check IMU/camera outputs in the canonical runs (H5, H6).
- **If modernizing (D2):** migrate to `IMUSensor` and `CameraSensor`/`RtxCamera`. Map dict keys and quaternion convention (w,x,y,z). Merge RGB+depth into one render product per resolution.

### WS5: Core API (`World`, `PhysicsContext`, prims): only if D2 = modernize
- Replace `World` lifecycle with `stage_utils` + `SimulationManager.step()` + `RenderingManager.render()`. Replace `SingleArticulation`/`ArticulationAction` with experimental `Articulation` (batched arrays). Replace `RigidPrim(track_contact_forces)` with the experimental `RigidPrim` contact tracking. Callbacks register pre-step.
- This is the largest optional piece of work. It touches all four simulation managers plus the robot, control, and telemetry code.

### WS6: ROS 2 (`src/environments_wrappers/ros2/__init__.py`, activation, Docker)
1. `enable_ros2()`: keep `enable_extension("isaacsim.ros2.bridge")` (meta-ext) or enable `isaacsim.ros2.core` directly. Replace the `foxy`/`humble` switch with `humble`/`jazzy` and force `--/exts/isaacsim.ros2.bridge/ros_distro`.
2. Update `cfg/mode/ROS2.yaml` `bridge_name`. End-to-end test with `omnilrs_ros2_demo` external nodes (H9).

### WS7: Docker (`omnilrs.docker/*`)
1. `FROM nvcr.io/nvidia/isaac-sim:6.2.0`. Add `USER root` around apt/pip system steps. Restore `USER isaac-sim`.
2. ROS apt distro per D3 (noble → `jazzy`). Remove numpy/packaging pins. Remove `--no-deps opencv-python`. Make `gdal==$(gdal-config --version)` build against py3.12/numpy 2 (noble GDAL 3.8).
3. `LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/${ROS_DISTRO}/lib` (or rely on 6.1+ `python.sh` auto ROS env). Aliases go to the isaac-sim user's `.bashrc`.
4. `run_docker.sh`: cache/log mounts writable by UID 1234 (`setfacl`, per NVIDIA docs). Update `entrypoint.sh` accordingly.

### WS8: CI and tests (`ci.docker/`, `artefacts.Dockerfile`, `artefacts.yaml`, `.github/workflows/*`, `test/`)
1. Rebuild the base image (`omni-ci-*` tag bump) with the new lock. Decide the base OS per D4 (22.04 still supported, deprecated).
2. Update test log patterns and timeouts (H11). Ruff on py312. Add a startup smoke test for **lunalab/lunaryard** (this would have caught B5) and for SDG.

### WS9: WorldBuilders submodule
- `test_*.py` use `omni.isaac.kit`/`omni.isaac.core` and `PreviewSurface`. Fix them upstream (fork/PR) or leave them alone (D6). Runtime modules (`pxr_utils`, `Types`, `Mixer`) are OK.

### WS10: Documentation
- `README.md` install section (py3.12, 6.2, driver ≥570, recommended 595), `CHANGELOG.md`, `docs/` (supersede the 5.0→5.1 docs). Move `MigrationSteps.md` into `docs/` history. New startup-warnings reference file for 6.2 (from the canonical runs).

---

## 6. Phased execution plan

| Phase | Goal | Exit gate |
|---|---|---|
| **0. Prep** | Branch `migrate/isaacsim-6.2`. Upgrade the host driver to 595.x (D7). No 5.0 reference capture (per decision). | Driver 595.x active. `nvidia-smi` OK. |
| **1. Env** | WS1. `pixi install` resolves. `python -c "import isaacsim, numpy, numba, cv2, zfpy, gdal, warp"` works. | Lockfile committed, `pixi lock --check` green |
| **2. Blockers** | B5, B6, B8, launch config (WS2), ROS path (WS6). | App starts in the canonical scenarios |
| **3. Static analysis (all modes × envs)** | Code walk of every mode (ROS2, Yamcs, Zenoh, SDG) × every environment config (`cfg/environment/*.yaml`) × robot (`husky`, `pragyaan`, `ex1`, `ssf`) × renderer (`ray_tracing`, `path_tracing`) × static assets. Trace the imports and API calls touched by 6.2 (B*/H* items) and fix what analysis finds. Grep gate: no removed symbol imported anywhere, and `ruff` + `python -m compileall` on py3.12. | Analysis matrix in the doc (§8.1) fully ticked |
| **4. Canonical runs (the only runtime gate)** | The four `artefacts.yaml` scenarios: `test_ros2_sanity` (ROS2, lunalab), `test_ros2_sanity_largescale` (ROS2, largescale), `test_yamcs_sanity` (Yamcs, lunaryard_40m_workshop, pragyaan), `test_yamcs_sanity_largescale` (Yamcs, largescale_workshop, pragyaan). Locally via `pixi run test-*`, then in CI. | All four pass |
| **5. CI** | WS8 (CI on 24.04/Jazzy). WS7: remove the user-facing `omnilrs.docker/`. | Artefacts CI green on the four scenarios |
| **6. Modernize (follow-up PR, not in this migration per D2)** | WS4/WS5 full migration to experimental APIs. | – |
| **7. Docs & cleanup** | WS9, WS10. Document expected behavior deltas (H1–H10) from analysis, not measured, for users to watch for. | PR ready |

---

## 7. Risks and mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| numpy 2 breaks numba/zfpy/GDAL code paths in terrain generation | Med | High (largescale terrain) | numba 0.63 pin. Run the large-scale terrain workers early (Phase 1 smoke). Have the conda-forge zfpy/gdal fallback ready. |
| Visual change from RT → RTPT renderer or Kit 110 RTX alters lunar imagery (shadows, regolith BRDF, TBN mode) | High | Med (SDG datasets, camera telemetry) | No reference capture. Document the expected change. Eyeball frames from the canonical runs (cameras are published in the ROS2/Yamcs scenarios). |
| IMU numeric changes (ω×v, device-side) affect downstream Yamcs/Zenoh consumers and tests | Med | Med | Document the expected delta from the release notes. Sanity-check IMU values on a stationary rover in the canonical runs (≈ lunar g, orientation plausible). |
| Multi-tick/loop-runner change alters real-time pacing or camera cadence (60/30 Hz largescale) | Med | Med | Analyze `World.step`/`rate.py` against the 6.2 loop settings. Watch timing in the largescale canonical runs. |
| Paths not exercised by the canonical runs (Zenoh, SDG, lunaryard_20m/80m, deformable envs, husky/ex1/ssf, path tracing) break at runtime | Med | Med | Phase 3 static analysis matrix. Grep/compile gates. List them as "analysis-only" in release notes. |
| Jazzy (sim) ↔ Humble (external) DDS interop | Med | Med for the ROS demo | Jazzy everywhere (D3). `omnilrs_ros2_demo` migration is follow-up work. |
| Deprecated APIs behave subtly differently (now shims over the experimental core) | Med | Med | Canonical runs cover articulation, IMU, cameras, and physics context. Phase 6 modernization later. |
| Driver upgrade on a shared host | Med | Med | Coordinate downtime (D7). |
| Larger envs/images (torch, newton) slow CI | High | Low | `rdfind` dedup. Cache the base image. |
| Ubuntu 22.04 deprecation (CI base) | Low now | Future | Plan a move to a 24.04 image (D4). |

---

## 8. Validation checklist
- [ ] `pixi install -e default -e ros2 -e yamcs -e zenoh -e ci -e test-sim` and `pixi lock --check`
- [ ] Import smoke: `isaacsim`, `numpy==2.3.1`, `numba`, `cv2` (single distribution), `zfpy`, `osgeo.gdal`, `warp==1.17`, `rclpy` (ros2 env)
- [ ] Static gates: no import of removed symbols (`add_update_semantics`, `omni.isaac.*` in runtime code, `isaacsim.ros2.bridge/humble/lib` paths), `ruff check` (py312), `python -m compileall src WorldBuilders`
- [ ] Static analysis matrix (§8.1) complete
- [ ] **Canonical runtime scenarios (`artefacts.yaml`)**: `pixi run test-ros2`, `test-ros2-largescale`, `test-yamcs`, `test-yamcs-largescale` pass locally
- [ ] Clean shutdown in those runs (no hang, exit code 0)
- [ ] User Docker removed; CI image (24.04/Jazzy) builds; artefacts CI green on the four scenarios
- [ ] Ruff green on py312

### 8.1 Static analysis matrix (code analysis only, not run)

| Mode \ Env | lunalab (+4SDG, deformable) | lunaryard 20/40/80m (+workshop/_full, 4SDG, deformable) | largescale (+200km, alt_rocks, workshop) |
|---|---|---|---|
| ROS2 | **run** (canonical) | analysis | **run** (canonical) |
| Yamcs | analysis | **run** (40m_workshop, canonical) | **run** (workshop, canonical) |
| Zenoh | analysis | analysis | analysis |
| SDG | analysis | analysis | n/a (verify config support) |

Also analyze: robots `husky`/`pragyaan`/`ex1`/`ssf` (+ controllers), static assets `ispace_ultra`/`ssf_rover_static`, renderers `ray_tracing`/`path_tracing`, `mission_specific/{husky,pragyaan}` subsystems/TMTC, `scripts/zenoh/*`, `terrain_manager.py __main__`.

**Analysis results:**
- **Saved OmniGraph node types** in all USD assets referenced by `cfg/` and `src/`: every type is `isaacsim.*` (no removed `omni.isaac.*` nodes), and all are still registered in 6.2. They come from `isaacsim.core.nodes`, `isaacsim.ros2.nodes` (still under the `isaacsim.ros2.bridge.*` type names), `isaacsim.robot.wheeled_robots.nodes`, `isaacsim.sensors.physics.nodes`, the deprecated `isaacsim.sensors.physx` (PhysX lidar, enabled by default; `/pointcloud` verified), and `omni.graph.scriptnode` (`ex1`). Only the husky TF graph is affected, and only by the deprecated `targetPrims` input (log spam, §4.1.2).
- **SDG:** the Replicator 1.13.38 annotators (`rgb`, `distance_to_image_plane`, `semantic_segmentation`/`instance_segmentation` with `colorize`) and the info keys `idToLabels`/`idToSemantics` are unchanged. `test/test_labeling_writers.py` passes. Semantic labels are written via `UsdSemantics.LabelsAPI` (B5).
- **Zenoh:** uses the same `World`/timeline/robot APIs as the tested modes. `eclipse-zenoh==1.10.1` resolves for py3.12.
- **Deformable envs:** `deformation_engine.py` is pure numpy (no PhysX deformables).
- **`path_tracing`:** `PathTracing` is still a valid renderer. 6.x applies the PT defaults only for keys missing from the config, and OmniLRS sets all of them.
- **Other lunaryard/largescale variants:** same code paths as the canonical scenarios. The crater profiles fix (B12) and the Warp fix (B14) cover all of them.
- **Pre-existing, not migration-related:** `cfg/robot/ssf.yaml` references `assets/isaacsim/ssf_rover/ssf_rover_velocity_light.usd`, which is missing from the checkout. `scripts/install_native.sh` still targets Isaac Sim 2023.1.

---

## 9. Decisions (agreed)

| ID | Decision | Choice | Consequences in this plan |
|---|---|---|---|
| D1 | Upgrade path | **Direct 5.0 → 6.2** | Phases in §6. 6.0.1/6.1 wheels only for bisecting. |
| D2 | API scope | **Compat-first.** Keep deprecated `core.api`/`core.prims`/`core.utils`/`sensors.camera`/`sensors.physics`. Fix only removals, blockers, and deprecated *arguments*. | WS5 and the full WS4 migration move to a follow-up PR (Phase 6 not part of this migration). Drop `read_gravity` now. |
| D3 | ROS 2 distro | **Jazzy everywhere** | pixi `ROS_DISTRO=jazzy`. Activation path `isaacsim.ros2.core/jazzy/lib`. Force `--/exts/isaacsim.ros2.bridge/ros_distro=jazzy` (or `system_default` on 24.04). `enable_ros2()` accepts `jazzy` (and keeps `humble` for 22.04). `cfg/mode/ROS2.yaml` `bridge_name: jazzy`. Tests source `/opt/ros/jazzy/setup.bash`. **External:** `omnilrs_ros2_demo/humble_ws` (sibling repo) is **separate follow-up work after this migration**: move it to Jazzy and to pixi. It is out of scope here, but parity testing uses a plain Jazzy external node (`ros2 topic list/echo`). |
| D4 | CI base OS | **Move CI to Ubuntu 24.04 now** | `ci.docker/ci.Dockerfile` `FROM public.ecr.aws/artefacts/ros2:jazzy-harmonic-nvidia` (exists, 24.04). Rebuild and bump `omni-ci-*` tag in `artefacts.Dockerfile`. Yamcs JDK/venv steps re-checked on noble (`openjdk-17` is available on 24.04). |
| D5 | Ray-tracing renderer | **Switch to `RealTimePathTracing` only** | `cfg/rendering/ray_tracing.yaml` `renderer: "RealTimePathTracing"` with RTPT bounce defaults (3/3/15) reviewed against current 6/6/4. `src/configurations/rendering_confs.py` validator accepts `RealTimePathTracing` (and drops `RayTracedLighting`). `src/environments/rendering.py::enable_RTX_real_time` currently runs viewport action `set_renderer_rtx_realtime`, which **sets legacy `RaytracedLighting`** in Kit 110, so it must set `RealTimePathTracing` instead (via viewport `set_renderer("rtx","RealTimePathTracing")` or `/rtx/rendermode`). This is also triggered by the ROS 2 topic `/OmniLRS/Render/EnableRTXRealTime` (`base_wrapper_ros2.py::use_RTX_real_time_render`). Visual check in the canonical runs. |
| D6 | WorldBuilders tests | **Fix** | Submodule PR: `omni.isaac.kit` → `isaacsim`, `omni.isaac.core` → `isaacsim.core.api`, `add_update_semantics` → `add_labels`, `PreviewSurface` from `isaacsim.core.api.materials`. Then bump the submodule pointer. |
| D7 | Host driver | **Upgrade to 595.x** (NVIDIA-tested 595.58.03) | Phase 0 host step. ⚠ Needs sudo and a reboot, and the machine is shared, so coordinate downtime. |
| D8 | Docker | **Docker for CI only. Users: pixi only.** | Remove the user-facing `omnilrs.docker/` (Dockerfile, build/run/entrypoint scripts) and its README references. `ci.docker/` + `artefacts.Dockerfile` remain and are migrated (D4). `spaceros_demo.docker/` stays as an **unmaintained demo** (not migrated; add a README note that it targets Isaac Sim 5.0). WS7 shrinks to "delete and redirect docs". |
| – | Doc location | `OmniLRS/docs/migration_plan_6.2.md` | – |
| – | Old docs | Remove superseded untracked `docs/Migration_5.0_to_5.1.md` and `docs/Migration_5.0_to_5.1_Checklist.md` | – |

### 9.1 Workstream deltas resulting from the decisions
- **WS1:** `[feature.ros2.activation.env] ROS_DISTRO = "jazzy"` (keep `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`).
- **WS2:** renderer per D5. Explicitly set `limit_cpu_threads` (decide after the parity check).
- **WS4:** remove `read_gravity=True` from `robot.py::get_imu_readings` (validate that gravity is still included in `lin_acc_*` as consumers expect).
- **WS6:** Jazzy (above). Remove the dead `foxy` branch.
- **WS7:** delete `omnilrs.docker/`, update README/docs to the pixi-only install, and add a note for users of the old Docker flow. Leave `spaceros_demo.docker/` untouched apart from an "unmaintained, targets Isaac Sim 5.0" note.
- **Follow-up (out of scope):** migrate `omnilrs_ros2_demo` to Jazzy and pixi. Phase 6 API modernization.
- **WS8:** CI on 24.04 Jazzy. `test/test_ros2_sanity*.py` source `/opt/ros/jazzy/setup.bash`. Extend the sanity matrix to lunalab/lunaryard + SDG.
- **Phase 0:** driver upgrade only (no reference capture).
- **Validation scope (agreed):** runtime = the four `artefacts.yaml` scenarios only. Everything else = static code analysis (§8.1).
