# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

[WIP]

> **Breaking:** migrated to Isaac Sim 6.2 (from 5.0). See [docs/migration_plan_6.2.md](docs/migration_plan_6.2.md).
>   - Python 3.12, numpy 2.3.1, numba 0.63, SciPy 1.17, Warp 1.17 (versions pinned by the Isaac Sim 6.2 wheels).
>   - ROS 2 uses the **Jazzy** libraries bundled with Isaac Sim (`ROS_DISTRO=jazzy`, `bridge_name: jazzy`); external nodes should run Jazzy.
>   - `cfg/rendering/ray_tracing.yaml` uses the `RealTimePathTracing` renderer (replaces the legacy `RayTracedLighting`); the `/OmniLRS/Render/EnableRTXRealTime` topic switches to it as well.
>   - The IMU is read through `isaacsim.sensors.experimental.physics.IMUSensor`; the reported values keep the same format.
>   - `assets/Terrains/crater_spline_profiles.pkl` was re-pickled with SciPy 1.17 (identical curves; no longer loadable with SciPy < 1.17).
>   - ROS 2 robot assets (`ros2_husky_PhysX_vlp16*.usd`, `ros2_jackal_PhysX_vlp16*.usd`, `ex1/ex1_spheres.usd`): TF graph migrated to `IsaacComputeTransformTree` → `ROS2PublishTransformTree` (removes the deprecated `targetPrims` path and its per-frame PoseTree warnings; published TF is unchanged).
>   - Poses passed to USD `Gf` types are cast to Python floats (Isaac Sim 6.2's pxr rejects non-float64 numpy scalars).
>   - CI runs on Ubuntu 24.04 / ROS 2 Jazzy (`public.ecr.aws/artefacts/ros2:jazzy-harmonic-nvidia`).
>   - OmniLRS no longer uses or enables the Isaac Sim APIs deprecated in 6.0 (`isaacsim.core.api/prims/utils`, `isaacsim.sensors.camera`); details in [docs/TODO_6.2.md](docs/TODO_6.2.md).
>   - Cameras use Isaac Sim's `CameraSensor`: robot cameras render RGBA and depth from one render product per resolution, frames requested before a camera has rendered are skipped (Yamcs, Zenoh), and cameras below 300 px (e.g. the 320×240 monitoring camera) render without post anti-aliasing. The pragyaan and Vikram lander camera prims now author square-pixel apertures (images unchanged).
>   - `World` and `PhysicsSceneManager` are replaced by `Simulation` (`src/simulation.py`, also applies `cfg/physics`), which `RobotManager.preload_robot()` now takes.
>   - Joint control and telemetry use Isaac Sim's experimental `Articulation`; the `ArticulationControl` / `ArticulationTelemetry` interface is unchanged (`joint_name_to_index` removed, use `joint_names`).

### Added

- Optional yaml config key `pose.height_above_terrain_override` for robots, static assets and monitoring cameras: when set, the (absolute) z of pose.position is overriden by the (absolute) terrain height at (x, y) plus this value. Works the same in Lunaryard, Lunalab and LargeScale.
- `largescale_workshop` environment: the LargeScale counterpart of `lunaryard_40m_workshop` (lander, rock and monitoring camera), differing only in terrain.
- Yamcs mode reads the TM/TC definitions (MDB) from the running Yamcs server and checks the robot and controller config against them and the server's storage buckets at startup: it stops if something it sends or handles is missing and warns about extras. `pixi run -e yamcs check-yamcs-contract` (`test/check_yamcs_contract.py`) runs the same check without Isaac Sim. See [docs/yamcs-omnilrs-sync.md](docs/yamcs-omnilrs-sync.md).
- Pragyaan has three cameras: `NavCam_left` and `NavCam_right` (rgba) and `cam_depth` (depth).

### Changed

- LargeScale static assets no longer implicitly add the terrain height to their z; set `height_above_terrain_override` instead.
- LargeScale robot spawn uses the configured pose in both ROS2 and Yamcs modes (ROS2 previously forced `(0, 0)`); the `largescale*` configs set `height_above_terrain_override: 0.5` to keep the previous spawn height.
- The Lunaryard workshop configs and `robot=pragyaan` now place the rover, lander, rock and monitoring camera relative to the terrain height.
- Robot yaml `camera` is a list of cameras, each with `name`, `prim_path`, `type` (`rgba` and/or `depth`) and `resolutions`. In Yamcs mode, each camera has its own buckets (`images_<camera>_<kind>`), metadata and streaming telemetry (`/Rover/camera/<camera>/<kind>/...`), streaming job and compression model; the camera commands take a `camera_name` argument. Names come from `src/tmtc/yamcs_naming.py`, which the Yamcs MDB generators also use.
- Yamcs controller yaml: `mdb` (file list) is replaced by `mdb_space_systems`, `images.buckets` is keyed by role (`lander`, `monitoring`) and `images.clear_buckets_on_startup` is added.
- Yamcs images are uploaded through the Yamcs storage API instead of written to `/tmp`. The image buckets are generated into the Yamcs `yamcs.yaml` together with the MDB, with limits sized to keep every image of a mission; OmniLRS never deletes images except when clearing the buckets at startup (`images.clear_buckets_on_startup`).

### Removed

- User-facing Docker image `omnilrs.docker/`; install with pixi instead (Docker is only used for CI). `spaceros_demo.docker/` is kept as an unmaintained demo.
- `cfg/mdb/pragyaan/` (local MDB copies), `scripts/fix_yamcs_tmp.sh` and the pixi `fix-tmp` task.

### Fixed

- ROS2 mode exits cleanly on SIGINT/SIGTERM (ROS 2 Jazzy shuts down the ROS context on these signals).
- Yamcs mode with the LargeScale environment: terrain, rocks and colliders now update (EC.update()) as the  rover drives, terrain worker threads are joined on shutdown, and the simulation stops if a terrain worker dies.

See also the [Roadmap and Milestones tracking](https://github.com/orgs/OmniLRS/projects/1)

## [3.0.0] - 2026-06-29

> **Breaking:** Robot control now uses the ArticulationControl API instead of Dynamic Control, and environment/physics config layouts and the mission database were reorganized. Existing config files and integrations must be updated.

### Added

- Yamcs Mission Control System integration for telemetry/telecommand and ground-station-style operations.
- Pragyaan rover integration for mission-realistic reference of Yamcs mode implementation.
- Husky robot controller for another reference example of Yamcs mode implementation.
- Framework for multi-physics simulation of power, radio, thermal, On-Board Computer and payload telemetry.
- Monitoring and lander camera streams, plus fault injection for admin control of the simulation for training and testing.
- New terrain assets and a script to preview DEMs before use.
- Pixi-based installation with multi-environment support, simplifying setup across machines.

### Changed

- Migrated robot control from Dynamic Control to the ArticulationControl telemetry API.
- Reworked environments into a mixin-based architecture (terrain, stellar/ephemeris, rock control).
- Refactored environment, physics, and mission-database (per-rover) modules for clarity and reuse (separates main modules and mission-specific modules).
- Adopted Ruff for linting and formatting (replacing Black) and cleaned up source headers and dead code.
- Reorganized, reviewed and expanded the [OmniLRS Wiki](https://github.com/OmniLRS/OmniLRS/wiki)

### Fixed

- Sim launches with mode=Yamcs now wait for the Yamcs server to be ready, preventing simulation crashes.
- Dynamic Sun positioning and rock instancing above/below the ground surface in the large scale environment.
- Terrain deformation behavior.
- Deformable-scene initialization via `SingleXFormPrim` base link.

## [2.5.0] - 2025-11-13

### Added

- Migration to Isaac Sim 5.0.
- Depth camera support and static-asset placement.

### Changed

- Convex-hull colliders for rover body and wheels for more stable physics.
- Streamlined environment configs.

## [2.0.0] - 2024-09-11

### Added

- Large-scale environment built on geometry clipmaps, with GPU-accelerated (Warp) and hybrid CPU/GPU mesh updates for kilometre-scale terrain.
- Stellar-engine ephemeris for physically-based Sun/Earth positioning.
- Git-LFS-tracked assets and lunar rocks, and SpaceROS (Humble) support.

### Changed

- Improved and expanded Synthetic Data Generation pipeline.
- Homogenized environment configs and code across small- and large-scale scenes.
- Optimized terrain/rock generation for efficiency: numba-based mesh backbone, ~2x faster rock sampling, seed/RNG-state storage, and DEM-by-reference handling to cut memory use.
- ROS1 wrappers for large-scale environments, lunaryard launch crash, and negative-time risk.

## [1.0.0] - 2024-07-24

### Added

- Lunar environments (Lunalab, Lunaryard) with procedural craters, rocks, and wheel traces.
- Synthetic data generation with semantic labels.
- ROS1/ROS2 wrappers, a robot API, and an extensible base environment.
- Terrain colliders and configurable physics scene.
- Terrain deformation engine (wheel traces).
- Docker images, SpaceROS demo, and documentation/wiki.

[Unreleased]: https://github.com/OmniLRS/OmniLRS/compare/v3.0.0...HEAD
[3.0.0]: https://github.com/OmniLRS/OmniLRS/compare/v2.5...v3.0.0
[2.5.0]: https://github.com/OmniLRS/OmniLRS/releases/tag/v2.5
