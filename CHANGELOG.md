# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

[WIP]

> **Breaking:** migrated to Isaac Sim 6.2 (from 5.0). See [docs/migration_plan_6.2.md](docs/migration_plan_6.2.md).
>   - Python 3.12, numpy 2.3.1, numba 0.63, SciPy 1.17, Warp 1.17 (versions pinned by the Isaac Sim 6.2 wheels).
>   - Docker installation is now deprecated: Pixi is the only (and best) supported method. Docker is used for CI only.
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
- Yamcs mode reads the TM/TC definitions (MDB) from the running Yamcs server and checks the robot and controller config against them and the server's storage buckets at startup: it stops if something it sends or handles is missing and warns about extras. `pixi run -e yamcs check-yamcs-contract` (`scripts/check_yamcs_contract.py`) runs the same check without Isaac Sim. See [docs/yamcs-omnilrs-sync.md](docs/yamcs-omnilrs-sync.md).
- Pragyaan has three cameras: `NavCam_left` and `NavCam_right` (rgba) and `cam_depth` (depth).
- Husky has three cameras: `cam_color`, `cam_left` and `cam_right` (rgba), `cam_left` also depth. In Zenoh mode each is published per resolution (`OmniLRS/husky/camera/<camera>/<resolution>`; previously only `cam_color`).
- Pixi tasks `yamcs-husky` (Husky in Yamcs mode on `lunaryard_20m`, Yamcs instance `husky`) and `check-yamcs-contract-husky`.
- Artefacts CI scenario `test_yamcs_husky_sanity` (`pixi run test-yamcs-husky`): the Husky in Yamcs mode against yamcs-mission-control-husky (telemetry, camera captures and streaming, driving, GO/NOGO). The CI image also builds the Husky Yamcs server and generates its MDB from the OmniLRS commit under test.
- Unit tests run in CI on every push and pull request (`pixi run -e test-unit test-unit`). Tests are organized in `test/unit` (no Isaac Sim), `test/ci_artefacts` (sanity tests: the Artefacts CI scenarios) and `test/physics_models`; `check_yamcs_contract.py` moved to `scripts/`.
- `scripts/assets/rebuild_pragyaan.py` writes `nograph_pragyaan.usd` from `nograph_pragyaan_source.usd`, the asset as exported from Blender, and applies the Pragyaan physics fixes. The plan and the audit are in [docs/todo_pragyaan_physics.md](docs/todo_pragyaan_physics.md).
- Yamcs sanity test `test_integrated_solar_panel_drive_turn_imu`. It deploys the solar panel, drives 2 m, turns 90° and stows the panel. It compares the distance, heading, turn angle, gyroscope rate and IMU orientation with the ground truth pose. It prints every measurement before it fails.
- Zenoh simulation control keys `OmniLRS/sim/play`, `OmniLRS/sim/pause` and `OmniLRS/sim/stop` (any payload). A pause keeps the simulation state; a stop then play restarts from the initial state (environment and robot reset). `OmniLRS/sim/is_running` now also reports `is_playing`.

### Changed

- LargeScale static assets no longer implicitly add the terrain height to their z; set `height_above_terrain_override` instead.
- LargeScale robot spawn uses the configured pose in both ROS2 and Yamcs modes (ROS2 previously forced `(0, 0)`); the `largescale*` configs set `height_above_terrain_override: 0.5` to keep the previous spawn height.
- The Lunaryard workshop configs and `robot=pragyaan` now place the rover, lander, rock and monitoring camera relative to the terrain height.
- Robot yaml `camera` is a list of cameras, each with `name`, `prim_path`, `type` (`rgba` and/or `depth`) and `resolutions`, validated when the robot config is loaded in every mode (`src/configurations/robot_camera_confs.py`). In Yamcs mode, each camera has its own buckets (`images_<camera>_<kind>`), metadata and streaming telemetry (`/Rover/camera/<camera>/<kind>/...`), streaming job and compression model; the camera commands take a `camera_name` argument. Names come from `src/tmtc/yamcs_naming.py`, which the Yamcs MDB generators also use.
- Yamcs controller yaml: `mdb` (file list) is replaced by `mdb_space_systems`, `images.buckets` is keyed by role (`lander`, `monitoring`) and `images.clear_buckets_on_startup` is added (default true; with `false`, earlier images are kept and each bucket's numbering continues after its last image).
- Yamcs images are uploaded through the Yamcs storage API instead of written to `/tmp`. The image buckets are generated into the Yamcs `yamcs.yaml` together with the MDB, with limits sized to keep every image of a mission; OmniLRS never deletes images except when clearing the buckets at startup (`images.clear_buckets_on_startup`).
- The ROS2 Artefacts CI scenarios load the Husky robot yaml (`robot=husky`, with the ROS 2 Husky USD) instead of the robot settings of the environment yaml.
- Husky in Yamcs mode starts in GO and sends `/Rover/go_nogo`; it handles `go_nogo_command`: NOGO stops it and blocks the drive commands until GO (previously it could always drive and sent no GO/NOGO state).
- Husky Zenoh camera images are published at 5 Hz (`zenoh.camera.publish_period_s: 0.2`, previously 30 Hz), for each of its 6 camera streams.
- Pragyaan `/Rover/solar_panel_state` follows the measured solar panel angle: it changes to DEPLOYED or STOWED once the panel is within 10° of that position, instead of when the command is received. The power model uses the same state.

- Pragyaan asset `nograph_pragyaan.usd` is one flattened layer in the REP-103 frame (x forward, y left, z up), built by `scripts/assets/rebuild_pragyaan.py`. No Xform, rigid body or camera has a scale. The mesh transforms are baked into the points. Each rigid body has its origin on its joint. base_link has its runtime pose, so its IMU no longer turns 120° on the first physics step. Prim paths are unchanged. The spawn `orientation` in `cfg/robot/pragyaan.yaml` is now `[0.70711, 0., 0., 0.70711]` for the same world pose. If you set a custom Pragyaan orientation, set it again.
- Pragyaan has its real size: 0.913 m long, 0.943 m wide and 0.366 m high without the solar panel (published 0.917 × 0.750 × 0.397 m). The Blender model was 1.9 times too large. The spawn height in `cfg/robot/pragyaan.yaml` is 0.42 m.
- Pragyaan left and right rockers, bogies and wheels are symmetric about the plane of base_link. The pivots and axles moved by 6 mm or less, and the left meshes are the mirrored right meshes.

### Removed

- `assets/USD_Assets/robots/pragyaan/pragyaan.usd` (unused version with an action graph).
- User-facing Docker image `omnilrs.docker/`; install with pixi instead (Docker is only used for CI). `spaceros_demo.docker/` is kept as an unmaintained demo.
- `cfg/mdb/` (local MDB copies of Pragyaan and Husky), `scripts/fix_yamcs_tmp.sh` and the pixi `fix-tmp` task.

### Fixed

- Pragyaan weighs 26 kg. Before, it weighed 675 kg, because the asset had no masses and PhysX used its default density on the collider volumes. Each rigid body now has a mass, and PhysX computes the inertia from its colliders. `base_link` has no collider, so it gets an explicit inertia. This removes the "invalid inertia tensor ... negative mass" warning.
- Pragyaan rocker, bogie and motor meshes render with their materials. Their bindings pointed outside the asset, so they had no material.
- ROS2 mode exits cleanly on SIGINT/SIGTERM (ROS 2 Jazzy shuts down the ROS context on these signals).
- Yamcs mode with the LargeScale environment: terrain, rocks and colliders now update (EC.update()) as the  rover drives, terrain worker threads are joined on shutdown, and the simulation stops if a terrain worker dies.
- Joint commands from Yamcs and Zenoh are no longer dropped, entirely or for some joints, when they arrive while PhysX is stepping: `ArticulationControl` queues them and `Robot.update_articulation_api()` writes them from the simulation loop.
- Yamcs mode no longer sends an empty `/Rover/motor_encoder` before the wheel joints are measured (startup, timeline restart).
- Stopping and replaying the timeline (e.g. from the GUI) runs the ROS2, Yamcs and Zenoh restart handling again (environment and robot reset). It was skipped because Isaac Sim's physics warm-up advanced the step counter it relied on.

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
