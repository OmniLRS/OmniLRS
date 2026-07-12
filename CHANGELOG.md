# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

[WIP]

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
