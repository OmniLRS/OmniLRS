# Isaac Sim 6.2: deprecated APIs still used by OmniLRS

Follow-up to the 6.2 migration ([migration_plan_6.2.md](migration_plan_6.2.md)), which was done "compat-first": OmniLRS
keeps using the APIs that Isaac Sim 6.0 deprecated. They ship under `isaacsim/extsDeprecated/` and are force-enabled at
startup by `LEGACY_EXTENSIONS` in `src/environments_wrappers/__init__.py`:
`isaacsim.core.api`, `isaacsim.core.prims`, `isaacsim.core.utils`, `isaacsim.sensors.camera`.

They work in 6.2 (all 4 canonical scenarios pass), but NVIDIA has given no removal version. Sources: 6.0 release notes
("Deprecated `isaacsim.core.api`, `isaacsim.core.prims`, and `isaacsim.core.utils`", "Deprecated
`isaacsim.sensors.camera`") and the 6.0 migration guide "Core API to Core Experimental API".

Already migrated in 6.2: IMU (`isaacsim.sensors.physics` -> `isaacsim.sensors.experimental.physics.IMUSensor`, forced
because the old import is broken) and the whole "others" group (section B).

## Recommendation

- Wait for NVIDIA to stabilize the replacements before rewriting World / Articulation / Camera. The replacements are
  `*.experimental.*` modules, and their APIs may still change between releases. NVIDIA's own extensions are being
  ported in that direction (6.0 release notes), so watch each release for promotion out of `experimental`.
- Remove the matching entry from `LEGACY_EXTENSIONS` only after the last import of that extension is gone. All four
  are still needed by group A.

## A. World / Articulation / Camera (large, behavior-sensitive)

### A1. `World` (`isaacsim.core.api.world.World`) and `PhysicsContext`

| Where | Usage |
|---|---|
| `src/environments_wrappers/{ros2,yamcs,zenoh,sdg}/simulation_manager_*.py` | `World(...)`, `world.step/reset/stop/is_playing/get_physics_dt/current_time_step_index` |
| `src/environments_wrappers/sdg/simulation_manager_sdg.py` | `world.get_physics_context().set_solver_type("PGS")` |
| `src/terrain_management/terrain_manager.py` (`__main__` test block) | `World(stage_units_in_meters=1.0)` |
| `src/robots/robot.py` | `World` type hint, prim initialization tied to world reset |
| `src/physics/physics_scene.py` | `PhysicsContext(...)`, `enable_ccd`, `set_broadphase_type`, `set_solver_type` |

Replacement (no drop-in; per the migration guide "there is no single replacement object"):

- Stage: `isaacsim.core.experimental.utils.stage` (`create_new_stage`, `open_stage`, `set_stage_units`, ...).
- Stepping, callbacks, physics settings: `isaacsim.core.simulation_manager.SimulationManager` (`step`, `get_physics_dt`,
  `set_physics_dt`, `get_num_physics_steps`, `is_simulating`, `register_callback`, `set_solver_type`,
  `set_broadphase_type`, `enable_ccd`, `add_physics_scene`, ...).
- Rendering: `isaacsim.core.rendering_manager.RenderingManager.render()`.
- Play/stop: `omni.timeline` or `isaacsim.core.experimental.utils.app` (`play`, `stop`, `is_playing`).

Risks: `World.reset()` currently initializes physics views for every registered prim; this must become explicit. The
main loops depend on `current_time_step_index == 0` to detect first step after reset. SimulationManager runs only in
multi-tick mode (`/rtx/hydra/supportMultiTickRate` must stay `true`).

### A2. Articulation (`isaacsim.core.prims.SingleArticulation`, `isaacsim.core.utils.types.ArticulationAction`)

| Where | Usage |
|---|---|
| `src/robots/articulation_control.py` | `SingleArticulation(...).initialize()`, `dof_names`, `apply_action(ArticulationAction(joint_positions / joint_velocities / joint_efforts, joint_indices))` |
| `src/robots/articulation_telemetry.py` | `SingleArticulation`, `get_joint_positions/velocities`, `get_measured_joint_efforts/forces` |
| `src/robots/robot.py` | `set_joint_velocities`, `set_joint_positions`, `initialize()` |

Replacement: `isaacsim.core.experimental.prims.Articulation` (batched, array-first API):
`dof_names`, `get_dof_indices`, `set_dof_position_targets`, `set_dof_velocity_targets`, `set_dof_efforts`,
`get_dof_positions/velocities`, `get_dof_efforts`, `get_dof_projected_joint_forces`, `get_link_incoming_joint_force`,
`set_dof_positions/velocities`, `switch_dof_control_mode`. `ArticulationAction` has no equivalent: call the setters
directly.

Risks: values are batched (shape `(N, dofs)`), and the backend may return warp arrays instead of numpy. Measured
efforts/forces semantics must be checked against the current telemetry (ROS / Yamcs tests read them). No
`initialize()`; physics view creation follows `SimulationManager`.

### A3. Camera (`isaacsim.sensors.camera.Camera`)

| Where | Usage |
|---|---|
| `src/robots/robot.py` | `Camera(...)`, `initialize()`, `add_distance_to_image_plane_to_frame()`, `get_rgba()`, `get_depth()` |
| `src/mission_specific/pragyaan/tmtc/camera_handler.py` | `Camera(...)`, `initialize()`, `get_rgba()`, `get_rgb()` (Yamcs images) |
| `src/environments/monitoring_cameras_manager.py` | `Camera(...)`, `initialize()` |

Replacement: `isaacsim.sensors.experimental.rtx` (`RtxCamera` authoring + `CameraSensor` runtime with
`get_data(annotator)`), or plain Replicator (`rep.create.render_product` + `AnnotatorRegistry`, already used in
`src/labeling/auto_label.py`).

Risks: the deprecated `Camera` adjusts `verticalAperture` to force square pixels (pragyaan camera: 1.529 -> 1.572, seen
in CI logs); the replacement may not, which changes intrinsics and images. The deprecated extension also pulls in torch
at runtime (blocks dropping torch from the CI image, see `ci.docker/TODO.md`).

### A4. Wheel-link rigid prims (`isaacsim.core.prims.SingleRigidPrim`, `RigidPrim`)

| Where | Usage |
|---|---|
| `src/robots/robot.py` (`RobotRigidGroup`) | `SingleRigidPrim` + `RigidPrim(track_contact_forces=True)` per target link: `get_world_pose`, `get_velocities`, `get_net_contact_forces`, `apply_forces_and_torques_at_pos` |

Replacement: `isaacsim.core.experimental.prims.RigidPrim` (has all four methods, batched, warp outputs).

Deferred from B: the prims are created in every scenario, but their data is only read by terrain deformation /
terramechanics (`lunaryard_*_deformable`), which the 4 canonical scenarios do not cover. Physics view creation is tied
to `World.reset()` (A1). Validate with a deformable scenario when migrating.

## B. Others (done)

| Deprecated | Where | Now |
|---|---|---|
| `isaacsim.core.prims.SingleXFormPrim` (base link) | `src/robots/robot.py` | path + `experimental.utils.xform.get_world_pose` |
| `isaacsim.core.utils.stage.add_reference_to_stage` | `src/environments/lunalab.py` | `experimental.utils.stage.add_reference_to_stage` |
| `isaacsim.core.utils.prims.is_prim_path_valid`, `delete_prim` | `src/mission_specific/{husky,pragyaan}/subsystems/*_subsystems_handler.py`, `src/labeling/instancer.py` | `experimental.utils.prim.is_prim_valid`, `experimental.utils.stage.delete_prim` |
| `isaacsim.core.utils.xforms.get_world_pose` | `src/mission_specific/{husky,pragyaan}/subsystems/*_subsystems_handler.py` | `experimental.utils.xform.get_world_pose` |
| `isaacsim.core.utils.rotations.quat_to_rot_matrix` | `src/robots/robot.py` | scipy `Rotation.from_quat(q, scalar_first=True).as_matrix()` |
| `isaacsim.core.utils.extensions.enable_extension` | `src/environments_wrappers/ros2/__init__.py` | `experimental.utils.app.enable_extension` |
| `isaacsim.core.utils.semantics.add_labels` | `src/labeling/instancer.py` | `experimental.utils.semantics.add_labels` |
| `semantics.schema.editor.PrimSemanticData` | `src/terrain_management/terrain_manager.py`, `large_scale_terrain/rock_manager.py`, `large_scale_terrain/nested_geometry_clipmaps_manager.py` | `experimental.utils.semantics.add_labels` |

Behavior notes:

- Experimental utils return warp float32 arrays on the default (CUDA) device. Call sites pass `device="cpu"` and
  convert with `.numpy()`. The old helpers returned numpy (float64 for `xforms.get_world_pose`, float32 for
  `SingleXFormPrim`); both read USD, as the new ones do.
- Quaternions from `get_world_pose` may have the opposite sign to before (`q` and `-q` are the same rotation).
- `PrimSemanticData.add_entry` (semantics.schema.editor 2.x) already wrote `UsdSemantics.LabelsAPI`, so labels on the
  stage are unchanged. The new `add_labels` only appends (no replace mode); the instancer labels freshly created prims,
  so the result is the same.
- The new `delete_prim` raises on invalid paths: the instancer now records the actual created path and skips invalid
  ones.
- `RobotRigidGroup` now raises if the base link prim does not exist (the old `SingleXFormPrim` silently created an empty
  Xform). All robot USDs used in `cfg/` have a `base_link`.
- Not covered by the 4 scenarios: the husky sun-direction update (only used with Yamcs + husky) and SDG labels.

Not affected: OmniLRS does not use the Euler helpers whose input order changed in 6.0
(`euler_angles_to_quaternion` / `euler_angles_to_rotation_matrix` now expect XYZ), nor the removed residual-reporting
APIs.

## Validation for any of the above

Run the 4 canonical scenarios (`artefacts.yaml`): ROS2 lunalab, ROS2 largescale, Yamcs lunalab, Yamcs largescale. For
camera or semantics changes, also run SDG and compare images / labels with the current output.
