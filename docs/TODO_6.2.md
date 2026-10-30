# Isaac Sim 6.2: deprecated APIs in OmniLRS

Follow-up to the 6.2 migration ([migration_plan_6.2.md](migration_plan_6.2.md)), which was done "compat-first" on the
APIs Isaac Sim 6.0 deprecated (`isaacsim.core.api`, `isaacsim.core.prims`, `isaacsim.core.utils`,
`isaacsim.sensors.camera`, shipped under `isaacsim/extsDeprecated/`). Sources: 6.0 release notes and the 6.0 migration
guide "Core API to Core Experimental API".

Status: OmniLRS no longer imports any deprecated Isaac Sim extension. Group B landed in #164, group A in #165. The
replacements are the `*.experimental.*` modules and `isaacsim.core.simulation_manager`; their APIs may still change
between releases, so check the release notes of each Isaac Sim upgrade for them.
`src/environments_wrappers/__init__.py` (`REQUIRED_EXTENSIONS`) enables the extensions OmniLRS imports. The deprecated
extensions still load at runtime because Isaac Sim's own default app does: `apps/isaacsim.exp.base.kit` enables
`isaacsim.sensors.camera`, and default extensions such as `isaacsim.robot.wheeled_robots`, `isaacsim.sensors.physx`
and `isaacsim.sensors.rtx` depend on `isaacsim.core.api`.

Earlier: IMU (`isaacsim.sensors.physics` -> `isaacsim.sensors.experimental.physics.IMUSensor`, forced because the old
import is broken).

Validation of each step: old vs new API side by side in one process (identical images, joint data, poses, velocities,
contact forces, physics scene and loop settings), then the 4 canonical scenarios. Not covered: SDG and Zenoh modes,
the deformable-terrain configs (terramechanics / deformation read the wheel-link data, see A4), husky in Yamcs mode.

## A. World / Articulation / Camera (done, #165)

### A1. `World` and `PhysicsContext` -> `src/physics/simulation.py`

`Simulation` implements the lifecycle OmniLRS used from `World` on top of `SimulationManager`/`PhysxScene`, the stage
utils and the timeline (the migration guide has no single replacement object). `step()` is one app update (Kit's loop in
manual mode steps `rendering_dt`; physics substeps follow from `physics_dt`), `reset()` stops and plays the timeline,
`current_time_step_index` counts physics steps since the last stop. `cfg/physics` is applied by
`Simulation.configure_physics_scene()` (ROS2 mode only, as before; `PhysicsSceneManager` removed). The physics scene
attributes, time codes, loop and physics carb settings are identical to `World` for the ROS2 and Yamcs setups.

### A2. Articulation -> `isaacsim.core.experimental.prims.Articulation`

`ArticulationControl` sets DOF position/velocity targets and efforts (NaN targets skipped, as `ArticulationController`
did); `ArticulationTelemetry` reads DOF positions, velocities, projected joint forces (efforts) and link incoming joint
wrenches. Both report ready only once the physics tensor view exists: without it the experimental setters write USD
drive attributes instead. Robot root pose and teleport use `get/set_world_poses` and `set_velocities`.

### A3. Camera -> `isaacsim.sensors.experimental.rtx.CameraSensor` (`src/sensors/camera.py`)

`RtxCamera` returns numpy frames like the old `Camera` (empty array during warm-up). Config resolutions stay
`[width, height]`; `CameraSensor` takes `(height, width)`. Robot cameras use one render product per resolution with RGBA
and depth (was two). The pragyaan `depth_cam` and the Vikram lander camera now author square-pixel apertures (vertical
15.7162 for 4:3), which the old `Camera` applied at runtime with a warning.

### A4. Wheel-link rigid prims -> `isaacsim.core.experimental.prims.RigidPrim`

`RobotRigidGroup` wraps all target links in one batched `RigidPrim`, created on a stopped simulation (creating it while
playing, before PhysX parsed the contact reporting, fails inside Isaac Sim). It keeps the side effects of the old
`RigidPrim(track_contact_forces=True)`: contact report threshold 0, sleep threshold 0, scene stabilization disabled.

### Behavior changes and findings

- `CameraSensor` disables post anti-aliasing on render products below 300 px (RTX/DLSS can return mis-sized buffers
  there): affects the 320x240 monitoring camera and the husky `low` resolution.
- Found, not changed (pre-existing):
  - Only ROS2 mode applies `cfg/physics`. Yamcs, Zenoh and SDG run with Isaac's defaults: Earth gravity (9.81),
    TGS, MBP.
  - `enable_stabilization: true` in `cfg/physics` has no effect: the wheel-link contact tracking disables
    stabilization (previously inside `RigidPrim(track_contact_forces=True)`, now explicit in `RobotRigidGroup`).
  - Husky camera resolutions are 320x240 (4:3) and 1280x720 (16:9) on one camera prim: square pixels are enforced per
    sensor on the shared prim, so the last created sensor sets the vertical aperture for both.
  - `RobotRigidGroup.get_world_poses()` sets `pose[i, :3, 3] = 1` instead of `pose[i, 3, 3] = 1` (unused).

## B. Others (done, #164)

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
