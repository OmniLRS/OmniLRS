# TODO: Pragyaan physics, scale and frames

Branch `fix/pragyan_physics`, from `main` at `916b0dc`. Audit date: 2026-11-12. Status: baseline recorded, Phase 1 done (not committed). Phase 2 is next.

## How the audit worked

Python scripts with `pxr` read `assets/USD_Assets/robots/pragyaan/nograph_pragyaan.usd`. A headless Isaac Sim probe loaded the rover on flat ground with lunar gravity (1.62 m/s²). The probe read the masses that PhysX computes and logged the pose for 10 s. The probe does not need a Yamcs server.

The standard run (`pixi run -e yamcs yamcs`) loads the robot only after it connects to the Yamcs server on `localhost:8090`.

## Results of the Yamcs sanity scenario

Each run is `pixi run -e test-sim test-yamcs` on 2026-11-12 (about 3 min). The scenario starts the Yamcs server and the simulator. The values come from `test_integrated_solar_panel_drive_turn_imu` (Phase 7) and from the Kit log.

| Measurement | Baseline (source asset) | After Phase 1 | Target |
| --- | --- | --- | --- |
| Sanity tests | 22 passed, 1 failed | 22 passed, 1 failed | all pass |
| Inertia warnings in the Kit log | 2 | 0 | 0 |
| PhysX total mass (probe) | 675.5 kg | 26.0 kg | 26 kg |
| Solar panel deploy and stow | ok | ok | ok |
| Straight drive distance, command 2.0 m | 3.186 m | 2.927 m | 2.0 m ± 10 % |
| Straight drive direction error | -0.4° | -0.1° | 5° or less |
| Heading change in the straight drive | 0.1° | 0.2° | 3° or less |
| Turn angle, command 90° to the left | 102.9° | 111.0° | 90° ± 5° |
| Position change in the turn | 0.946 m | 0.757 m | 0.2 m or less |
| Gyroscope gz, command 8°/s | 0.136 (rad/s, 7.8°/s) | 0.129 (rad/s, 7.4°/s) | 8°/s ± 20 % |
| Largest IMU yaw error | 0.4° | 0.4° | 2° or less |
| Largest IMU roll or pitch error | 3.6° | 3.4° | 2° or less |

The remaining failures have known causes: the drive factors (finding 21), the IMU conversion (finding 25) and the gyroscope unit (finding 26).

## Findings

### Mass and inertia

1. No rigid body authors a mass or a density. PhysX applies its default density of 1000 kg/m³ to the collider volumes. The simulated rover weighs 675 kg. The real rover weighs 26 kg.

   | Rigid body | PhysX mass now (kg) | Proposed (kg) |
   | --- | --- | --- |
   | `Body` | 452.8 | 16.99 |
   | `solarPanel` | 98.4 | 1.5 |
   | 6 × `Wheel_*` | 6 × 16.8 | 6 × 0.75 |
   | 2 × `rocker_*` | 2 × 6.9 | 2 × 0.9 |
   | 2 × `bogie_*` | 2 × 4.3 | 2 × 0.6 |
   | `base_link` | 1.0 (fallback) | 0.01 |
   | Total | 675.5 | 26.0 |

2. `base_link` is a rigid body with no collider and no mass. PhysX logs this warning and uses a 1 kg fallback:

   ```
   The rigid body at /Robots/pragyaan/base_link has a possibly invalid inertia tensor of {1.0, 1.0, 1.0} and a negative mass, small sphere approximated inertia was used. Either specify correct values in the mass properties, or add collider(s) to any shape(s) that you wish to automatically compute mass properties for.
   ```

3. The solar panel colliders are disabled (`physics:collisionEnabled = False`). PhysX still uses them for the mass, so the panel weighs 98 kg.
4. The probe applied the proposed split as stage overrides. The total was 26.00 kg, the warning was gone, and the rover settled at the same height.

### Scale

5. The rover is about two times larger than the published size. The ratio is different on each axis, so one uniform factor cannot match all three published values.

   | Dimension | Model (m) | Published (m) | Ratio |
   | --- | --- | --- | --- |
   | Length, without solar panel | 1.74 | 0.917 | 1.9 |
   | Width | 1.79 | 0.750 | 2.4 |
   | Height, without solar panel | 0.70 | 0.397 | 1.75 |

   Other model values: wheel diameter 0.369 m, wheel width 0.165 m, track width 1.67 m (wheel centers), wheelbase 1.36 m, chassis 1.40 × 1.08 × 0.39 m.

6. The Vikram lander (`assets/USD_Assets/common/lander/Vikram/Vikram.usd`) is also too large. Its bounding box is 4.4 × 3.7 × 3.5 m. The published size is 2.0 × 2.0 × 1.166 m. Its stage is Y-up.
7. The Blender export left many scales in the hierarchy:
   - 27 of 39 meshes have a world scale that is not 1, from 0.005 to 1.86.
   - 5 meshes have a negative (mirrored) scale.
   - The 6 wheel rigid bodies have a scale of (±186, ±115, ±115). The `solarPanel` rigid body has (117, 120, 144).
   - The cameras inherit a world scale of 100 (`depth_cam`) and 138 (`NavCam_left`, `NavCam_right`).
8. `rocker_bogie.usd` and `motor_black.usd` declare centimeters (`metersPerUnit = 0.01`), but their values are in meters. The main file references `rocker_bogie.usd` without a scale correction. A tool that applies the units metadata will make these parts 100 times smaller.
9. The file also holds a stray `/FlatGrid` with a `PhysicsScene` and a reference to an Isaac Sim 5.0 asset on S3. OmniLRS does not load `/FlatGrid`, because a reference loads only the default prim. Each standalone open of the file tries to fetch the S3 asset.

### Reference frames

10. The root `/Pragyaan` turns the model 90° about X (Y-up to Z-up). `Body`, `Mobility` and `base_link` keep the Y-up frame. `rocker_bogie` adds `xformOp:rotateX:unitsResolve = -90`. The forward direction of the rover is -Y in the asset frame.
11. The base_link `FixedJoint` has `localRot0 = (0.5, 0.5, 0.5, 0.5)`, a 120° rotation. The authored base_link pose does not include this rotation. PhysX turns base_link and its IMU on the first step. After that step, base_link has x forward, y left and z up (REP-103). The other 11 joints match their authored rest pose.
12. The rocker and bogie origins are at the group origin, not on their pivot axis. Each wheel has a different spin angle at rest, between -79° and +122°.
13. All wheel joint axes point to +X in the asset frame. A positive wheel velocity drives the rover forward on both sides. The CI tests depend on this sign.
14. The rocker joints use the collider mesh `Body/body_top` as `body0`, not the rigid body `Body`. PhysX accepts this.

### Joints and contacts

15. Rocker joints have `physxJoint:jointFriction = 1000`. Bogie joints have 100. These values can lock the suspension. The flat-ground probe cannot show this.
16. The rocker, bogie and solar panel joints have no limits.
17. The wheel drives are velocity drives with damping 1000, stiffness 0 and no `maxForce` (unlimited torque). The right wheels do not author `stiffness`. These gains were tuned on the 675 kg rover.
18. The rocker drives are springs to 0° (stiffness 100, damping 10). They replace the differential bar.
19. Neither the rover nor the terrain has a physics material. Contacts use the PhysX default: friction 0.5, restitution 0.
20. The articulation root has no `PhysxArticulationAPI` attributes. Self-collision and solver iterations use the defaults.

### Code and configuration that depend on the size

21. `src/tmtc/drive_handler.py` multiplies the commanded speed and distance by 10. It then sends the speed to the wheel joints as an angular velocity. Turns use `dimensions.width` (1.6 m) and `turn_speed_coef: 19`. No code uses the wheel radius, so a new scale changes the ground speed. The factor 10 equals 1 / wheel radius for a wheel of 0.1 m. The wheel at real scale has a radius of 0.097 m (0.1845 × 0.527).
22. `cfg/robot/husky.yaml` has a copy of the same `dimensions` values.
23. `cfg/robot/pragyaan.yaml` sets `height_above_terrain_override: 0.8` and `orientation: [0., 0., 0.70711, 0.70711]` for the current root frame. The spawn point is about 6 m from the lander.
24. `test/ci_artefacts/test_yamcs_sanity.py` asserts the sign of each wheel encoder change: all forward for `drive_straight`, left backward and right forward for `drive_turn`.
25. The IMU orientation equals the base_link orientation, but `get_imu_readings` in `src/robots/robot.py` converts it wrongly. `transform_orientation_from_xyzw_into_xyz` uses scipy `as_euler("zyx")`, which gives rotations about the world axes. The code then negates roll and pitch. Yaw is correct. When the rover does not face along the X axis, roll and pitch are wrong. The baseline telemetry equals this calculation to 0.01°. The fix is the intrinsic order `as_euler("ZYX")` (yaw, pitch, roll about the rover axes), without the negation.
26. `/Rover/imu_gyroscope` sends rad/s (the Isaac Sim IMU unit), but the MDB declares deg/s. The baseline turn sent gz = 0.136, which is 7.8°/s for a command of 8°/s.

## Decisions for review

Decided on 2026-11-12.

- [x] D1. Reference size: one uniform factor of 0.527, which matches the published length of 0.917 m. The rover becomes 0.917 m long, 0.94 m wide and 0.37 m high.
- [x] D2. Mass split: the proposed split in the table, 26.0 kg in total.
- [x] D3. Lander: scale Vikram in this branch, in a separate commit.
- [x] D4. base_link origin: keep it near the chassis center, as today. Only the scale changes it.
- [x] D5. Method: a Python rebuild script that reads a source copy of the current USD.
- [x] D6. Unused files: delete `pragyaan.usd`. Keep `chandrayaan_3_rover.usdz` as the original model.
- [x] D7. The audit and probe scripts stay out of the repository. Only the rebuild script goes in.

## Todo

### Phase 1: mass properties, no geometry change

This phase can ship alone. It removes the warning and gives a 26 kg rover.

- [x] Add `scripts/assets/rebuild_pragyaan.py`. The script reads a source copy of the current USD and writes `nograph_pragyaan.usd`.
- [x] Keep the current file as the source copy `nograph_pragyaan_source.usd`, so that each run of the script starts from the same input.
- [x] Author `physics:mass` on the 13 rigid bodies (D2). Do not author inertia or center of mass on bodies with colliders. PhysX computes them from the colliders and the mass.
- [x] Author on `base_link`: mass 0.01 kg, `diagonalInertia` 1e-6 kg·m² on each axis, `centerOfMass` (0, 0, 0), `principalAxes` identity.
- [x] Run the probe. The total mass is 26.00 kg and the log has no inertia warning.
- [x] Run the Yamcs sanity scenario. Results are in the table above.

### Phase 2: clean structure, same size

- [ ] Flatten `rocker_bogie.usd` and `motor_black.usd` into the output file. Keep the two sub-files, because the source copy references them.
- [ ] Remove `/FlatGrid`.
- [ ] Set the root frame to REP-103: x forward, y left, z up. Keep the origin near the chassis center (D4).
- [ ] Delete `pragyaan.usd` (D6).
- [ ] Give each rigid body a scale of 1. Bake the scales into the mesh points below it.
- [ ] Remove the negative scales. Reverse the face order of each mesh that had a mirror, so that its normals stay outward.
- [ ] Put the rocker and bogie origins on their pivot axis.
- [ ] Put each wheel origin on its axle center, with a spin angle of 0.
- [ ] Calculate each joint `localPos0`, `localPos1`, `localRot0` and `localRot1` again from the new body frames.
- [ ] Keep the joint axis directions, so that a positive wheel velocity still drives forward on both sides.
- [ ] Author the base_link pose that PhysX gives it now. The `FixedJoint` rotation then becomes the identity.
- [ ] Give the cameras and the IMU a world scale of 1. Keep their world pose, focal length and aperture.
- [ ] Set `body0` of the rocker joints to `Body`.
- [ ] Apply `MaterialBindingAPI` to the solar panel meshes. This removes the material binding warnings.
- [ ] Keep each prim path and name that the configuration uses:
  - `base_link` and `base_link/Imu_Sensor`
  - `Body/body_main/depth_cam`, `Body/NavCam_left/NavCam_left`, `Body/NavCam_right/NavCam_right`
  - `Mobility/Wheel_*` (target links)
  - joints `wheel_fl`, `wheel_ml`, `wheel_bl`, `wheel_fr`, `wheel_mr`, `wheel_br` and `solarJoint`
- [ ] Run the probe. Make sure that the masses and the world rest pose did not change, and that no joint turns a body at start.

### Phase 3: scale

- [ ] Apply the factor 0.527 (D1) to the mesh points, body positions, joint positions and camera positions.
- [ ] Author the masses again (mass only). PhysX then computes the inertia from the new colliders.
- [ ] In a separate commit, scale the Vikram lander to the published 2.0 × 2.0 × 1.166 m (D3).
- [ ] Convert the lander stage to Z-up.
- [ ] Keep the lander camera pose and intrinsics consistent with the new lander size.

### Phase 4: code and configuration

- [ ] In `src/tmtc/drive_handler.py`, replace the factor 10 and `turn_speed_coef` with wheel kinematics:
  - straight: wheel speed (rad/s) = linear speed / wheel radius
  - turn: wheel speed (rad/s) = yaw rate × track width / 2 / wheel radius
- [ ] Add `wheel_radius` and `track_width` to `src/configurations/robot_confs.py`, `cfg/robot/pragyaan.yaml` and `cfg/robot/husky.yaml`.
- [ ] If no other code uses `dimensions` and `turn_speed_coef`, remove them.
- [ ] In `cfg/robot/pragyaan.yaml`, set the spawn height, the spawn orientation for the new root frame, and a spawn point near the lander.
- [ ] Update the lander pose and height in the environment files (D3).
- [ ] In `get_imu_readings`, convert the IMU orientation with the intrinsic order `as_euler("ZYX")` and remove the roll and pitch negation (finding 25).
- [ ] Convert the gyroscope values from rad/s to deg/s before the transmitter sends them, as the MDB declares (finding 26).
- [ ] Find the other users of `transform_orientation_from_xyzw_into_xyz` before any change to it.

### Phase 5: physics tuning

- [ ] Tune the wheel drives for 26 kg: damping, and a `maxForce` from the motor torque.
- [ ] Tune the rocker springs and the solar panel drive for the new masses.
- [ ] Decrease the rocker and bogie `jointFriction` (1000 and 100). Test on rough terrain.
- [ ] Add joint limits to the rockers, the bogies and the solar panel, with values from the rover design.
- [ ] Test a mimic joint (`PhysxMimicJointAPI`) between the two rockers, in place of the two springs.
- [ ] Add a physics material to the wheels. Decide the terrain friction.
- [ ] Decide whether the solar panel collides. Its colliders are disabled now.
- [ ] Set the `PhysxArticulationAPI` attributes on the root: self-collision and solver iterations.

### Phase 6: tests

- [ ] Run the audit (outside the repository, D7). Make sure that no rigid body has a scale other than 1, no mesh has a negative scale, and each joint matches its rest pose.
- [ ] Run the probe. Make sure that the mass is 26 kg, the log has no physics warning, and the rest pose is stable.
- [ ] Start the Yamcs server with `./mvnw yamcs:run` in `yamcs-mission-control-pragyaan/yamcs-server`.
- [ ] Run `pixi run -e yamcs yamcs`. Make sure that the Kit log has no inertia warning.
- [ ] Run `pixi run test-yamcs` and `pixi run test-yamcs-largescale`.
- [ ] Make sure that the NavCam and depth images show the expected view. The camera height changes with the scale.

### Phase 7: integrated test in Yamcs mode

Done before Phase 1, so that the baseline uses the source asset. The test is `test_integrated_solar_panel_drive_turn_imu` in `test/ci_artefacts/test_yamcs_sanity.py`. It runs in the `test_yamcs_sanity` scenario of `artefacts.yaml`, after the joint tests and before `test_no_ignored_joint_writes`. It uses the `rover` fixture, so the motor controller is on and the rover is GO.

The test reads this telemetry, which the rover sends every 1 s:

- `/Rover/pose_ground_truth`: base_link position relative to the lander, and orientation as a quaternion (w, x, y, z).
- `/Rover/imu_orientation`: roll, pitch and yaw in degrees.
- `/Rover/imu_gyroscope`: angular velocity gx, gy and gz in deg/s (MDB unit).
- `/Rover/motor_encoder`: wheel angles. When all wheels are still between two samples, the motion is complete.
- `/Rover/solar_panel_state`: `DEPLOYED` or `STOWED`.

The test computes roll, pitch and yaw from the pose quaternion in the intrinsic order yaw, pitch, roll (REP-103). It compares the IMU with the pose sample of the same time (0.1 s or less apart).

- [x] Send `/Rover/system/deploy_solar_panel` with `deployment=DEPLOY`. Wait for `DEPLOYED`.
- [x] At rest, make sure that the IMU roll, pitch and yaw match the pose within 2°.
- [x] Send `/Rover/motor/drive_straight` with `linear_velocity=0.2` and `distance=2.0`.
- [x] Record the pose and the IMU until the wheels stop, then wait 2 s.
- [x] Make sure that the horizontal travel distance is 2.0 m ± 10 %.
- [x] Make sure that the travel direction matches the start heading (base_link x axis) within 5°.
- [x] Make sure that the pose yaw changed by 3° or less.
- [x] During and after the drive, make sure that the IMU matches the pose within 2°.
- [x] Send `/Rover/motor/drive_turn` with `angular_velocity=8.0` and `angle=90.0`.
- [x] Make sure that the median gyroscope yaw rate (gz) is 8°/s ± 20 %, without the first and last 2 samples.
- [x] Make sure that the pose yaw increased by 90° ± 5°. A positive `angular_velocity` turns left (counterclockwise from above).
- [x] Make sure that the position moved 0.2 m or less during the turn.
- [x] During and after the turn, make sure that the IMU matches the pose within 2°.
- [x] Send `deployment=STOW`. Wait for `STOWED`, so that the next tests start from the stowed state.
- [x] Put each tolerance in a named constant above the test.
- [x] Print each measurement to the test log. Fail at the end with the list of failed checks.
- [x] Record the baseline on the source asset.
- [ ] After Phases 2 to 5, make sure that the test passes.
- [ ] Ask before each run. The scenario takes about 3 min.

### Phase 8: documentation

- [ ] Add a `CHANGELOG.md` entry: mass, scale, frames, drive kinematics and the new pose origin.
- [ ] Update the wiki: rover dimensions, base_link frame and origin, mass per part.
- [ ] Close the item `Pragyaan "invalid inertia tensor … negative mass" on base_link` in `omnilrs-ci-build/docs/todo_minor_naming.md`.

## Risks

- The USD is a binary LFS file, so its diff cannot be read. The rebuild script is the part to review.
- When the base_link origin or the scale changes, the Yamcs pose telemetry changes value.
- A new root frame changes the spawn `orientation` in the configuration.
- If a wheel axis changes direction, the tests in `test/ci_artefacts/` fail.
