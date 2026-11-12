# TODO: Pragyaan physics, scale and frames

Branch `fix/pragyan_physics`, from `main` at `916b0dc`. Audit date: 2026-11-12. Status: Phases 1, 2 and 2b committed. Phase 3 done (not committed). Phase 4 is next.

## How the audit worked

Python scripts with `pxr` read `assets/USD_Assets/robots/pragyaan/nograph_pragyaan.usd`. A headless Isaac Sim probe loaded the rover on flat ground with lunar gravity (1.62 m/s²). The probe read the masses that PhysX computes and logged the pose for 10 s. The probe does not need a Yamcs server.

The standard run (`pixi run -e yamcs yamcs`) loads the robot only after it connects to the Yamcs server on `localhost:8090`.

## Results of the Yamcs sanity scenario

Each run is `pixi run -e test-sim test-yamcs` on 2026-11-12 (about 3 min). The scenario starts the Yamcs server and the simulator. The values come from `test_integrated_solar_panel_drive_turn_imu` (Phase 7) and from the Kit log.

| Measurement | Baseline (source asset) | After Phase 1 | After Phase 2, run 1 | After Phase 2, run 2 | After symmetry | After Phase 3 | Target |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Sanity tests | 22 passed, 1 failed | 22 passed, 1 failed | 22 passed, 1 failed | 22 passed, 1 failed | 22 passed, 1 failed | 22 passed, 1 failed | all pass |
| Inertia warnings in the Kit log | 2 | 0 | 0 | 0 | 0 | 0 | 0 |
| PhysX total mass (probe) | 675.5 kg | 26.0 kg | 26.0 kg | 26.0 kg | 26.0 kg | 26.0 kg | 26 kg |
| Solar panel deploy and stow | ok | ok | ok | ok | ok | ok | ok |
| Straight drive distance, command 2.0 m | 3.186 m | 2.927 m | 2.851 m | 2.898 m | 2.878 m | 1.465 m | 2.0 m ± 10 % |
| Straight drive direction error | -0.4° | -0.1° | -0.2° | 0.0° | -0.4° | -0.1° | 5° or less |
| Heading change in the straight drive | 0.1° | 0.2° | 0.3° | 0.2° | 0.0° | 0.2° | 3° or less |
| Turn angle, command 90° to the left | 102.9° | 111.0° | 125.6° | 104.8° | 97.2° | 96.5° | 90° ± 5° |
| Position change in the turn | 0.946 m | 0.757 m | 0.566 m | 0.877 m | 0.781 m | 0.472 m | 0.2 m or less |
| Gyroscope gz, command 8°/s | 0.136 (rad/s, 7.8°/s) | 0.129 (rad/s, 7.4°/s) | 0.182 (rad/s, 10.4°/s) | 0.168 (rad/s, 9.6°/s) | 0.132 (rad/s, 7.6°/s) | 0.175 (rad/s, 10.0°/s) | 8°/s ± 20 % |
| Largest IMU yaw error | 0.4° | 0.4° | 0.5° | 0.5° | 0.4° | 0.3° | 2° or less |
| Largest IMU roll or pitch error | 3.6° | 3.4° | 4.2° | 4.2° | 3.1° | 6.1° | 2° or less |

The remaining failures have known causes: the drive factors (finding 21), the IMU conversion (finding 25) and the gyroscope unit (finding 26). The two Phase 2 runs differ by 21° in the turn angle, with the same asset. The turn is sensitive to the start state (finding 27). After Phase 3, the straight drive is short because the wheels slip about 24 %: the wheel speed (2 rad/s × 0.097 m) gives 1.94 m in 10 s.

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

5. Fixed in Phase 3. The rover is about two times larger than the published size. The ratio is different on each axis, so one uniform factor cannot match all three published values.

   | Dimension | Model (m) | Published (m) | Ratio |
   | --- | --- | --- | --- |
   | Length, without solar panel | 1.74 | 0.917 | 1.9 |
   | Width | 1.79 | 0.750 | 2.4 |
   | Height, without solar panel | 0.70 | 0.397 | 1.75 |

   Other model values: wheel diameter 0.369 m, wheel width 0.165 m, track width 1.67 m (wheel centers), wheelbase 1.36 m, chassis 1.40 × 1.08 × 0.39 m.

6. Fixed in Phase 3, with a factor of 0.85 (see Phase 3). The Vikram lander (`assets/USD_Assets/common/lander/Vikram/Vikram.usd`) is also too large. Its bounding box is 4.4 × 3.7 × 3.5 m. The published size is 2.0 × 2.0 × 1.166 m. Its stage is Y-up.
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
27. The turn result changes from run to run: 104.8° and 125.6° with the same asset. The drive timer uses simulated time, so the turn duration is the same. The earlier joint tests leave the rover at a slightly different place on the uneven terrain, and skid-steer contacts amplify this.
28. Fixed in Phase 2b. The model is not symmetric. Left and right parts differ by up to 2.6 cm. For example, the rocker pivots are at y = +0.524 m and y = -0.512 m in the Phase 2 frame, before the scale.
29. Fixed in Phase 2b. The rocker, bogie and motor meshes bound materials of the Blender scene, outside the referenced files (`/World/chandrayaan_3_rover/Looks/...`, `/scene/Materials/black`). USD ignored these bindings, so the 10 meshes rendered without material.
30. The static asset loader (`_apply_pose` in `src/environments/static_assets_manager.py`) reads the orientation as x, y, z, w. The environment files say w/x/y/z, and the robot loader reads w, x, y, z. The old lander value [0.5, 0.5, 0.5, 0.5] reads the same in both orders, so this did not matter until Phase 3.

## Decisions for review

Decided on 2026-11-12.

- [x] D1. Reference size: one uniform factor of 0.527, which matches the published length of 0.917 m. The rover becomes 0.917 m long, 0.94 m wide and 0.37 m high.
- [x] D2. Mass split: the proposed split in the table, 26.0 kg in total.
- [x] D3. Lander: scale Vikram in this branch, in a separate commit. The factor is 0.85 (main body to the published size), not 0.527 (see Phase 3).
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

`scripts/assets/rebuild_pragyaan.py` does this phase in step 2, and checks the result against the source at the end of each run.

- [x] Flatten `rocker_bogie.usd` and `motor_black.usd` into the output file. Keep the two sub-files, because the source copy references them.
- [x] Remove `/FlatGrid` and `/Render`. Kit UI flags (`no_delete`, `hide_in_stage_window`) are lost in the flattening.
- [x] Set the root frame to REP-103: x forward, y left, z up. Keep the origin near the chassis center (D4).
- [x] Set the Pragyaan spawn `orientation` in `cfg/robot/pragyaan.yaml` to `[0.70711, 0., 0., 0.70711]` (w, x, y, z). The rover keeps the same world pose.
- [x] Delete `pragyaan.usd` (D6). Keep `chandrayaan_3_rover.usdz`: the textures come from it (`./chandrayaan_3_rover.usdz[0/...]`).
- [x] Give each Xform, rigid body and camera a scale of 1. Bake each mesh transform into its points and normals, so that each mesh has an identity transform.
- [x] Remove the negative scales. The 5 mirrored meshes (3 left wheels, `Cube_077`, `Cube_080`) get a reversed face order, also for their face-varying normals and texture coordinates.
- [x] Align the groups `Mobility` and `rocker_bogie` and each rigid body with the root frame.
- [x] Put the rocker and bogie origins on their pivot axis.
- [x] Put each wheel origin on its axle center, with a spin angle of 0.
- [x] Calculate each joint `localPos0`, `localPos1`, `localRot0` and `localRot1` again from the new body frames.
- [x] Keep the joint axis directions. All wheel axes point to +y (left), so a positive wheel velocity still drives forward.
- [x] Author the base_link pose that PhysX gives it now. The `FixedJoint` frames are now the identity.
- [x] Give the cameras and the IMU a world scale of 1. Keep their world pose, focal length and aperture.
- [x] Set `body0` of the rocker joints to `Body`.
- [x] Apply `MaterialBindingAPI` to each prim with a material binding. This removes the material binding warnings.
- [x] Keep each prim path and name that the configuration uses:
  - `base_link` and `base_link/Imu_Sensor`
  - `Body/body_main/depth_cam`, `Body/NavCam_left/NavCam_left`, `Body/NavCam_right/NavCam_right`
  - `Mobility/Wheel_*` (target links)
  - joints `wheel_fl`, `wheel_ml`, `wheel_bl`, `wheel_fr`, `wheel_mr`, `wheel_br` and `solarJoint`
- [x] Script checks. The mesh points are within 6e-8 m of the source, and the camera poses did not change. The two frames of each joint are equal, and the joint world poses did not change. The masses, the wheel layout and the texture paths are correct.
- [x] Probe against Phase 1 on flat ground: same masses, same principal inertias (within 0.3 %), same settle height (0.4794 m against 0.4795 m), base_link in REP-103 from the start.
- [x] Camera images against Phase 1: mean RGBA difference of 1 to 3 out of 255, median depth difference of 0.2 % or less. The differences come from a 2 cm difference in the settle position.
- [x] Run the Yamcs sanity scenario two times. Results are in the table above.

### Phase 2b: left/right symmetry, same size

`scripts/assets/rebuild_pragyaan.py` does this phase in step 3.

- [x] Find the plane of symmetry. The rocker and bogie meshes are exact mirror copies about y = +5.15 mm (base_link at y = 0). The chassis is centered on this plane within 0.3 mm.
- [x] Move the rover by -5.15 mm in y, so that the plane is y = 0. base_link stays at the root origin, on the plane.
- [x] Give each pair of pivots and axles the average of the two sides, mirrored. The moves are 4.6 mm (rockers), 5.9 mm (bogies) and 2.0 to 2.4 mm (wheels). The largest move across the joint axis was the rocker pivot height (8.2 mm difference between the sides).
- [x] Move each wheel mesh with its axle.
- [x] Replace each left mesh by the mirrored right mesh (8 meshes). This also makes the motors and the wheel spin angles symmetric. The left meshes moved by 2.5 mm or less.
- [x] Use the joint frame of the right side for both sides. The frames of a pair differed only by a rotation about the axle (the old wheel spin angles), which sets where the joint angle starts.
- [x] Bind the 10 rocker, bogie and motor meshes to the `silver_metal` and `black` materials of the asset (finding 29).
- [x] Script checks. The left meshes are the mirrored right meshes within 2.4e-8 m, the paired joints are mirrored, and each mesh has a material.
- [x] Probe on flat ground: same masses and settle height. The rockers settle at 0.00° (before: ±0.05°).
- [x] Run the Yamcs sanity scenario. Results are in the table above.

### Phase 3: scale

`scripts/assets/rebuild_pragyaan.py` step 4 scales the rover. `scripts/assets/rebuild_vikram.py` rebuilds the lander. The helpers that both scripts use are in `scripts/assets/usd_bake.py`.

- [x] Apply the factor 0.527 (D1) to the rover mesh points, body positions, joint positions and camera positions, about the root origin (base_link). The rover is now 0.913 m long, 0.943 m wide and 0.366 m high without the solar panel. The wheels are 0.194 m in diameter, the track width is 0.882 m and the wheelbase is 0.718 m.
- [x] Keep the masses (mass only). PhysX computes the inertia from the smaller colliders: the principal inertias are 0.278 (0.527²) times the Phase 2 values.
- [x] Remove the cooked collision data that the source stored (`physxCookedData`, 33 meshes). It was a cache in the old mesh coordinates.
- [x] Make the left and right rockers, bogies and wheels symmetric (finding 28). Done before the scale, see Phase 2b.
- [x] Set the Pragyaan spawn height to 0.42 m (0.8 m × 0.527). base_link settles 0.252 m above flat ground.
- [x] Keep `dimensions` and `turn_speed_coef` in `cfg/robot/pragyaan.yaml` until Phase 4. The turn rate of the wheels depends on the ratio of wheel radius to track width, which the scale does not change.
- [x] Scale the Vikram lander to the published size (D3), in a separate step. The 4.4 m bounding box of the audit (finding 6) includes the legs and the ramp. The main body (foil box) was 2.35 × 2.40 m and 1.34 m high, because the source already had a 0.5 scale on `/Vikram/Lander`. The factor is 0.85, not 0.527. The main body is now 2.00 × 2.04 m and 1.13 m high (published 2.0 × 2.0 × 1.166 m).
- [x] Keep `Vikram_source.usd` as the unchanged input. `Vikram.usd` is one flattened layer, Z-up, with each mesh transform baked into its points. `/Render` and `/GroundPlane` are removed.
- [x] Set the lander orientation in the three workshop environments to `[0., 0., 0.70711, 0.70711]` (x, y, z, w) for the same world pose. Set the lander height to 0.18 m, so that the footpads keep their 4 cm clearance.
- [x] Keep the lander camera intrinsics. The camera moves with the scale (0.85 about the placement origin) and keeps its view direction. A headless render of the lander camera shows the same view, with the depth median 0.87 times the source value.
- [x] Run the probe: base_link settles at 0.252 m, the wheel axles at 0.097 m, and the masses are 26.0 kg.
- [x] Run the Yamcs sanity scenario. Results are in the table above.

### Phase 4: code and configuration

- [ ] In `src/tmtc/drive_handler.py`, replace the factor 10 and `turn_speed_coef` with wheel kinematics:
  - straight: wheel speed (rad/s) = linear speed / wheel radius
  - turn: wheel speed (rad/s) = yaw rate × track width / 2 / wheel radius
- [ ] Add `wheel_radius` and `track_width` to `src/configurations/robot_confs.py`, `cfg/robot/pragyaan.yaml` and `cfg/robot/husky.yaml`.
- [ ] If no other code uses `dimensions` and `turn_speed_coef`, remove them.
- [ ] In `cfg/robot/pragyaan.yaml`, set the spawn height, the spawn orientation for the new root frame, and a spawn point near the lander.
- [x] Update the lander pose and height in the environment files (D3). Done in Phase 3.
- [ ] Use one quaternion order in the static asset loader, the robot loader and the environment files (finding 30).
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
- [ ] After Phases 4 and 5, run the scenario 3 times and record the spread of the turn angle. Set the turn tolerance from this spread (finding 27).
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
