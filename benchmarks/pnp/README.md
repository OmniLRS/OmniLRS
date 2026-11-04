# Terrain PnP benchmark

Measures image-to-map localization on Lunaryard terrain using SIFT and PnP.
Run it before and after a terrain or material change to compare localization
results. Each run uses one configured terrain and material.

For a rover-camera square route without pose estimation, see the
[image-matching benchmark](../loop_closure/README.md).

## Run

From the repository root:

```sh
pixi run --frozen -e default python benchmarks/pnp/benchmark.py --run baseline
# After changing the material or terrain:
pixi run --frozen -e default python benchmarks/pnp/benchmark.py --run updated
```

The script launches Isaac Sim headless, without a rover, ROS or Yamcs.
The default environment is `lunaryard_40m_workshop`; `--environment` selects
another Lunaryard config. LargeScale is not supported by this benchmark.
Static assets and monitoring cameras are disabled. Disable rock scattering in
the selected config before running.

### Outputs

Results are saved in `benchmarks/pnp/outputs/<run>/` and ignored by Git:

- `reference.png` and `reference_depth.npy`: the reference map.
- `query_00.png` through `query_23.png`: query images.
- `manifest.json`: resolved configuration, protocol settings, sun transform,
  intensity, angular size, automatic-exposure state and true camera poses.
- `results.json`: aggregate counts and per-query errors, matches and inliers.

Use a new run name for each capture; existing folders are not overwritten,
including failed runs.

To repeat analysis without launching Isaac Sim:

```sh
pixi run --frozen -e default python benchmarks/pnp/benchmark.py --run baseline --analyze-only
```

This rewrites only `results.json`. The script checks that its protocol matches
the saved settings. Use the original script revision to analyze older runs if
the protocol has changed.

## How it works

1. Render a 2048 x 2048 nadir reference at 10 m above the terrain centre
   `(20, 20)` m, covering approximately 10 x 10 m.
2. Extract up to 8,000 SIFT reference features. Back-project their pixels using
   rendered axial depth and camera intrinsics to build a 3D feature map.
3. Render 24 query images at 768 x 768: eight nadir poses on a 2 m grid and
   sixteen seeded poses with yaw and up to 15 degrees of tilt. Query heights are
   2.5-3.5 m above the terrain.
4. Extract up to 400 SIFT query features, match descriptors against the map, and
   apply a 0.75 nearest-neighbour ratio test. Keep one match per map point.
5. Estimate the pose with RANSAC PnP: 2,000 iterations, 3-pixel reprojection
   threshold, 0.999 confidence, and at least 12 inliers. Refine with LM.
6. Score each estimated pose against the saved camera pose. Before analysis,
   check the solver using synthetic correspondences with 0.5-pixel image noise.

### Lighting and coverage

The sun stays at the environment's initial pose, including the stellar-engine
date and location when enabled. Automatic exposure is disabled; other exposure
and tone-mapping settings use application defaults. Brightness may therefore
differ from an interactive viewport. No image contrast or material brightness
adjustments are applied.

Camera and solver settings are defined in the script. When selecting another
environment, ensure its terrain covers the reference and query positions.

## Reading the results

- **Correct:** camera-position error <= 10 cm **and** rotation error <= 2 degrees.
- **Wrong:** the solver accepts a pose outside either threshold.
- **No pose:** too few usable matches/inliers, or the solver cannot find a pose.

Compare all three counts. Median errors include correct and wrong poses, but
exclude queries with no pose. Per-query errors use metres and degrees.
A featureless reference produces no-pose results; missing or unreadable files
stop analysis with an error.

## Comparing runs

Keep the environment, poses, lighting, camera, feature budgets, solver settings,
software and hardware unchanged except for the change being evaluated. Record
the code revision and the change with your results: the manifest stores
configuration, not asset or source snapshots. Repeat runs when count differences
are small, since rendering is not bit-for-bit deterministic.

This is a synthetic localization test with an exact depth map and fixed lighting,
not a rover-navigation or lunar-realism test. Geometry, contrast, shading and
texture repetition all affect the result. The feature budgets are benchmark
settings, not a measured rover processing budget.
