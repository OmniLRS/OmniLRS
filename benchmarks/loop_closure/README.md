# Square-route image matching

Checks whether a terrain view is mistaken for the starting location. This is
an image-matching test, not SLAM: it does not estimate motion or build a map.

## Run

From the repository root:

```sh
pixi run --frozen -e default python benchmarks/loop_closure/benchmark.py --run baseline
# Change the material or terrain, then repeat:
pixi run --frozen -e default python benchmarks/loop_closure/benchmark.py --run updated
```

The script runs headless in Lunaryard with rock scattering disabled. It uses
the configured terrain material and Pragyaan's `depth_cam` optics and mounting
pose from its USD asset, at the configured low resolution (640 x 480).
The camera follows a 5 x 5 m square from `(18, 18)` m, sampled every 0.5 m.
At each corner it turns left in 15-degree steps. The last image has the same
position and heading as the first.

Only the camera is rendered, not the rover body, to avoid matching fixed parts
of the rover itself. The asset's upright mounting pose is retained and its
horizontal viewing direction is aligned with each leg. Rover-root height uses
Pragyaan's configured terrain clearance. There is no wheel physics, suspension,
terrain-following tilt or continuous driving. The sun stays at its initial
environment pose and automatic exposure is disabled.

Outputs are ignored by Git and saved under `benchmarks/loop_closure/outputs/<run>/`:

- `frame_000.png` onwards: route images.
- `manifest.json`: route, camera poses, configuration, lighting and matching settings.
- `results.json`: start-recognition results for each frame and summary counts.
- `false_match_NNN.png`: each false match, with the query on the left and the
  starting image on the right. Colored lines show every tenth RANSAC inlier.
  The corresponding result includes the image filename.

Existing run folders are not overwritten. To repeat analysis without Isaac Sim:

```sh
pixi run --frozen -e default python benchmarks/loop_closure/benchmark.py --run baseline --analyze-only
```

Analysis also regenerates false-match images and removes images for frames no
longer classified as false matches.

## Matching

Each image is compared with the first image using up to 1,000 SIFT features,
a 0.75 descriptor-ratio test and one match per reference feature. A RANSAC
homography rejects inconsistent matches using a 3-pixel threshold.

The start is recognized when there are at least 20 inliers, at least half the
matches are inliers, and their convex hull covers at least 10% of each image.
The estimated image alignment must also move every image corner by no more than
5% of the image diagonal. This rejects views that match the same patch of ground
but from a visibly different position or heading. These settings are fixed in
the script and saved with the run.

Ground truth is used only to label results, not to decide whether images match.
An accepted match more than 1 m from the initial camera position is counted as a
**false start match**. Matches within 1 m are not counted as false: adjacent
views can legitimately show the same terrain.

## Reading the results

- `false_start_matches`: frames recognized as the start while more than 1 m away.
- `frames_far_from_start`: number of frames eligible for that count.
- `final_return_recognized`: whether the actual return image matches the start.
- `per_frame`: matches, inliers, coverage, alignment error and true distance.

Fewer false matches is better only if the actual return is still recognized.
A featureless or washed-out image may produce no matches anywhere, including
the finish; that is not an improvement. Inspect the saved images and per-frame
results as well as the counts.

Use the same route, camera, environment, lighting and matching settings when
comparing materials. Record the code revision; the manifest does not copy assets.
Repeat small differences because rendering is not bit-for-bit deterministic.

A homography is an approximate check for locally planar terrain, not proof of
being at the same place. Nearby overlapping views can pass even beyond the 1 m
cutoff. Report these as image-matching ambiguities, not confirmed SLAM failures.
This test cannot measure tracking loss, trajectory drift or actual loop closures.
