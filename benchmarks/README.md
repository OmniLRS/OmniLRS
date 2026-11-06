# Terrain benchmarks

Two headless Lunaryard benchmarks for checking how a terrain or material change
affects visual matching. Run a benchmark before and after the change, then
compare the results.

```sh
pixi run python benchmarks/pnp.py --run baseline
pixi run python benchmarks/square_route.py --run baseline
```

Both scripts:

- Use the terrain and material from the selected Lunaryard config
  (`--environment`, default `lunaryard_40m_workshop`). Rock scattering must be disabled.
- Disable static assets and monitoring cameras.
- Keep the sun at the environment's initial pose and disable automatic exposure.
- Save results in `benchmarks/outputs/<benchmark>/<run>/`, which is ignored by Git.
  Existing runs are never overwritten.

Add `--analyze-only` to analyze saved images again without Isaac Sim. The script
stops if the saved settings differ from its current settings.

## PnP localization: `pnp.py`

A nadir reference image, rendered at 2048 x 2048 pixels from 10 m above `(20, 20)` m,
and its depth map form a 3D map of up to 8,000 SIFT features. The script renders
24 query images at 768 x 768 pixels, 2.5-3.5 m above the terrain:

- 8 nadir views on a 2 m grid.
- 16 seeded random views with any yaw and up to 15 degrees of tilt.

Each query uses up to 400 SIFT features, a 0.75 ratio test and RANSAC PnP
(3-pixel threshold, at least 12 inliers). The result is:

- **correct:** position error at most 10 cm and rotation error at most 2 degrees;
- **wrong:** a pose outside either limit;
- **no pose:** too few matches or inliers.

Outputs: `reference.png`, `reference_depth.npy`, `query_NN.png`, `manifest.json`
and `results.json`.

## Square route: `square_route.py`

Uses the optics and mounting pose of Pragyaan's `depth_cam` from the robot USD,
at 640 x 480 pixels. The camera moves around a 5 x 5 m square starting at
`(18, 18)` m: 0.5 m steps along each side, and 15-degree left turns in place at
each corner. The 65th image has the same pose as the first. Only the camera is
rendered: there is no rover body, wheel physics or terrain tilt.

Each image is compared with the first one using up to 1,000 SIFT features,
a 0.75 ratio test and a RANSAC homography with a 3-pixel threshold. An image is
recognized as the start when all of these hold:

- at least 20 inliers;
- at least half of the matches are inliers;
- inliers cover at least 10% of both images;
- the alignment moves no image corner by more than 5% of the image diagonal.

A recognized image taken more than 1 m from the start is a **false start
match**. Each one is saved as `false_match_NNN.png`, with the query on the left,
the start on the right, and every tenth inlier drawn.

Outputs: `frame_NNN.png`, `false_match_NNN.png`, `manifest.json` and `results.json`.
The main fields of `results.json` are `false_start_matches`,
`final_return_recognized` and `per_frame`.

## Comparing runs

- Change one thing at a time, and keep the config, software and hardware the same.
  Note the code revision with the results.
- Rendering is not bit-for-bit repeatable. Repeat runs when the difference is small.
- Look at the images, not only the counts. A dark or washed-out material can
  produce fewer false matches or more failed poses for the wrong reason.
  Fewer false start matches only counts if the final return is still recognized.
- These are image-matching tests on a single synthetic scene. They do not
  evaluate SLAM or lunar realism, and terrain shape, lighting and contrast also
  affect the results.
