# Dataset generation (`mode=SDG_Dataset`)

`mode=SDG_Dataset` renders segmentation datasets with a stereo (or mono) camera rig on procedural Lunaryard terrain or on real LOLA terrain (LargeScale), and `scripts/sdg_dataset/` turns the raw capture into training labels. The two halves are independent:

- **Capture** (`python run.py mode=SDG_Dataset ...`) needs Isaac Sim 5.0 and a GPU. It writes a *shard*: RGB, semantic and instance masks, depth, normals, exact poses, and the terrain ground truth (DEM, craters, rocks).
- **Post-processing** (`scripts/sdg_dataset/*.py`) needs only `numpy scipy opencv-python pyyaml`. It reads any shard that follows the [output contract](#output-contract) and writes size classes, traversability, slope/crater masks and train/val splits. A shard from another simulator works as long as it follows the contract.

Contents: [What it produces](#what-it-produces) - [A: capture only](#workflow-a-capture-only-needs-isaac-sim) - [B: post-processing only](#workflow-b-post-processing-only-no-isaac-sim) - [C: end to end](#workflow-c-end-to-end) - [Changing the site](#changing-the-site) - [Quality guards](#quality-guards) - Reproducing the reference dataset - [Known limitations](#known-limitations)

## What it produces

Per frame and per camera. Raw capture is what Workflow A writes; built dataset is what `build.py` writes (files are named `<id>_L` / `<id>_R`, with `<id>` = `s<base_seed:05d>_t<terrain:04d>_f<frame:03d>`, e.g. `s00000_t0003_f012`; a mono rig has only `_L`).

| Output | Raw capture (`<shard>/<data_hash>/<cam>_<annotator>/`) | Built dataset (`OUT/<shard>/`) |
|---|---|---|
| RGB | `<cam>_rgb/<folder>/<n>.png` | `images/<id>_{L,R}.png` |
| Semantic | `<cam>_semantic_segmentation/...png` + `<cam>_semantic_segmentation_id_label/...json` (colour to class) | `labels/<id>_{L,R}_sem.png`, uint8: 0 space, 1 ground, 2 rock_small, 3 rock_large |
| Instance | `<cam>_instance_segmentation/...png` + `_id_label` json (colour to prim path) | folded into `sem` and `meta/<id>.json` (`rocks`, `rocks_measured`) |
| Traversability | not captured | `labels/<id>_{L,R}_trav.png`, uint8: 0 free, 1 caution, 2 hazard |
| Bit mask | not captured | `masks/<id>_{L,R}_bits.png`, uint8 bitfield: 1 slope_caution, 2 slope_hazard, 4 crater, 8 rock_small, 16 rock_large, 32 unlabelled |
| Depth | `<cam>_depth/...npz` (key `depth`, float32 metres, distance to image plane) | `depth/<id>_{L,R}_mm.png`, uint16 millimetres, 0 = no surface (space); saturates at 65.535 m |
| Normals | `<cam>_normals/...npz` (key `normals`, H x W x 3) | `normals/<id>_{L,R}.png`, uint8 `(n+1)/2*255`, same frame as recorded |
| Pose | `<cam>_pose/` (stock pose writer) and `frames[].rig` in `manifest.json` | `meta/<id>.json` (`rig`, per-camera `world_T_cam`, `K`) |
| Sun, render mode, guard outcomes | `frames[].sun`, `.render`, `.guards` in `manifest.json` | `meta/<id>.json` |
| Terrain truth | `terrains/terrain_<k:04d>.npz/.json` | copied to `OUT/<shard>/terrains/` |

Craters and slopes are baked into the terrain mesh and carry no renderer labels. `build.py` recovers them exactly by unprojecting every depth pixel through the recorded pose and looking the world (x, y) up in the terrain's DEM.

Label rules (from `build.py`):

- **Rock size**: each rock blob's height above the local ground is measured from depth plus camera pose (a contact band under the blob within 6 m, projected extent beyond), then compared with `--wheel-clearance-m`. Height >= clearance is `rock_large`, below is `rock_small`. A blob that cannot be measured (depth >= 65 m or no depth) takes the majority class from the generator's rock table (`height_above_ground_m`).
- **Traversability**: hazard (2) = `rock_large` or slope_hazard bit or space; caution (1) = `rock_small` or slope_caution bit or crater bit; free (0) = everything else. Slope is the DEM slope averaged over `--footprint-m`, compared with `--slope-caution-deg` (15) and `--slope-hazard-deg` (25).
- Geometry the renderer left unlabelled (the 2-5 cm pebble layer) becomes ground (1) and gets bit 32, so it is never confused with space.

## Workflow A: capture only (needs Isaac Sim)

Run from the repository root with the Isaac Sim 5.0 Python (the same way you run `run.py` for any other mode).

Lunaryard (procedural 20 m yard, one shard = 10 terrains x 25 frames = 250 frames by default):

```bash
python run.py mode=SDG_Dataset environment=lunaryard_20m4Dataset \
    rendering.renderer.headless=True mode.dataset_settings.base_seed=0
```

LargeScale (real LOLA DEM, default site Site20; one shard = 10 locations x 25 frames):

```bash
python run.py mode=SDG_Dataset environment=largescale4Dataset \
    rendering.renderer.headless=True mode.dataset_settings.base_seed=0
```

Every option lives under `mode.dataset_settings.<key>` (defaults in `cfg/mode/SDG_Dataset.yaml`, validated in `src/configurations/dataset_confs.py`):

| Key | Default | Meaning |
|---|---|---|
| `base_seed` | 0 | Shard id. Terrain `k` uses seed `base_seed * 1000 + k`. One shard per seed: run several seeds for more data. Output goes to `<out_dir>/shard_<base_seed:05d>/` |
| `num_terrains` | 10 | Terrains (Lunaryard) or locations (LargeScale) per shard |
| `frames_per_terrain` | 25 | Frames per terrain/location |
| `out_dir` | `data/sdg_dataset` | Parent of the shard directories |
| `settle_steps` | 4 | Render steps after every re-roll, before recording (flushes temporal AA history) |
| `exit_watchdog_s` | null | If set, force-exit the process this many seconds after the run ends (Isaac's shutdown can hang holding the GPU) |
| `rig.baseline_m`, `rig.hfov_deg` | 0.12, 90.0 | Stereo baseline and horizontal field of view. Placeholders: replace with your camera |
| `rig.fx/fy/cx/cy` | null | Override the pinhole intrinsics in pixels (`fx` overrides `hfov_deg`) |
| `rig.height_m`, `rig.pitch_deg` | [0.35, 1.0], [5.0, 22.0] | Camera height above ground and pitch, sampled uniformly per frame |
| `rig.roll_jitter_deg`, `rig.terrain_margin_m` | 2.0, 1.5 | Roll jitter and Lunaryard edge margin |
| `sun.elevation_buckets`, `sun.intensity_mode`, ... | see yaml | Sun elevation is drawn from weighted buckets (low-angle heavy); `constant_ground_brightness` scales intensity as 1/sin(elevation) |
| `largescale.region_radius_m`, `step_m`, `frame_jitter_m`, `rock_clearance_m` | 150, [40, 80], 8, 0.6 | LargeScale random walk of locations, rig jitter around each location, and clearance kept from rocks when placing the rig |
| `largescale.dem_crop_half_m`, `dem_crop_res_m` | 80, 0.05 | Half-size and resolution of the DEM crop saved per location |
| `guards.*` | all off | See [Quality guards](#quality-guards) |
| `terrain_material` | null | `{texture_scale, bump_factor}` override on the terrain shader |

Camera set-up lives in `mode.generation_settings`: `camera_names` (one name = mono rig; two = stereo), `camera_resolutions` (all cameras must share one resolution), `annotators_list`. The `guards` need `rgb` and `depth` in the first camera's annotators.

Hydra override examples: `mode.dataset_settings.num_terrains=50`, `mode.dataset_settings.rig.baseline_m=0.2`, `mode.dataset_settings.guards.mesh_probe.enabled=true`, `'mode.generation_settings.camera_resolutions=[[1640,1232],[1640,1232]]'`.

Rendering: `rendering=ray_tracing` (RayTracedLighting) is the tested renderer. For path tracing set `mode.dataset_settings.guards.pt_runtime_switch=true` (boots ray tracing and switches the render mode at runtime, because selecting `rendering=path_tracing` at startup crashes headless on Isaac Sim 5.0 in the tested install). The frame record's `render` field is `pt` or `rt`.

Progress is printed with the prefix `[sdg_dataset]`. If the run raises, the manifest is still written, with `partial: true` and an `error` string, containing only the frames recorded so far, and the process exits with code 1.

### Output contract

This is the interface between capture and post-processing: any capture that produces it, from this mode or elsewhere, can be fed to the tools in Workflow B. The Isaac-free reader is `scripts/sdg_dataset/_common.py` (`load_manifest` raises `ContractError` naming the missing file or field).

```
<out_dir>/shard_<base_seed:05d>/
  manifest.json
  <cam>_intrinsics.json                       (one per camera; informational, manifest intrinsics are authoritative)
  terrains/terrain_<k:04d>.npz
  terrains/terrain_<k:04d>.json
  <data_hash>/                                (the writer directory; manifest.data_dir points at it)
    <cam>_rgb/<folder>/<n>.png
    <cam>_depth/<folder>/<n>.npz
    <cam>_normals/<folder>/<n>.npz
    <cam>_semantic_segmentation/<folder>/<n>.png
    <cam>_semantic_segmentation_id_label/<folder>/<n>.json
    <cam>_instance_segmentation/<folder>/<n>.png
    <cam>_instance_segmentation_id_label/<folder>/<n>.json
    <cam>_pose/...
```

File naming: with frame index `i` (`frames[].index`), `<folder>` = `i // 1000` and `<n>` = `i % 1000` zero-padded to 4 digits (`0000`). Only `depth`, `rgb`, semantic and instance are required by `build.py`; normals are optional. `build.py` looks for the writer directory as `<shard>/<basename of data_dir>` first, then `data_dir` as given. Camera names are `cam_left` and `cam_right` (or, if absent, the first two intrinsics keys in sorted order). Left is `_L`, right is `_R`.

**`manifest.json`**

| Field | Type | Required by | Notes |
|---|---|---|---|
| `base_seed` | int | build | shard id |
| `environment` | str | validate | class name, `DatasetLunaryard` or `DatasetLargeScale` (validate uses a 5 cm DEM tolerance for names ending in `Lunaryard`, 0.5 m otherwise) |
| `dataset_settings` | object | - | full resolved `mode.dataset_settings` (the settings that produced the shard) |
| `data_dir` | str | build, validate | writer directory, path as seen by the capture process |
| `intrinsics` | object | build, validate | keyed by camera name, see below |
| `frames` | list | build, validate | one record per recorded frame, see below |
| `partial` | bool | - | true if the run failed or the manifest is a periodic flush (written every 5 terrains and at the end) |
| `error` | str | - | only when the run failed |
| `skipped_locations` | list of int | - | locations the mesh-probe guard skipped |
| `frames_recorded` | int | - | frames actually recorded |

`intrinsics.<cam>`: `K` (3x3 pinhole matrix in pixels, required), `width`, `height` (required), `rig_offset_y_m` (camera offset along the rig y axis, +baseline/2 for left, -baseline/2 for right, 0 for mono; required), `baseline_m` (used for `meta.baseline_m`, default 0), `hfov_deg`, `usd` (informational).

`frames[]`:

| Field | Type | Notes |
|---|---|---|
| `index` | int | global frame index, also the file index on disk |
| `terrain_index` | int | `k`, selects `terrains/terrain_<k:04d>.*` |
| `terrain_seed` | int | `base_seed * 1000 + k`, used for splitting |
| `frame_in_terrain` | int | 0-based |
| `render` | str | `rt` or `pt` |
| `sun` | object | `elevation_deg`, `azimuth_deg`, `intensity`, `temperature_k` |
| `rig` | object | `position` [x, y, z] metres (world/local frame, z up), `quat_xyzw` (rig orientation), `height_above_ground_m`, `yaw_deg`, `pitch_deg`, `roll_deg`; LargeScale adds `placement_attempts` |
| `guards` | object | present in this mode's output; keys only appear when the guard produced them: `probe_err_m`, `probe_remedy` (mesh_probe), `lit_fraction`, `dark_retries` (dark_frame), `terrain_mean_gray`, `ae_gain` (auto_exposure). `build.py` also accepts `lit_fraction` and `dark_retries` at the top level of a frame |

Camera pose convention: the camera looks along the rig +X axis; the world-from-camera transform is `R_rig * R_cam` with `R_cam` the quaternion `(0.5, -0.5, -0.5, 0.5)` (xyzw), translation `position + R_rig * [0, rig_offset_y_m, 0]`. Camera space is the USD convention (looks down -Z, +Y up) and `depth` is the z distance (distance to image plane). World units are metres, z up.

**`terrains/terrain_<k:04d>.npz`**

| Key | Lunaryard | LargeScale |
|---|---|---|
| `dem` | float32 [H, W], the whole yard at `grid_m` (default 0.025 m) | float32 [H, W], crop around the location (default 160 m square at 0.05 m) |
| `mask` | uint8, yard mask (not read by the tools) | - |
| `origin_xy`, `grid` | - | origin (metres, local frame) and grid size; informational copies of the json fields |

DEM convention (required): the elevation at world (x, y) is `dem[H-1-round((y-oy)/g), round((x-ox)/g)]`, where `g = grid_m` and `(ox, oy) = origin_xy_m` (0, 0 for Lunaryard). In other words rows run from +y (row 0) to -y, columns from -x to +x. `validate.py` checks this against the depth images and fails if the transposed reading fits better.

**`terrains/terrain_<k:04d>.json`**

| Field | Required | Notes |
|---|---|---|
| `grid_m` | yes | DEM grid size in metres |
| `origin_xy_m` | no (default [0, 0]) | world xy of DEM column 0 / bottom row |
| `crater_xy_frame` | no (default `lunaryard_index`) | `lunaryard_index`: crater `coord_m` are DEM-index metres (axis 0 = row); `local_xy`: crater centres are world xy metres |
| `craters` | yes (may be empty) | Lunaryard: `{coord_m: [a, b], size_px, xy_deformation_factor: [sx, sy], rotation_deg, profile_id}`. LargeScale (`local_xy`): `{xy_local_m: [x, y], radius_m, xy_deformation_factor, rotation_deg, xy_global_m}` |
| `rocks` | yes (may be empty) | one entry per labelled rock prim: `path` (USD prim path; matched by prefix against the instance-segmentation label), `height_above_ground_m`, `footprint_m` (both required); `group`, `prototype`, `position`, `scale`, `aabb_min`, `aabb_max` are informational |
| `terrain_index`, `terrain_seed`, `base_seed`, `dem_shape`, `dem_convention`, `build_seconds`, ... | no | informational. LargeScale also records `lr_dem`, `starting_position_global_m`, `location_local_m`, `location_global_m`, `hr_dem_shape`, `crater_source` |

A crater is masked where the DEM inside its ellipse lies more than 1 cm below the median DEM on its rim ring.

**Frame frame of reference for LargeScale**: coordinates are local metres relative to `starting_position` (the location walk starts at local (0, 0)); `location_global_m` gives the position in the DEM frame (metres from the DEM centre).

## Workflow B: post-processing only (no Isaac Sim)

Requirements: Python 3.10+ and

```bash
pip install numpy scipy opencv-python pyyaml
```

The scripts import each other as `scripts.sdg_dataset`, so keep the folder at `<some_root>/scripts/sdg_dataset/` (a clone of this repository, or just that folder copied). Run them from that root. They import nothing from Isaac Sim, `omni`, `pxr` or `warp`.

Take a shard that follows the [output contract](#output-contract) (say `data/sdg_dataset/shard_00000`) and run, in order:

```bash
SHARD=data/sdg_dataset/shard_00000        # a raw shard
OUT=data/sdg_dataset_built

# 1. Check the shard before spending time on it (writes validation_report.json + contact_sheet.png into the shard, exit 1 on failure)
python scripts/sdg_dataset/validate.py $SHARD

# 2. Build labels, masks, depth, normals and splits (any number of shards)
python scripts/sdg_dataset/build.py --shards $SHARD --out $OUT

# 3. Flag frames whose rocks float or sink into the rendered mesh, or that are dark (writes quality.json + rejected.txt)
python scripts/sdg_dataset/flag_frames.py $OUT/shard_00000

# 4. Rebuild the splits without the flagged frames (no images are rewritten)
python scripts/sdg_dataset/build.py --out $OUT --splits-only --exclude $OUT/*/rejected.txt

# 5. Optional: sensor noise model on the RGB images (labels untouched)
python scripts/sdg_dataset/sensor_model.py --in $OUT/shard_00000/images --out $OUT/shard_00000/images_sensor
```

Several shards: `--shards data/sdg_dataset/shard_0000{0,1,2}`, then run `flag_frames.py` once per built shard directory. `build.py` only fills in what it has (it merges into an existing `index.json`), so you can add shards later; re-run step 4 afterwards.

Built layout (`--out`):

```
OUT/
  shard_00000/{images,labels,masks,depth,normals,meta,terrains}/    files as in "What it produces"
  shard_00000/quality.json, rejected.txt                             from flag_frames.py
  splits/<size>_<split>.txt                                          frame ids, one per line, e.g. S_train.txt, S_val.txt
  index.json                                                         per-frame shard, seeds, dark/far flags
  summary.json                                                       frame counts per size and split, class pixel fractions, missing/excluded
```

Splits: a frame goes to `val` (or `test`) by a hash of its terrain seed, so no terrain is in two splits; the rest is `train`. Frames flagged dark go to `<size>_dark.txt` and frames with too much far terrain to `<size>_far.txt`. Sizes are nested (S in M in L): each takes the first `ceil(target / frames_per_terrain)` terrains in shard order.

`rejected.txt` is one `frame_id<TAB>reason` line per rejected frame; reasons are `mesh_issue` (rendered ground more than 0.3 m from the DEM), `rock_issue` (a rock floats above or is buried in the mesh by more than `--tol`), `dark_near` (less than half of the nearby terrain is lit) and `far_flat` (too much textureless far terrain). `--exclude` reads the first column of any number of such files.

### CLI reference

| Script | Flag (default) |
|---|---|
| `validate.py SHARD` | `--n 12` frames sampled; `--wheel-clearance-m 0.05`; `--footprint-m 0.30` |
| `build.py` | `--shards` (list); `--out` (required); `--wheel-clearance-m 0.05`; `--slope-caution-deg 15`; `--slope-hazard-deg 25`; `--footprint-m 0.30`; `--sizes S=2000,M=6000,L=10000`; `--val-frac 0.15`; `--test-frac 0`; `--exclude` (list of files); `--splits-only`; `--limit 0` (debug: first N frames); `--max-dark 0.9`; `--min-lit 0.7`; `--max-far 1.0`; `--workers 0` (0 = min(16, cpu count)) |
| `flag_frames.py BUILT_SHARD` | `--tol 0.06`; `--max-range 15.0`; `--min-area 12`; `--max-far-flat 0.02`; `--workers 0` |
| `sensor_model.py` | `--in` (required, folder of `*.png`); `--out` (required); `--calib` (JSON, default generic 12-bit CMOS); `--limit 0` |
| `sensor_calibrate.py` | `--darks CSV`, `--flats CSV`, `--out JSON` (all required); `--bit-depth 12`; `--exposure-ref` (default median flat exposure) |

`--wheel-clearance-m` is the height above local ground at which a rock stops being `rock_small` (something the wheels roll over: caution) and becomes `rock_large` (an obstacle: hazard). Set it to your vehicle's ground clearance or wheel radius. It changes the labels, so choose it before building; rebuilding requires re-running step 2.

`sensor_model.py` is deterministic: each frame's seed is a CRC32 of its id, and a stereo pair shares exposure, temperature and blur. Its defaults are generic placeholder CMOS numbers; fit your camera with `sensor_calibrate.py` (dark frames and flat fields listed in two CSV files, see its docstring) and pass `--calib`. It can also be used as a library (`SensorModel.from_json(...)`, `sm(img, seed=...)`).

## Workflow C: end to end

```bash
# capture (Isaac Sim): three shards of the default LargeScale site
for s in 0 1 2; do
  python run.py mode=SDG_Dataset environment=largescale4Dataset rendering=ray_tracing \
      rendering.renderer.headless=True mode.dataset_settings.base_seed=$s
done

# post-process (no Isaac Sim needed from here on)
OUT=data/sdg_dataset_built
for s in 00000 00001 00002; do python scripts/sdg_dataset/validate.py data/sdg_dataset/shard_$s || exit 1; done
python scripts/sdg_dataset/build.py --shards data/sdg_dataset/shard_0000{0,1,2} --out $OUT --wheel-clearance-m 0.05
for s in 00000 00001 00002; do python scripts/sdg_dataset/flag_frames.py $OUT/shard_$s; done
python scripts/sdg_dataset/build.py --out $OUT --splits-only --exclude $OUT/*/rejected.txt
python scripts/sdg_dataset/sensor_model.py --in $OUT/shard_00000/images --out $OUT/shard_00000/images_sensor   # optional
```

Do a two-terrain smoke run first (`mode.dataset_settings.num_terrains=2 mode.dataset_settings.frames_per_terrain=3`) and check `validate.py`'s contact sheet before starting a long run.

## Changing the site

Everything site-specific for LargeScale is the low-resolution DEM plus one starting position. To capture on another lunar site:

1. **Pick a DEM.** Choose a LOLA 5 m/px site from `scripts/dems_list.txt` (Site01, Site04, Site06, Site07, Site11, Site20, Site23, Site42, Haworth, Shoemaker, DM1, DM2, SL2, SL3, NPA-NPD, LM1-LM8, and the 87S mosaic), or append the URL of any metric-projected GeoTIFF (one file per site).
2. **Download and convert it.** `./scripts/get_dems.sh` downloads every URL in `scripts/dems_list.txt` into `tmp/` (8 parallel `wget`s; delete the lines you do not need first, the files are large). Then `./scripts/extract_dems.sh` runs `gdalinfo` and `scripts/process_info.py` / `scripts/preprocess_dem.py` on each `tmp/*.tif` (needs GDAL with its Python bindings) and, after two `y` prompts, writes `assets/Terrains/SouthPole/<name>/dem.npy` and `dem.yaml`, where `<name>` is the GeoTIFF file name without extension (e.g. `Site20_final_adj_5mpp_surf`). `dem.yaml` holds:
   - `size`: `[width, height]` in pixels, as printed by `gdalinfo` (the `.npy` array is indexed `[row, col]` = `[height, width]`)
   - `pixel_size`: metres per pixel (`pixel_size[0]` is used)
   - `center_coordinates`: `[longitude, latitude]` of the DEM centre in degrees
3. **Select it:** `environment.large_scale_terrain.lr_dem_name=<name>` (Hydra override, or edit `cfg/environment/largescale4Dataset.yaml`; the folder is `lr_dem_folder_path`, default `assets/Terrains/SouthPole`).
4. **Choose the starting position.** `environment.large_scale_terrain.starting_position=[x,y]` (quote it in the shell: `'environment.large_scale_terrain.starting_position=[2800,-2200]'`) is in metres **from the DEM centre**: the terrain generator converts a coordinate to a DEM pixel as `pixel = coord / pixel_size + shape // 2`, with `x` along DEM array axis 0 (rows) and `y` along axis 1 (columns) (`querry_low_res_dem` in `src/terrain_management/large_scale_terrain/high_resolution_DEM_generator.py`). The valid range is therefore `+-(size * pixel_size / 2)` minus `largescale.region_radius_m` minus one 50 m block. For the default Site20 (a 16 km square at 5 m/px, so +-8000 m) with `region_radius_m` 150 this is `|x|, |y| <= 8000 - 150 - 50 = 7800`; the shipped default `[2800, -2200]` is well inside. For a non-square DEM apply the row count to `x` and the column count to `y`. To choose a spot, preview the DEM (jet colour map with min/max elevation, plus the centre, pixel size and size from `dem.yaml` when present):

   ```bash
   python scripts/generate_dem_previews.py --dem-path assets/Terrains/SouthPole/<name>/dem.npy --size 2048
   ```

   This writes `preview.png` next to `dem.npy` (other flags: `--root`, `--dem-name`, `--output-name`, `--mode preview|mosaic|preview_statistics`, `--mosaic-output`, `--mosaic-tile-size`, `--statistics-output`; with no arguments it previews every DEM under `assets/Terrains/SouthPole`). Convert a pixel `(row, col)` to metres with `x = (row - height // 2) * pixel_size`, `y = (col - width // 2) * pixel_size`. Prefer terrain that is reasonably flat around the spot: a location walk of radius `region_radius_m` is captured around it.
5. **Keep the walk inside the streamed window.** `mode.dataset_settings.largescale.region_radius_m` (default 150) is the radius, around `starting_position`, in which locations are drawn (each frame adds up to `frame_jitter_m` = 8 m of jitter). `hr_dem_num_blocks: 4` in the environment yaml streams a (2*4+1) x (2*4+1) grid of 50 m blocks, a 450 m window; the default 150 m fits it. If you raise `region_radius_m`, raise `hr_dem_num_blocks` too (the streaming cost grows with it).
6. **Sun and Earth.** `LargeScaleController` reads latitude/longitude from `dem.yaml` (`center_coordinates`) automatically and passes them to the stellar engine. In this mode the sun's elevation, azimuth and intensity are sampled per frame from `mode.dataset_settings.sun`, so the site changes the terrain, not the sun distribution: edit `sun.elevation_buckets` if your site needs a different lighting regime. The crater and rock fields are procedural and seeded, so any site gets them. Note that `mode.dataset_settings.base_seed` changes the walk, rig poses and sun samples; the procedural crater field derives from `environment.seed` (`large_scale_terrain.seed`) and the rock generators have their own fixed seeds in `rock_gen_cfgs`. Change `environment.seed` as well if two shards at the same site should have different craters.
7. **Lunaryard** has no DEM: its "location" is `environment.seed` plus `mode.dataset_settings.base_seed` plus the yard size. Terrain `k` of a shard is a pure function of `base_seed` and `k` (the generators are reseeded per terrain), so a new `base_seed` gives a new yard. For a different yard size, copy `cfg/environment/lunaryard_20m4Dataset.yaml` and change `lunaryard_settings.lab_length` / `lab_width` (the crater distribution and terrain size follow them), then pass `environment=<your copy>`. The rig stays `rig.terrain_margin_m` from the yard edge.

After changing the site, do a two-terrain smoke run and run `validate.py`: it checks the DEM convention and contact sheet on your new terrain.

## Quality guards

All guards default to **off** (`mode.dataset_settings.guards.<name>`). They exist because real relief and streamed terrain produce failure modes that a flat-yard run never sees. The `cfg/mode/SDG_Dataset_moonseg.yaml` preset enables all of them.

| Guard | What it fixes | Cost | Settings (defaults) |
|---|---|---|---|
| `mesh_probe` | LargeScale clipmap sometimes renders the ground 0.3-2 m off the DEM on some frames (rocks then float or sink). Compares the rendered depth with the DEM query (p95 of the height error, up to 30 m range); on failure adds up to 3 settle rounds, then a clipmap re-sample, then re-rolls the rig spot (3 tries). A location is skipped only on frame 0 (`skipped_locations`); a later frame is dropped | extra render steps on bad frames, roughly one extra depth read per frame | `enabled`, `tol_m` 0.3 |
| `dark_frame` | Low sun on real relief leaves the whole view in shadow (black frame). Re-rolls the sun (from `retry_min_elevation_deg` up) and re-renders | up to `retries` extra renders per dark frame | `enabled`, `min_lit_fraction` 0.5, `retries` 3, `retry_min_elevation_deg` 10 |
| `auto_exposure` | The sin(elevation) intensity rule assumes flat ground; sun-facing slopes blow out, shaded ones go murky. Measures the mean gray of lit terrain and rescales the sun once if it is outside the band | one extra settle + measurement when triggered | `enabled`, `min_mean` 70, `max_mean` 185, `target_mean` 125 |
| `hide_far_mesh` | LargeScale only: the coarse 5 m mesh beyond the fine clipmap shows streaks and a flat bright sheet at the horizon. Hides it; beyond the fine mesh the frame shows space (label 0, depth saturated) | none, but no far terrain in the image | bool |
| `pt_runtime_switch` | Path tracing selected at startup crashes headless on Isaac Sim 5.0; boots ray tracing and flips to path tracing at runtime (`rendering.renderer.samples_per_pixel_per_frame`, default 32 spp) | path tracing costs more time per frame than ray tracing | bool |

A guard that cannot run (mesh_probe, dark_frame or auto_exposure without `rgb` and `depth` annotators) raises at startup rather than mid-run. Guard outcomes are recorded per frame in `manifest.json` (`frames[].guards`) and mirrored into `meta/<id>.json`.

## Reproducing stride-moon-seg-v1

The stride-moon-seg-v1 dataset (LargeScale, LOLA Site20) was produced with the `SDG_Dataset_moonseg` preset and these build settings. The dataset itself is not published from this repository.

```bash
python run.py mode=SDG_Dataset_moonseg environment=largescale4Dataset rendering=ray_tracing \
    rendering.renderer.headless=True mode.dataset_settings.base_seed=<seed>
python scripts/sdg_dataset/build.py --shards <shards> --out <out> --wheel-clearance-m 0.05 --val-frac 0.15
```

What the preset changes against `cfg/mode/SDG_Dataset.yaml`:

- resolution 1640 x 1232 per camera (default 1280 x 960)
- 50 terrains (locations) per shard (default 10), 25 frames each
- `exit_watchdog_s: 180` (default null)
- all guards on: `dark_frame`, `auto_exposure`, `mesh_probe` enabled, `hide_far_mesh: true`, `pt_runtime_switch: true`
- `terrain_material: {texture_scale: 0.25, bump_factor: 0.5}` (default null)

Everything else is the default (baseline 0.12 m, hfov 90, `region_radius_m` 150). Note that `--wheel-clearance-m 0.05` is also the `build.py` default; `--val-frac 0.15` is passed explicitly to document the split.

## Known limitations

- **Clipmap coarseness beyond about 40 m.** LargeScale renders the terrain as a geometry clipmap whose texel size grows with distance, while rocks sit on the full-resolution DEM. At 20-40 m, small craters and rims are smoothed, so a rock on a rim can hover and one in a bowl can be buried. `flag_frames.py` finds these (it tests the mesh-versus-DEM error under each rock, within `--max-range` 15 m by default) and lists them in `rejected.txt`; exclude them at split time. Treat far-field masks (crater, slope) beyond about 20 m as approximate; `build.py` only records `far_frac` (share of pixels beyond 150 m) for filtering.
- **Always-on LargeScale workarounds.** `DatasetLargeScale` refreshes the fine clipmap's DEM buffer after every block shift and re-samples the clipmap after each move (the stock update path leaves it stale, offsetting the mesh by metres), and it replaces the rock height sampler so rocks are placed with the same height query as the rig instead of the clipmap kernel. Both are correctness fixes, not options, and can be removed when the upstream `update_visual_mesh` is fixed.
- **Placeholder rig numbers.** `rig.baseline_m` 0.12 and `rig.hfov_deg` 90 are generic defaults, not any specific camera: set them (or `rig.fx/fy/cx/cy`) and the resolution to match your camera. `sensor_default.json` is likewise a generic CMOS model until you run `sensor_calibrate.py`.
- **Isaac Sim 5.0 only.** Capture is developed and tested on Isaac Sim 5.0; Isaac Sim 6 is not supported. Path tracing must go through `pt_runtime_switch`.
- **Semantics.** The mode labels rocks per prim (labelled rock groups only); the 2-5 cm pebble layer stays an unlabelled point instancer and is mapped to ground with bit 32. Every rig camera renders at one resolution. Depth is 16-bit millimetres in the built dataset (saturates at 65.5 m).
- **Terrain reproducibility.** Lunaryard terrain `k` is a pure function of `(base_seed, k)`. LargeScale locations are a function of `base_seed` and `k`, but the streamed procedural terrain also depends on the environment seeds noted in step 6 of "Changing the site".
