# Yamcs ↔ OmniLRS sync

The Yamcs server MDB is the single source of truth for telemetry and commands (TM/TC). OmniLRS defines the robot, including its cameras. The Yamcs side generates its MDB from that definition, using a naming module that lives in OmniLRS. At startup OmniLRS fetches the MDB from the server, checks itself against it, then continues.

## 1. Preparation (before launch)

### OmniLRS defines the robot

`cfg/robot/<robot>.yaml` lists each camera with `name`, `prim_path`, `type` and `resolutions`. `type` is a list of `rgba` and/or `depth`: `rgba` gives the camera streaming and on-command images, `depth` gives it depth images.

| Robot | rgba cameras | depth cameras |
|---|---|---|
| Pragyaan | NavCam_left, NavCam_right | cam_depth |
| Husky | cam_color, cam_left, cam_right | cam_left |

### OmniLRS owns the naming rule

`src/tmtc/yamcs_naming.py` is a new module that uses only the Python standard library:

```python
CAMERA_ROOT = "/Rover/camera"
KINDS_BY_TYPE = {"rgba": ("streaming", "oncommand"), "depth": ("depth",)}
IMAGE_META_FIELDS = ("number", "name", "url_storage", "url_full", "url_full_nginx")
STREAMING_FIELDS = {"pragyaan": ("state", "bandwidth", ...), "husky": ("state",)}
CAMERA_COMMAND_TYPE = {"camera_capture_high": "rgba", "camera_streaming": "rgba",
                       "set_camera_streaming_params": "rgba", "camera_capture_depth": "depth"}
CAMERA_NAME_ARG = "camera_name"

def parse_cameras(camera_conf) -> list[dict]  # validates the robot yaml camera list
def camera_kinds(camera) -> tuple[str, ...]
def camera_system(camera) -> str              # /Rover/camera/<camera>
def meta_path(camera, kind) -> str            # /Rover/camera/<camera>/<kind>
def streaming_param(camera, field) -> str     # /Rover/camera/<camera>/streaming/<field>
def bucket_name(camera, kind) -> str          # images_<camera>_<kind>
def camera_buckets(cameras) -> list[tuple]    # (camera, kind, bucket_name, meta_path) for every bucket
def cameras_for_command(cameras, command) -> list[str]
def streaming_fields(robot) -> tuple[str, ...]
```

Camera telemetry is scoped by camera through nested space systems: one space system per camera, with one child per kind. Example for Pragyaan:

```
/Rover/camera
├── camera_capture_high, camera_streaming, …     commands (take a camera_name argument)
├── NavCam_left
│   ├── streaming     number, name, url_storage, url_full, url_full_nginx,
│   │                 state, bandwidth, packets_sent, packets_lost, decode_errors
│   └── oncommand     number, name, url_storage, url_full, url_full_nginx
├── NavCam_right
│   ├── streaming     (same as NavCam_left)
│   └── oncommand
└── cam_depth
    └── depth         number, name, url_storage, url_full, url_full_nginx
```

Yamcs bucket names can't contain `/`, so buckets stay flat. The camera name comes first so buckets group by camera when listed: `images_NavCam_left_streaming`, `images_NavCam_left_oncommand`, `images_cam_depth_depth`. Nothing ever parses a bucket name back into a camera and kind; `bucket_name()` is only ever called in the forward direction.

The list of streaming telemetry fields is set per robot in `yamcs_naming.STREAMING_FIELDS`:

| Robot | Fields |
|---|---|
| Husky | `state` |
| Pragyaan | `state`, `bandwidth`, `packets_sent`, `packets_lost`, `decode_errors` |

The Yamcs-side generator imports the same constants.

### The Yamcs side generates its MDB from OmniLRS

In `generate_mdb_*.py`:

- It reads `$OMNILRS_ROOT/cfg/robot/<robot>.yaml` directly. There's no copy of the robot yaml in the Yamcs repo.
- It loads `yamcs_naming.py` by file path from `$OMNILRS_ROOT`.
- It creates one space system per camera (`camera_system`) with a child per kind (`meta_path`) holding the image metadata, plus the streaming telemetry in the `streaming` child of each rgba camera.
- For each camera command, the allowed `camera_name` values are only the cameras whose type matches that command.
- It writes only to its own repo. There is no deploy into OmniLRS, no controller-yaml patching and no `yamcs.yaml` bucket list.
- The lander and admin generators need their `parse_and_write` calls fixed.

Yamcs loads the MDB when an instance starts, so Yamcs has to be restarted after regenerating.

## 2. Startup sequence in OmniLRS

1. **Wait for Yamcs** (`Yamcs_SimulationManager._wait_for_yamcs`). Wait until the server answers **and** the instance state is `RUNNING`, so the MDB is loaded.
2. **Fetch the MDB** (`MdbParsingService.load_from_server` in `src/tmtc/mdb_parsing_service.py`). Call `export_space_system(s)` for each entry in `mdb_space_systems` and parse the XTCE the server returns into a `ServerMdb`:
   - `registry`: the command decoding table used by `CommandsHandler` (wire name → command),
   - `parameters` and `space_systems`: every qualified parameter and space system name,
   - `commands`: each command's arguments, including the allowed values (enum labels) of `camera_name`.
3. **Compute what's expected** from the robot yaml (`Robot.cameras`, parsed with `yamcs_naming.parse_cameras`), the controller yaml, `yamcs_naming` and the controller's `COMMAND_ARGS`.
4. **Check ([§3](#3-how-camera-telemetry-and-buckets-get-checked)), then report** (`YamcsTMTC.__init__`, before any handler exists). Print one summary of everything found. Stop if anything is missing; only warn about extras.
5. **Set up buckets** (`ImagesHandler`). Call `get_bucket(name, create=True)` for every computed camera bucket, plus `images.buckets.*` and the APXS bucket. If `clear_buckets_on_startup` is set, delete the old objects. Once every bucket is registered (when the "no data" images are sent), warn about `images_*` buckets on the server that none of them uses ([§8](#8-smaller-choices)).
6. **Continue as today:** register command handlers, send the "no data" placeholder images and start the telemetry intervals.

## 3. How camera telemetry and buckets get checked

OmniLRS builds an expected set and compares it with the MDB. Example for Pragyaan:

| Expected item | Source | Checked against | If missing / extra |
|---|---|---|---|
| `/Rover/camera/cam_depth/depth/{number,name,url_*}` | cam_depth type `depth` | MDB parameters | fail / – |
| `/Rover/camera/NavCam_{left,right}/{streaming,oncommand}/{number,name,url_*}` | NavCams type `rgba` | MDB parameters | fail / – |
| `/Rover/camera/NavCam_{left,right}/streaming/{state,bandwidth,…}` | rgba cams × Pragyaan field list | MDB parameters | fail / – |
| Camera space systems under `/Rover/camera/` for cameras not in the robot yaml, or kinds a camera's `type` doesn't produce | not produced by the robot yaml | MDB space systems | – / warn ("server generated from a different robot config?") |
| `camera_capture_depth.camera_name` ∈ {cam_depth} | depth cameras | command argument values | fail if a camera is missing / warn if extra |
| `camera_capture_high`, `camera_streaming`, `set_camera_streaming_params` ∈ {NavCam_left, NavCam_right} | rgba cameras | command argument values | fail / warn |
| Every `parameters:` and `commands:` path in the controller yaml | hand-written | MDB | fail / – |
| Argument names each handler expects (`action`, `camera_name`, `bitrate`, …) | `COMMAND_ARGS` in `src/mission_specific/<robot>/tmtc/command_spec.py` | command arguments (and `commands:` keys) | fail / – |
| `images.buckets.*.path`, `payload.apxs.bucket.path` (+ their `number`, `name`, `url_*`) | hand-written | MDB space systems and parameters | fail / – |

Buckets aren't compared, because OmniLRS creates them in step 5. The only bucket-related thing that can go wrong is the MDB metadata folder, and that's covered in the table.

The checker (`src/tmtc/yamcs_contract.py`) uses only the standard library and the parsed MDB, so it has no Isaac Sim imports. `COMMAND_ARGS` is the single list of handler arguments: the controller's `setup_command_callbacks` registers each handler with it, and the checker verifies it.

## 4. Running

- `ImagesHandler.save_camera_image(image, camera, kind)`, or `save_image(image, role)` for lander, monitoring and APXS (the payload handler registers its bucket as role `apxs` with `add_bucket(role, bucket_conf)`):
  - encodes the PNG in memory, uploads it to `bucket_name(camera, kind)` (or the role's bucket), and sets the `number`, `name` and `url_*` values under `meta_path(camera, kind)` (or the role's `path`).
  - The URLs are the same as today (`/storage/buckets/<name>/objects/<file>`), so Grafana keeps working.
  - Buckets created through the API get the server's default limits (Yamcs 5.13: 100 MB, 1000 objects), so before each upload the bucket's oldest objects are deleted until the new image fits. A failed upload is logged and skipped; a failing command never stops the TC listener.
- Commanders, the camera handler and transmitters call `yamcs_naming` (or `ImagesHandler`) and never assemble a name themselves.
- Streaming is per camera: one streaming job and one compression model per rgba camera. The streaming-state telemetry checks that camera's own job.
- Turning the camera on or off only starts or stops cameras whose type includes `rgba`.
- `robot.py` only creates the render products listed in each camera's `type`.

## 5. CI

A standalone script runs steps 2–4 against a running Yamcs server for a robot and controller pair, without Isaac Sim (exit code 1 if anything is missing):

```
pixi run -e yamcs check-yamcs-contract
python scripts/check_yamcs_contract.py --robot pragyaan --controller pragyaan-controller [--address localhost:8090 --instance workshop]
```

CI generates the MDB from the OmniLRS commit under test before starting Yamcs: `artefacts.Dockerfile` runs `mdb/generate_mdb_{rover,lander,admin}.py` of yamcs-mission-control-pragyaan with `OMNILRS_ROOT=/workspace/omnilrs`. The sim then runs the same check at startup, and `test/test_yamcs_sanity*.py` checks that the startup "no data" image metadata arrived for every (camera, kind) of the robot yaml.

## 6. Deleted

- **OmniLRS:**
  - `cfg/mdb/`
  - `scripts/fix_yamcs_tmp.sh` and the pixi `fix-tmp` task
  - the `/tmp` writes in `ImagesHandler`
  - the `mdb:` file lists
- **Yamcs repos:**
  - `mdb/cfg/<robot>.yaml` (the robot yaml copy)
  - the deploy paths and `deploy_buckets` logic in `_mdb_cli.py`
  - the generated image bucket list in `yamcs.yaml` (the `stacks` bucket stays)

## 7. Yaml layout

- **Both robot yamls:** a `type` on each camera and a comment pointing to `yamcs_naming.py`.
- **Both controller yamls:**
  - fully hand-written; no generated sections remain
  - `mdb:` file list → `mdb_space_systems: ["/Rover", "/Lander", "/Admin"]` for Pragyaan, `["/Rover"]` for Husky
  - camera buckets and per-camera telemetry removed
  - `images.buckets` changed from a list to entries named by role (`lander`, `monitoring`; empty for Husky)
  - `clear_buckets_on_startup: true` added
  - the `crf` and `seed` nulls restored

## 8. Smaller choices

- Lander and monitoring buckets are named by role, so the code uses `save_image(img, "lander")` and no bucket name is written in Python.
- APXS stays under `payload.apxs.bucket`, to avoid moving unrelated config.
- `CAMERA_ROOT = "/Rover/camera"` is a constant in the module, not yaml. Both robots use it, and the generator needs it too.
- Old images are deleted at startup by default.

- **CI generator source:** use yamcs-mission-control main for CI to generate the MDB, so it always runs against the OmniLRS commit under test.
- **Grafana dashboards:** they refer to camera parameter paths. Disregard for now.
- **Stale-bucket cleanup:** whether to also delete leftover buckets, e.g. `images_cam_depth_streaming` after `cam_depth` became depth-only, or only warn about them. only warn for now.
