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

The robot camera list itself is plain robot config, the same in every mode: `src/configurations/robot_camera_confs.py` (standard library only) parses and validates it into `RobotCameraConf(name, prim_path, type, resolutions)` objects when the robot config is loaded (`RobotParameters.camera`), and provides `cameras_of_type(cameras, type)`. Only Yamcs mode imports Yamcs code: `test/unit/test_import_layering.py` fails if the robot, the configurations, the environments or the ROS2/Zenoh/SDG modes import `src/tmtc` or `src/mission_specific/<robot>/tmtc`.

`src/tmtc/yamcs_naming.py` is a new module that uses only the Python standard library and imports nothing from OmniLRS (it works on the `RobotCameraConf` objects it is given):

```python
CAMERA_ROOT = "/Rover/camera"
KINDS_BY_TYPE = {"rgba": ("streaming", "oncommand"), "depth": ("depth",)}
IMAGE_META_FIELDS = ("number", "name", "url_storage", "url_full", "url_full_nginx")
CAMERA_COMMAND_TYPE = {"camera_capture_high": "rgba", "camera_streaming": "rgba",
                       "set_camera_streaming_params": "rgba", "camera_capture_depth": "depth"}
CAMERA_NAME_ARG = "camera_name"

def camera_kinds(camera) -> tuple[str, ...]
def camera_system(camera) -> str              # /Rover/camera/<camera>
def meta_path(camera, kind) -> str            # /Rover/camera/<camera>/<kind>
def streaming_param(camera, field) -> str     # /Rover/camera/<camera>/streaming/<field>
def bucket_name(camera, kind) -> str          # images_<camera>_<kind>
def camera_buckets(cameras) -> list[tuple]    # (camera, kind, bucket_name, meta_path) for every bucket
def image_buckets(cameras, yamcs_tmtc)        # (bucket_name, meta_path): cameras + controller yaml role/payload buckets
def cameras_for_command(cameras, command) -> list[str]
def check_kind_resolutions(cameras, kind_resolution)  # every camera has the resolution of each of its kinds
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

The streaming telemetry fields are what each robot's transmitter sends, so they are defined next to it, in `STREAMING_FIELDS` of `src/mission_specific/<robot>/tmtc/enums.py` (standard library only), and not keyed by robot name:

| `enums.py` | `STREAMING_FIELDS` |
|---|---|
| `mission_specific/husky/tmtc/` | `state` |
| `mission_specific/pragyaan/tmtc/` | `state`, `bandwidth`, `packets_sent`, `packets_lost`, `decode_errors` |

The controller passes them to the startup check (`YamcsTMTC.CAMERA_STREAMING_FIELDS`). The Yamcs-side generator loads the same `enums.py`, `robot_camera_confs.py` and `yamcs_naming.py` by file path.

### The Yamcs side generates its MDB from OmniLRS

In `generate_mdb_*.py`:

- It reads `$OMNILRS_ROOT/cfg/robot/<robot>.yaml`, `cfg/controller/<controller>.yaml` and the streaming telemetry fields of `src/mission_specific/<robot>/tmtc/enums.py` directly. There's no copy of them in the Yamcs repo. Which files is set by `ROBOT` and `CONTROLLER` in the Yamcs repo's `mdb/_omnilrs.py` (`--robot` / `--controller` to read others); the robot name itself is only in the robot yaml.
- It loads `robot_camera_confs.py` (to parse the cameras) and `yamcs_naming.py` by file path from `$OMNILRS_ROOT`.
- It creates one space system per camera (`camera_system`) with a child per kind (`meta_path`) holding the image metadata, plus the streaming telemetry in the `streaming` child of each rgba camera.
- For each camera command, the allowed `camera_name` values are only the cameras whose type matches that command.
- It writes the image buckets (`yamcs_naming.image_buckets`) into `etc/yamcs.yaml`: filesystem buckets (one directory of PNG files each) with limits sized to keep every image of a mission (100 GiB and 1,000,000 objects per bucket; Yamcs' default, also for buckets created through the API, is 100 MB and 1000 objects). The rest of `yamcs.yaml` is kept as is (round-trip yaml).
- It writes only to its own repo. There is no deploy into OmniLRS and no controller-yaml patching.

Yamcs loads the MDB when an instance starts, so Yamcs has to be restarted after regenerating.

## 2. Startup sequence in OmniLRS

1. **Wait for Yamcs** (`Yamcs_SimulationManager._wait_for_yamcs`). Wait until the server answers **and** the instance state is `RUNNING`, so the MDB is loaded.
2. **Fetch the MDB** (`MdbParsingService.load_from_server` in `src/tmtc/mdb_parsing_service.py`). Call `export_space_system(s)` for each entry in `mdb_space_systems` and parse the XTCE the server returns into a `ServerMdb`:
   - `registry`: the command decoding table used by `CommandsHandler` (wire name → command),
   - `parameters` and `space_systems`: every qualified parameter and space system name,
   - `commands`: each command's arguments, including the allowed values (enum labels) of `camera_name`.
3. **Compute what's expected** from the robot yaml (`Robot.cameras`, parsed with the robot config by `robot_camera_confs.parse_cameras`), the controller yaml, `yamcs_naming` and the controller's `command_handlers()`.
Steps 4–6 run in order in `YamcsTMTC.start()`, which the simulation manager calls once the controller is built:

4. **Check ([§3](#3-how-camera-telemetry-and-buckets-get-checked)), then report** (`check_yamcs_server()`), against the MDB and the server's storage buckets, before any command is registered or any telemetry or image is sent. Print one summary of everything found. Stop if anything is missing; only warn about extras.
5. **Prepare the buckets** (`ImagesHandler.prepare_buckets()`): images are named `<bucket>_<number>.png`. With `images.clear_buckets_on_startup: true` (the default when the key is left out), delete the objects of every bucket OmniLRS uses; numbering restarts at 0000. With `false`, keep them and continue each bucket's numbering after its highest `<bucket>_<number>.png`. OmniLRS never creates or deletes buckets, and never deletes images otherwise.
6. **Continue as today:** register command handlers (`setup_command_callbacks`), send the "no data" placeholder images and start the telemetry intervals (`start_transmitting_data`).

## 3. How camera telemetry and buckets get checked

OmniLRS builds an expected set and compares it with the MDB and the storage buckets of the server. Example for Pragyaan:

| Expected item | Source | Checked against | If missing / extra |
|---|---|---|---|
| `/Rover/camera/cam_depth/depth/{number,name,url_*}` | cam_depth type `depth` | MDB parameters | fail / – |
| `/Rover/camera/NavCam_{left,right}/{streaming,oncommand}/{number,name,url_*}` | NavCams type `rgba` | MDB parameters | fail / – |
| `/Rover/camera/NavCam_{left,right}/streaming/{state,bandwidth,…}` | rgba cams × Pragyaan field list | MDB parameters | fail / – |
| Camera space systems under `/Rover/camera/` for cameras not in the robot yaml, or kinds a camera's `type` doesn't produce | not produced by the robot yaml | MDB space systems | – / warn ("server generated from a different robot config?") |
| `camera_capture_depth.camera_name` ∈ {cam_depth} | depth cameras | command argument values | fail if a camera is missing / warn if extra |
| `camera_capture_high`, `camera_streaming`, `set_camera_streaming_params` ∈ {NavCam_left, NavCam_right} | rgba cameras | command argument values | fail / warn |
| Every `parameters:` and `commands:` path in the controller yaml | hand-written | MDB | fail / – |
| Argument names each handler expects (`action`, `camera_name`, `bitrate`, …) | the controller's `command_handlers()` (sim startup only) | command arguments (and `commands:` keys) | fail / – |
| `images.buckets.*.path`, `payload.apxs.bucket.path` (+ their `number`, `name`, `url_*`) | hand-written | MDB space systems and parameters | fail / – |
| Every image bucket (`yamcs_naming.image_buckets`: camera buckets, `images.buckets.*.name`, `payload.apxs.bucket.name`) | robot yaml + controller yaml | storage buckets | fail / – |
| Other `images_*` buckets on the server (e.g. a renamed camera, or a kind its type no longer produces) | not used by this config | storage buckets | – / warn (kept, never deleted) |

The checker (`src/tmtc/yamcs_contract.py`) uses only the standard library, the parsed MDB and the server's bucket names, so it has no Isaac Sim imports. Each controller's `command_handlers()` (commands-yaml key → handler and its argument names) is the single place for command handlers: `YamcsTMTC.setup_command_callbacks` registers them, and `check_yamcs_server()` verifies their commands and arguments. The standalone `scripts/check_yamcs_contract.py` cannot import the controllers (they need Isaac Sim), so it checks everything except the handler argument names.

## 4. Running

- `ImagesHandler.save_camera_image(image, camera, kind)`, or `save_image(image, role)` for lander, monitoring and APXS (the payload handler registers its bucket as role `apxs` with `add_bucket(role, bucket_conf)`):
  - encodes the PNG in memory, uploads it to `bucket_name(camera, kind)` (or the role's bucket), and sets the `number`, `name` and `url_*` values under `meta_path(camera, kind)` (or the role's `path`).
  - The URLs are the same as today (`/storage/buckets/<name>/objects/<file>`), so Grafana keeps working.
  - Every image of the run is kept: the buckets' limits (set by the generator) are sized for a mission. A failed upload (e.g. a full bucket, or an image over Yamcs' 5 MB upload limit) is logged and skipped; a failing command never stops the TC listener.
- Commanders, the camera handler and transmitters call `yamcs_naming` (or `ImagesHandler`) and never assemble a name themselves.
- Camera commands reach the robot's handler only with a `camera_name` of the command's camera type (`CAMERA_COMMAND_TYPE`): `YamcsTMTC.setup_command_callbacks` wraps their handlers with `commands_handler.check_camera_argument`, which logs and drops any other camera (the server MDB may list cameras this robot does not have). The commanders do not check it again.
- Generic vs robot-specific: what follows from the robot yaml or the naming rule is generic (`cameras_of_type`, the camera argument check, `check_kind_resolutions`); how each robot snaps, processes and transmits images, its `KIND_RESOLUTION` and `STREAMING_FIELDS`, and its command policy (e.g. the Pragyaan camera power gate) stay in `src/mission_specific/<robot>/tmtc`, even where two robots match today.
- Streaming is per camera: one streaming job and one compression model per rgba camera. The streaming-state telemetry checks that camera's own job.
- Turning the camera on or off only starts or stops cameras whose type includes `rgba`.
- `robot.py` only creates the render products listed in each camera's `type`.

## 5. CI

A standalone script runs steps 2–4 against a running Yamcs server for a robot and controller pair, without Isaac Sim (exit code 1 if anything is missing). It checks everything except the handler argument names, which only the sim knows:

```
pixi run -e yamcs check-yamcs-contract         # Pragyaan, Yamcs instance workshop
pixi run -e yamcs check-yamcs-contract-husky   # Husky, Yamcs instance husky
python scripts/check_yamcs_contract.py --robot pragyaan --controller pragyaan-controller [--address localhost:8090 --instance workshop]
```

The unit tests (`test/unit/test_robot_camera_confs.py`, `test/unit/test_yamcs_naming.py`) also check, without a server, that every `cfg/robot` camera list parses, that the husky and pragyaan cameras have the resolution of each of their image kinds (`KIND_RESOLUTION` in `src/mission_specific/<robot>/tmtc/enums.py`, also checked by the camera handler at startup), that their `STREAMING_FIELDS` are valid MDB names, and that every `cfg/controller` yaml has `mdb_space_systems`, a boolean `images.clear_buckets_on_startup` if set, and unique image bucket names.

CI generates the MDB from the OmniLRS commit under test before starting Yamcs: `artefacts.Dockerfile` runs `mdb/generate_mdb_{rover,lander,admin}.py` of yamcs-mission-control-pragyaan and `mdb/generate_mdb_rover.py` of yamcs-mission-control-husky with `OMNILRS_ROOT=/workspace/omnilrs`. The sim then runs the same check at startup, and `test/ci_artefacts/test_yamcs_sanity*.py` (Pragyaan) and `test/ci_artefacts/test_yamcs_husky_sanity.py` (Husky) check that the startup "no data" image metadata arrived for every (camera, kind) of the robot yaml. The Husky test also captures an image with every camera command and checks that its image metadata arrives.

## 6. Deleted

- **OmniLRS:**
  - `cfg/mdb/`
  - `scripts/fix_yamcs_tmp.sh` and the pixi `fix-tmp` task
  - the `/tmp` writes in `ImagesHandler`
  - the `mdb:` file lists
- **Yamcs repos:**
  - `mdb/cfg/<robot>.yaml` (the robot yaml copy)
  - the deploy paths and `deploy_buckets` logic in `_mdb_cli.py`
  - the `/tmp` image bucket list in `yamcs.yaml`, replaced by the generated image buckets

## 7. Yaml layout

- **Both robot yamls:** a `type` on each camera and a comment pointing to `robot_camera_confs.py` and `yamcs_naming.py`.
- **Both controller yamls:**
  - fully hand-written; no generated sections remain
  - `mdb:` file list → `mdb_space_systems: ["/Rover", "/Lander", "/Admin"]` for Pragyaan, `["/Rover"]` for Husky
  - camera buckets and per-camera telemetry removed
  - `images.buckets` changed from a list to entries named by role (`lander`, `monitoring`; empty for Husky)
  - `clear_buckets_on_startup: true` added (optional, default true)
  - the `crf` and `seed` nulls restored

## 8. Smaller choices

- Lander and monitoring buckets are named by role, so the code uses `save_image(img, "lander")` and no bucket name is written in Python.
- APXS stays under `payload.apxs.bucket`, to avoid moving unrelated config.
- `CAMERA_ROOT = "/Rover/camera"` is a constant in the module, not yaml. Both robots use it, and the generator needs it too.
- Every image of a run is kept (bucket limits sized for a mission). By default each OmniLRS start is a new run: `clear_buckets_on_startup` (default true) empties the buckets and numbering restarts at 0000. With `false`, the images of earlier runs are kept and numbering continues after the last one.

- **CI generator source:** use yamcs-mission-control main for CI to generate the MDB, so it always runs against the OmniLRS commit under test.
- **Grafana dashboards:** they refer to camera parameter paths. Disregard for now.
- **Stale-bucket cleanup:** leftover image buckets on the server (e.g. `images_cam_depth_streaming` after `cam_depth` became depth-only) are only warned about by the startup check, never deleted.
