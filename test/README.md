# Tests

| Folder | What | Run | Needs |
|---|---|---|---|
| `test/unit/` | Unit tests: robot camera config, Yamcs naming and startup check, camera argument check of the commands, image uploads, SDG writers, import rule (only Yamcs mode imports Yamcs code) | `pixi run -e test-unit test-unit` (or a bare `pytest`). CI: `.github/workflows/unit-tests.yaml`, on every push and pull request | No Isaac Sim, no server |
| `test/ci_artefacts/` | Sanity tests: start Isaac Sim (plus a Yamcs server, ROS 2 or Zenoh clients) and check the running sim | `pixi run test-ros2`, `test-ros2-largescale`, `test-yamcs`, `test-yamcs-largescale`, `test-zenoh`. CI: the Artefacts scenarios of `artefacts.yaml`, on push to main | Isaac Sim and the pixi `test-sim` env; Yamcs: a `yamcs-mission-control-pragyaan` checkout next to this repository |
| `test/physics_models/` | Inputs and Python requirements of the physics model scripts (below) | `python -m src.subsystems.robot_physics_models.<model>` | A venv with `test/physics_models/requirements.txt` |

`scripts/check_yamcs_contract.py` (`pixi run -e yamcs check-yamcs-contract`) is a tool, not a test: it checks a robot and
controller config against the MDB and buckets of a running Yamcs server.

## Unit tests

They import OmniLRS as `src.*` from the repository root (`pythonpath` in `pyproject.toml`) and need only `pytest`,
PyYAML (checks of the `cfg/robot` and `cfg/controller` yamls), Pillow (image uploads) and numpy, pandas and OpenCV (SDG
writers): the pixi `test-unit` environment.

```bash
pixi run -e test-unit test-unit
pixi run -e test-unit pytest -v test/unit/test_yamcs_naming.py   # a single file
```

## Physics models

Standalone tests for the physics models (power, thermal, radio, camera compression).
They exercise each model in isolation and produce diagnostic plots.

### Setup

```bash
# from the repository root (OmniLRS/)
python3.11 -m venv test/physics_models/venv
source test/physics_models/venv/bin/activate
pip install -r test/physics_models/requirements.txt
```

> **Note:** Python 3.11+ is required (`StrEnum` support). The venv is
> created with `python3.11` explicitly (isaacsim 5.0). Adjust as needed.

### Run

All commands assume the venv is active and are executed from the repository root:

```bash
python -m src.subsystems.robot_physics_models.radio_model
python -m src.subsystems.robot_physics_models.thermal_model
python -m src.subsystems.robot_physics_models.power_model
python -m src.subsystems.robot_physics_models.camera_compression_model
```

Each script prints a message and saves its plots in `test/physics_models/outputs/`.

The camera compression test streams the 640x480 rover frames in `test/physics_models/inputs/camera_compression/` through the model for six bitrate / packet-loss / keyframe scenarios. For each scenario it writes the processed frames and a 1 fps video to `test/physics_models/outputs/camera_compression/<scenario>/`, and prints the bandwidth and packet-loss counts.
