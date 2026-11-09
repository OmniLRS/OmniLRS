# Tests

| Folder | What | Run | Needs |
|---|---|---|---|
| `test/unit/` | Unit tests: robot camera configuration, Yamcs naming and startup check, camera argument check of the commands, image uploads, SDG writers and dataset tools, import rule (only Yamcs mode imports Yamcs code) | `pixi run -e test-unit test-unit` (or a bare `pytest`). CI: `.github/workflows/unit-tests.yaml`, on every push and pull request | No Isaac Sim, no server |
| `test/ci_artefacts/` | Sanity tests: start Isaac Sim (plus a Yamcs server, ROS 2 or Zenoh clients) and check the running sim | `pixi run test-ros2`, `test-ros2-largescale`, `test-yamcs`, `test-yamcs-largescale`, `test-yamcs-husky`, `test-zenoh`. CI: the Artefacts scenarios of `artefacts.yaml`, on push to main (below) | Isaac Sim and the pixi `test-sim` env; Yamcs: a `yamcs-mission-control-pragyaan` (`yamcs-mission-control-husky` for `test-yamcs-husky`) checkout next to this repository, or `YAMCS_MC_DIR` (`YAMCS_HUSKY_MC_DIR`) |
| `test/physics_models/` | Inputs of the physics model scripts (below) | `pixi run python -m src.subsystems.robot_physics_models.<model>` | The pixi `default` environment (the scripts do not start Isaac Sim) |

## Sanity test matrix

The Artefacts scenarios (`artefacts.yaml`), one `test/ci_artefacts` file each:

| Scenario | Environment | Robot | Mode |
|---|---|---|---|
| `test_ros2_sanity` | `lunalab` | husky (ROS 2 USD) | ROS2 |
| `test_ros2_sanity_largescale` | `largescale` | husky (ROS 2 USD) | ROS2 |
| `test_yamcs_sanity` | `lunaryard_40m_workshop` | pragyaan | Yamcs |
| `test_yamcs_sanity_largescale` | `largescale_workshop` | pragyaan | Yamcs |
| `test_yamcs_husky_sanity` | `lunaryard_20m` | husky | Yamcs |
| `test_zenoh_sanity` | `lunaryard_20m` | husky | Zenoh |

## Unit tests

They import OmniLRS as `src.*` from the repository root (`pythonpath` in `pyproject.toml`). They need only the pixi
`test-unit` environment. It has `pytest`, PyYAML (`cfg/robot` and `cfg/controller` yamls), Pillow (image uploads),
numpy, pandas and OpenCV (SDG writers), and SciPy (SDG dataset tests).

```bash
pixi run -e test-unit test-unit
pixi run -e test-unit pytest -v test/unit/test_yamcs_naming.py   # a single file
```

## Physics models

Standalone tests for the physics models (power, thermal, radio, camera compression).
They exercise each model in isolation and produce diagnostic plots.

### Setup

The scripts need numpy, matplotlib, av and Pillow. The pixi `default` environment of the OmniLRS installation has them,
so no other setup is necessary.

### Run

Run the scripts from the repository root:

```bash
pixi run python -m src.subsystems.robot_physics_models.radio_model
pixi run python -m src.subsystems.robot_physics_models.thermal_model
pixi run python -m src.subsystems.robot_physics_models.power_model
pixi run python -m src.subsystems.robot_physics_models.camera_compression_model
```

Each script prints a message and saves its plots in `test/physics_models/outputs/`.

The camera compression test streams the 640x480 rover frames in `test/physics_models/inputs/camera_compression/` through the model for six bitrate / packet-loss / keyframe scenarios. For each scenario it writes the processed frames and a 1 fps video to `test/physics_models/outputs/camera_compression/<scenario>/`, and prints the bandwidth and packet-loss counts.
