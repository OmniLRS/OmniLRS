# Model Unit Tests

Standalone tests for the physics models (power, thermal, radio, camera compression).
They exercise each model in isolation and produce diagnostic plots.

## Setup

```bash
# from the repository root (OmniLRS/)
python3.11 -m venv test/venv
source test/venv/bin/activate
pip install -r test/requirements.txt
```

> **Note:** Python 3.11+ is required (`StrEnum` support). The venv is
> created with `python3.11` explicitly (isaacsim 5.0). Adjust as needed.

## Run

All commands assume the venv is active and are executed from the repository root:

```bash
python -m src.subsystems.robot_physics_models.radio_model
python -m src.subsystems.robot_physics_models.thermal_model
python -m src.subsystems.robot_physics_models.power_model
python -m src.subsystems.robot_physics_models.camera_compression_model
```

Each script will print a message and save a `.png` plot in the current directory.

The camera compression test streams the 640x480 rover frames in `test/inputs/camera_compression/` through the model for six bitrate / packet-loss / keyframe scenarios. For each scenario it writes the processed frames and a 1 fps video to `test/outputs/camera_compression/<scenario>/`, and prints the bandwidth and packet-loss counts.

## Yamcs TM/TC unit tests

The robot camera config, the Yamcs naming rule, the parsing of the XTCE exported by the Yamcs server, the startup check
against it, the camera argument check of the commands, the import rule (only Yamcs mode imports Yamcs code) and the
image uploads to Yamcs storage (with a fake storage client) run without Isaac Sim or a Yamcs server (`pytest`, plus
PyYAML for the checks of the `cfg/robot` and `cfg/controller` yamls, and Pillow for the image uploads):

```bash
python -m pytest -v test/test_robot_camera_confs.py test/test_yamcs_naming.py test/test_mdb_parsing_service.py \
    test/test_yamcs_contract.py test/test_commands_handler.py test/test_import_layering.py test/test_images_handler.py
```
