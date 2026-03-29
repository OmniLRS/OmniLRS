# Model Unit Tests

Standalone tests for the physics models (power, thermal, radio).  
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
```

Each script will print a message and save a `.png` plot in the current directory.
