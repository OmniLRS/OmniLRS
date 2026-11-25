"""Zenoh mode config: the stream settings are in the controller yaml (robots_settings.zenoh), not in the robot yaml.

python -m pytest -v test/unit/test_zenoh_config.py
"""

import re
import tomllib
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]

# Keys the Zenoh bridges and the Zenoh robot manager read (src/environments_wrappers/zenoh)
REQUIRED_STREAM_KEYS = {
    "camera": {"publish_period_s", "is_logging"},
    "imu": {"publish_period_s", "is_logging"},
    "joint_telemetry": {
        "publish_period_s",
        "init_retry_s",
        "not_ready_log_period_s",
        "keep_history",
        "history_len",
        "is_logging",
    },
    "joint_commands": {"is_logging"},
    "ground_truth_pose": {"is_logging"},
}


def _zenoh_tasks() -> dict[str, str]:
    with open(REPO_ROOT / "pixi.toml", "rb") as f:
        tasks = tomllib.load(f)["feature"]["zenoh"]["tasks"]
    return {name: command for name, command in tasks.items() if "mode=Zenoh" in command}


@pytest.mark.parametrize("path", sorted((REPO_ROOT / "cfg" / "robot").glob("*.yaml")), ids=lambda p: p.stem)
def test_robot_yaml_has_no_zenoh_settings(path):
    parameters = yaml.safe_load(path.read_text())["robots_settings"]["parameters"]
    assert "zenoh" not in parameters, "the Zenoh stream settings belong in the controller yaml (robots_settings.zenoh)"


def test_every_zenoh_task_passes_a_controller_with_the_stream_settings():
    tasks = _zenoh_tasks()
    assert tasks
    for name, command in tasks.items():
        match = re.search(r"\bcontroller=(\S+)", command)
        assert match, f"pixi task {name}: Zenoh mode needs controller=<robot>-controller"
        controller = yaml.safe_load((REPO_ROOT / "cfg" / "controller" / f"{match.group(1)}.yaml").read_text())
        streams = controller["robots_settings"]["zenoh"]
        for channel, keys in REQUIRED_STREAM_KEYS.items():
            assert keys <= set(streams[channel]), (
                f"{match.group(1)}: zenoh.{channel} lacks {keys - set(streams[channel])}"
            )
