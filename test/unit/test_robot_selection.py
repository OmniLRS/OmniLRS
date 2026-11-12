"""The robot comes from a robot yaml (robot=<name>), never from the environment yaml.

python -m pytest -v test/unit/test_robot_selection.py
"""

from pathlib import Path

import pytest
import yaml

from src.environments_wrappers import MODES_WITH_ROBOT, ROS2_HUSKY_USD, _require_robot

REPO_ROOT = Path(__file__).resolve().parents[2]
ENVIRONMENTS = sorted((REPO_ROOT / "cfg" / "environment").glob("*.yaml"))
ROBOTS = sorted((REPO_ROOT / "cfg" / "robot").glob("*.yaml"))


@pytest.mark.parametrize("path", ENVIRONMENTS, ids=lambda p: p.stem)
def test_environment_yaml_has_no_robot(path):
    robots_settings = yaml.safe_load(path.read_text()).get("robots_settings")
    assert robots_settings in (None, {"robots_root": "/Robots"}), "pass the robot with robot=<name> (cfg/robot)"


@pytest.mark.parametrize("path", ROBOTS, ids=lambda p: p.stem)
def test_robot_yaml_spawns_relative_to_the_terrain(path):
    # The same robot yaml is used in every environment, so its spawn height follows the terrain
    pose = yaml.safe_load(path.read_text())["robots_settings"]["parameters"]["pose"]
    assert pose.get("height_above_terrain_override") is not None


def _cfg(mode: str, robots_settings: dict | None) -> dict:
    environment = {} if robots_settings is None else {"robots_settings": robots_settings}
    return {"mode": {"name": mode}, "environment": environment}


@pytest.mark.parametrize("mode", MODES_WITH_ROBOT)
@pytest.mark.parametrize("robots_settings", [None, {"robots_root": "/Robots"}], ids=["no_settings", "root_only"])
def test_modes_with_a_robot_stop_without_robot(mode, robots_settings):
    with pytest.raises(ValueError, match=r"needs a robot: pass robot=<name>") as error:
        _require_robot(_cfg(mode, robots_settings))
    assert (ROS2_HUSKY_USD in str(error.value)) == (mode == "ROS2")


@pytest.mark.parametrize("mode", MODES_WITH_ROBOT)
def test_modes_with_a_robot_start_with_robot(mode):
    _require_robot(_cfg(mode, {"robots_root": "/Robots", "parameters": {"robot_name": "husky"}}))


def test_sdg_needs_no_robot():
    _require_robot(_cfg("SDG", None))


def test_ros2_ci_tests_use_the_ros2_husky_usd():
    for name in ("test_ros2_sanity.py", "test_ros2_sanity_largescale.py"):
        source = (REPO_ROOT / "test" / "ci_artefacts" / name).read_text()
        assert '"robot=husky"' in source and f"usd_path={ROS2_HUSKY_USD}" in source, name
