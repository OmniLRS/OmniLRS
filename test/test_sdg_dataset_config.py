"""DatasetConf (mode=SDG dataset=<preset>) validation. No Isaac Sim required."""

from pathlib import Path

import pytest
import yaml

from src.configurations.dataset_confs import DEFAULTS, DatasetConf

REPO = Path(__file__).resolve().parents[1]


def test_defaults_load():
    c = DatasetConf()
    assert c.rig["baseline_m"] == 0.12
    assert all(g is False or g.get("enabled") is False for g in c.guards.values())


def test_partial_nested_override_keeps_other_defaults():
    c = DatasetConf(rig={"baseline_m": 0.2}, guards={"mesh_probe": {"enabled": True}})
    assert c.rig["baseline_m"] == 0.2
    assert c.rig["height_m"] == DEFAULTS["rig"]["height_m"]
    assert c.guards["mesh_probe"] == {"enabled": True, "tol_m": 0.3}
    assert c.guards["dark_frame"]["enabled"] is False


@pytest.mark.parametrize(
    "kwargs, msg",
    [
        ({"num_terrains": 0}, "num_terrains"),
        ({"num_terrains": 1001}, "num_terrains"),
        ({"rig": {"height_m": [1.0, 0.5]}}, "height_m"),
        ({"sun": {"elevation_buckets": [{"weight": 0.0, "range": [1, 5]}]}}, "weight"),
        ({"sun": {"intensity_mode": "auto"}}, "intensity_mode"),
        ({"rig": {"unknown_key": 1}}, "unknown_key"),
    ],
)
def test_invalid_values_are_rejected(kwargs, msg):
    with pytest.raises(AssertionError, match=msg):
        DatasetConf(**kwargs)


@pytest.mark.parametrize(
    "kwargs, msg",
    [
        ({"guards": {"mesh_probe": True}}, r"guards\.mesh_probe: expected a dict"),
        ({"guards": {"pt_runtime_switch": {"enabled": False}}}, r"guards\.pt_runtime_switch: expected a bool"),
        ({"guards": {"dark_frame": {"enabled": "yes"}}}, r"guards\.dark_frame\.enabled: expected a bool"),
        ({"rig": 5}, r"rig: expected a dict"),
    ],
)
def test_wrong_type_does_not_replace_a_default(kwargs, msg):
    with pytest.raises(AssertionError, match=msg):
        DatasetConf(**kwargs)


def test_right_types_still_merge():
    c = DatasetConf(guards={"hide_far_mesh": True, "pt_runtime_switch": True, "dark_frame": {"enabled": True}})
    assert c.guards["hide_far_mesh"] is True and c.guards["pt_runtime_switch"] is True
    assert c.guards["dark_frame"]["enabled"] is True and c.guards["dark_frame"]["retries"] == 3


@pytest.mark.parametrize("name", ["default.yaml", "moonseg.yaml"])
def test_dataset_yamls_validate(name):
    text = (REPO / "cfg" / "dataset" / name).read_text()
    # merged into mode=SDG; without the header Hydra would put the keys at the config root
    assert text.startswith("# @package mode\n")
    y = yaml.safe_load(text)
    assert "name" not in y  # mode=SDG keeps its name; dataset_settings selects the dataset manager
    DatasetConf(**y["dataset_settings"])


def test_config_has_optional_dataset_group():
    defaults = yaml.safe_load((REPO / "cfg" / "config.yaml").read_text())["defaults"]
    assert defaults.index({"optional dataset": None}) > defaults.index({"mode": "ROS2"})


def test_registered_in_config_factory():
    from src.configurations import configFactory

    assert "dataset_settings" in configFactory.getConfigs()
