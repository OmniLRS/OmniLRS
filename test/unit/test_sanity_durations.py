"""Duration check of the sanity tests (test/ci_artefacts/sanity_durations.py) and its baseline file.

python -m pytest -v test/unit/test_sanity_durations.py
"""

import importlib.util
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
CI_DIR = REPO_ROOT / "test" / "ci_artefacts"

# Loaded by path: test/ci_artefacts is not a package, and conftest.py imports it as a top-level module.
_spec = importlib.util.spec_from_file_location("sanity_durations", CI_DIR / "sanity_durations.py")
sanity_durations = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sanity_durations)

INFRA_HOST = "platform-6b6fbc05-37de-41b9-bffd-d7932f31e70d-3"


@pytest.mark.parametrize(
    "environ, hostname, expected",
    [
        ({}, INFRA_HOST, True),
        ({"AWS_BATCH_JOB_ID": "a", "ARTEFACTS_JOB_ID": "b"}, "ip-10-0-0-1", True),
        ({"ARTEFACTS_JOB_ID": "b"}, "ip-10-0-0-1", False),  # a local `artefacts run`
        ({}, "ip-10-0-0-1", False),
        ({}, "platform-local", False),
    ],
)
def test_on_artefacts_infra(environ, hostname, expected):
    assert sanity_durations.on_artefacts_infra(environ, hostname) is expected


def test_isaac_ready_from_the_kit_app_ready_line():
    log = "[21.5s] [ext: omni.kit.app] startup\n[22.364s] app ready\n[2026-11-09 05:34:07,629][src] - ready\n"
    assert sanity_durations.isaac_ready_s(log) == 22.364
    assert sanity_durations.isaac_ready_s("no kit output") is None


def test_measure_splits_the_first_test_into_isaac_and_omnilrs():
    assert sanity_durations.measure([400.0, 1.0, 2.5, 0.5]) == {"startup_s": 400.0, "tests_s": 4.0}
    assert sanity_durations.measure([400.0, 1.0, 2.5, 0.5], isaac_s=25.0) == {
        "startup_s": 400.0,
        "isaac_s": 25.0,
        "omnilrs_s": 375.0,
        "tests_s": 4.0,
    }


BASELINE = {"isaac_s": 25, "omnilrs_s": 400, "tests_s": 10}


@pytest.mark.parametrize(
    "changes, flagged_metrics",
    [
        ({}, []),  # at baseline
        ({"omnilrs_s": 590}, []),  # x1.48: under the ratio
        ({"omnilrs_s": 610}, ["omnilrs_s"]),  # x1.53 and +210 s
        ({"isaac_s": 80}, []),  # x3.2 but only +55 s
        ({"isaac_s": 90}, ["isaac_s"]),  # x3.6 and +65 s
        ({"tests_s": 75}, ["tests_s"]),  # x7.5 and +65 s
        ({"isaac_s": 10, "omnilrs_s": 100, "tests_s": 1}, []),  # faster is never flagged
    ],
)
def test_regressions_need_ratio_and_min_increase(changes, flagged_metrics):
    flagged = sanity_durations.regressions(BASELINE | changes, BASELINE, ratio=1.5, min_increase_s=60)
    assert [message.split()[0] for message in flagged] == flagged_metrics


def test_baseline_covers_every_artefacts_scenario():
    baseline = sanity_durations.load_baseline()
    assert isinstance(baseline["gpu"], str) and baseline["gpu"]
    assert set(baseline["tolerance"]) == {"ratio", "min_increase_s"}
    with open(REPO_ROOT / "artefacts.yaml") as f:
        jobs = yaml.safe_load(f)["jobs"]
    scenarios = {s["name"] for job in jobs.values() for s in job["scenarios"]["settings"]}
    assert set(baseline["scenarios"]) == scenarios
    for name, values in baseline["scenarios"].items():
        assert set(values) == set(sanity_durations.METRICS), name
        assert (CI_DIR / f"{name}.py").is_file(), f"{name}: no test file of that name"


@pytest.mark.parametrize("path", sorted(CI_DIR.glob("test_*.py")), ids=lambda p: p.stem)
def test_duration_check_is_the_last_test(path):
    tests = [
        line.split("(")[0].removeprefix("def ")
        for line in path.read_text().splitlines()
        if line.startswith("def test_")
    ]
    assert tests[-1] == "test_durations_within_baseline"
