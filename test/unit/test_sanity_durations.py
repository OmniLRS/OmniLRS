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


ISAAC_READY_LINE = (
    "[2026-11-09 05:24:08,512][src.environments_wrappers][INFO] - Isaac Sim ready after 25.417s since the Kit start "
    "(material library ready in 2.9s). OmniLRS startup begins."
)
KIT_LOG_LINE = (
    "2026-11-12T06:01:28Z [Info] [carb] Logging to file: "
    "/workspace/omnilrs/.pixi/envs/ci/lib/python3.12/site-packages/isaacsim/kit/logs/Kit/Isaac-Sim Python/6.2/"
    "kit_20261112_060128.log"
)


def _kit_line(ms: int, message: str) -> str:
    return f"2026-11-12T06:01:58Z [{ms:,}ms] [Info] {message}"


def _rtpso(ms: int, seconds: int) -> str:
    return _kit_line(ms, f"[omni.rtx] *** Waiting for RtPso async group async compilation: {seconds} seconds so far")


def test_isaac_ready_from_the_omnilrs_marker():
    log = "[22.364s] app ready\n" + ISAAC_READY_LINE + "\n[2026-11-09 05:34:07,629][src] - ready\n"
    assert sanity_durations.isaac_ready_s(log) == 25.417
    assert sanity_durations.isaac_ready_s("[22.364s] app ready\n") is None


def test_kit_log_path_from_the_sim_log():
    path = sanity_durations.kit_log_path("Starting kit application\n" + KIT_LOG_LINE + "\nnext line\n")
    assert str(path).endswith("Isaac-Sim Python/6.2/kit_20261112_060128.log")
    assert sanity_durations.kit_log_path("no Kit log line") is None


def test_startsim_logs_the_line_that_the_tests_parse():
    source = (REPO_ROOT / "src" / "environments_wrappers" / "__init__.py").read_text()
    assert '"Isaac Sim ready after %.3fs since the Kit start' in source


def test_rtx_compile_window_from_the_rtpso_lines():
    # As in the Kit log of a CI container run: other threads log during the compile, the main thread logs after it
    kit_log = "\n".join(
        [
            _kit_line(24_290, "[carb] Initializing plugin: rtx.spg.fabric"),
            _rtpso(29_300, 5),
            _kit_line(31_000, "[omni.ext.plugin] another thread"),
            _rtpso(34_300, 10),
            _rtpso(154_305, 130),
            "[ext: omni.foo-1.0.0] registered",  # no Kit time: ignored
            _kit_line(155_600, "[omni.usd] Encountered USD Warnings"),
        ]
    )
    assert sanity_durations.rtx_compile_windows(kit_log) == [(24.3, 155.6)]


def test_rtx_compile_windows_split_when_the_count_starts_again_and_end_at_most_5_s_later():
    kit_log = "\n".join([_rtpso(10_000, 5), _rtpso(15_000, 10), _rtpso(40_000, 5), _kit_line(60_000, "late line")])
    assert sanity_durations.rtx_compile_windows(kit_log) == [(5.0, 20.0), (35.0, 45.0)]
    assert sanity_durations.rtx_compile_windows(_kit_line(1_000, "no compile")) == []


@pytest.mark.parametrize(
    "windows, isaac_s",
    [
        ([], 25.0),  # no compile (warm cache)
        ([(24.0, 155.0)], 155.0),  # compile after the ready line: Isaac Sim time
        ([(10.0, 20.0)], 25.0),  # compile before the ready line: already in it
        ([(20.0, 30.0)], 30.0),  # only the part after the ready line is added
    ],
)
def test_measure_counts_the_rtx_compile_after_the_ready_line_as_isaac_time(windows, isaac_s):
    metrics = sanity_durations.measure([400.0, 1.0, 2.5, 0.5], isaac_ready_s=25.0, rtx_windows=windows)
    assert metrics["isaac_s"] == isaac_s
    assert metrics["omnilrs_s"] == 400.0 - isaac_s
    assert metrics["rtx_compile_s"] == sum(end - start for start, end in windows)
    assert metrics["isaac_ready_s"] == 25.0 and metrics["tests_s"] == 4.0


def test_measure_without_the_ready_line_reports_only_startup_and_tests():
    assert sanity_durations.measure([400.0, 1.0, 2.5, 0.5]) == {"startup_s": 400.0, "tests_s": 4.0}


def test_copy_kit_logs_copies_the_kit_log_named_in_each_sim_log(tmp_path):
    kit_dir = tmp_path / "kit"
    kit_dir.mkdir()
    (kit_dir / "kit_1.log").write_text("kit log")
    upload = tmp_path / "upload"
    upload.mkdir()
    (upload / "sim_startup.log").write_text(f"[Info] [carb] Logging to file: {kit_dir / 'kit_1.log'}\nmore\n")
    (upload / "sim_startup_other.log").write_text("no Kit log line\n")
    assert sanity_durations.copy_kit_logs(upload) == [upload / "kit_1.log"]
    assert (upload / "kit_1.log").read_text() == "kit log"


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
