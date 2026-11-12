import json
import os
import socket
import time
from pathlib import Path

import pytest
import sanity_durations

# Durations of this run (Artefacts remote infra only, see sanity_durations.py)
CHECK_TEST = "test_durations_within_baseline"
BASELINE_PATH = "test/ci_artefacts/durations_baseline.yaml"
SUMMARY_METRICS = ("startup_s", "isaac_ready_s", "rtx_compile_s", "isaac_s", "omnilrs_s", "tests_s", "total_s")
_ON_INFRA = sanity_durations.on_artefacts_infra()
_session_start = 0.0
_test_durations: dict[str, float] = {}  # nodeid -> setup + call + teardown seconds, in run order
_isaac_ready_s: float | None = None  # from the sim log, set by the duration check
_rtx_windows: list[tuple[float, float]] = []  # RTX pipeline compiles, from the Kit log, set by the duration check
_record: dict = {}  # durations.json content, also printed in the terminal summary


def pytest_configure(config):
    """junit output path for the Artefacts CI scenarios (artefacts.yaml), written to its upload directory"""
    upload_dir = os.environ.get("ARTEFACTS_SCENARIO_UPLOAD_DIR")
    if upload_dir and not config.option.xmlpath:
        config.option.xmlpath = os.path.join(upload_dir, "tests_junit.xml")


def pytest_sessionstart(session):
    global _session_start
    _session_start = time.monotonic()


def pytest_runtest_logreport(report):
    _test_durations[report.nodeid] = _test_durations.get(report.nodeid, 0.0) + report.duration


def _measured() -> dict[str, float]:
    """Metrics of this run, from the tests before the duration check."""
    durations = [seconds for nodeid, seconds in _test_durations.items() if not nodeid.endswith("::" + CHECK_TEST)]
    return sanity_durations.measure(durations, _isaac_ready_s, _rtx_windows)


@pytest.fixture
def check_durations(request, record_testsuite_property):
    """Compare the durations of this run with durations_baseline.yaml; called by the last test of each file with the
    sim log, which gives the Isaac Sim part of the startup (ready line, and the Kit log with the RTX compile)."""

    def check(sim_log: str):
        global _isaac_ready_s, _rtx_windows
        if not _ON_INFRA:
            pytest.skip("durations are only checked on the Artefacts remote infra")
        if request.session.testsfailed:
            pytest.skip("other tests failed, so the durations are not compared")
        _isaac_ready_s = sanity_durations.isaac_ready_s(sim_log)
        if _isaac_ready_s is None:
            pytest.fail(
                "No `Isaac Sim ready after <seconds>s` line in the sim log (logged by startSim in "
                "src/environments_wrappers/__init__.py): cannot measure the Isaac Sim startup."
            )
        kit_log = sanity_durations.kit_log_path(sim_log)
        if kit_log is None or not kit_log.is_file():
            pytest.fail(
                f"No Kit log ({kit_log or 'no `[carb] Logging to file:` line in the sim log'}): cannot "
                "measure the RTX pipeline compile."
            )
        _rtx_windows = sanity_durations.rtx_compile_windows(kit_log.read_text(errors="replace"))
        measured = _measured()
        for metric, seconds in measured.items():
            record_testsuite_property(metric, round(seconds, 1))

        baseline = sanity_durations.load_baseline()
        gpu = sanity_durations.gpu_name()
        if gpu is not None and gpu != baseline["gpu"]:
            pytest.fail(
                f"The Artefacts infra GPU is {gpu!r}, but the baseline was measured on {baseline['gpu']!r}. "
                f"Measure the baseline again and update {BASELINE_PATH}."
            )
        scenario = request.path.stem
        reference = baseline["scenarios"].get(scenario)
        measured_text = ", ".join(f"{metric}: {measured[metric]:.0f}" for metric in sanity_durations.METRICS)
        if reference is None:
            pytest.skip(f"no baseline for {scenario} in {BASELINE_PATH} (this run: {measured_text})")
        flagged = sanity_durations.regressions(measured, reference, **baseline["tolerance"])
        if flagged:
            tolerance = baseline["tolerance"]
            message = (
                f"{scenario} is slower than its baseline (tolerance: more than x{tolerance['ratio']} "
                f"and more than +{tolerance['min_increase_s']} s):\n  "
                + "\n  ".join(flagged)
                + f"\nIf the increase is expected, update {scenario} in {BASELINE_PATH}."
            )
            if any(line.startswith("isaac_s ") for line in flagged):
                message += "\n" + sanity_durations.ISAAC_SLOW_HINT
            pytest.fail(message)

    return check


def pytest_sessionfinish(session):
    """Copy the Kit log into the Artefacts upload directory (every run), write durations.json (remote infra only)."""
    upload_dir = os.environ.get("ARTEFACTS_SCENARIO_UPLOAD_DIR")
    if upload_dir:
        sanity_durations.copy_kit_logs(Path(upload_dir))  # after the sim fixture teardown: the Kit log is complete
    if not _ON_INFRA or not _test_durations:
        return
    scenario = Path(next(iter(_test_durations)).split("::")[0]).stem
    reference = sanity_durations.load_baseline()["scenarios"].get(scenario, {})
    _record.update(
        scenario=scenario,
        artefacts_job_id=os.environ.get("ARTEFACTS_JOB_ID"),
        hostname=socket.gethostname(),
        gpu=sanity_durations.gpu_name(),
        **{metric: round(seconds, 1) for metric, seconds in _measured().items()},
        total_s=round(time.monotonic() - _session_start, 1),
        baseline=reference,
        tests={nodeid.split("::", 1)[-1]: round(seconds, 1) for nodeid, seconds in _test_durations.items()},
    )
    if upload_dir:
        Path(upload_dir, "durations.json").write_text(json.dumps(_record, indent=2) + "\n")


def pytest_terminal_summary(terminalreporter):
    if not _record:
        return
    terminalreporter.write_sep("-", f"durations on the Artefacts remote infra ({_record['gpu']})")
    for metric in SUMMARY_METRICS:
        if metric not in _record:
            continue
        reference = _record["baseline"].get(metric)
        baseline_text = f" (baseline {reference} s)" if reference is not None else ""
        terminalreporter.write_line(f"{metric}: {_record[metric]:.0f} s{baseline_text}")
