"""Duration check of the sanity tests, on the Artefacts remote infra only.

Every sanity test file ends with test_durations_within_baseline, which compares the durations of its run with
durations_baseline.yaml (see conftest.py). Speeds differ between machines, so local runs (including a local
`artefacts run`) neither log nor check durations.

Standard library and PyYAML only: test/unit/test_sanity_durations.py loads this file by path.
"""

import os
import re
import socket
import subprocess
from pathlib import Path

import yaml

BASELINE_FILE = Path(__file__).with_name("durations_baseline.yaml")
METRICS = ("isaac_s", "omnilrs_s", "tests_s")  # checked against the baseline

# Logged by startSim (src/environments_wrappers/__init__.py) once Isaac Sim has finished its startup work, with the
# time since the Kit start: `[<time>][src.environments_wrappers][INFO] - Isaac Sim ready after <seconds>s since the
# Kit start (... material cache found|missing ...)`
_ISAAC_READY = re.compile(r"Isaac Sim ready after (\d+(?:\.\d+)?)s(.*)$", re.M)

# Failure hint when isaac_s is too long
ISAAC_SLOW_HINT = (
    "Hint: isaac_s ends at the `Isaac Sim ready after` line of the sim log. If that line says `material cache "
    "missing`, Kit rebuilt its material cache (about 5 min): after an Isaac Sim upgrade, update "
    "ci.docker/material_cache.json and its path in artefacts.Dockerfile (see ci.docker/README.md)."
)

# Hostname of the Artefacts remote runners: platform-<job uuid>-<scenario index> (from the junit files of the
# reference job in durations_baseline.yaml). On that infra the Artefacts CLI also reads the AWS Batch job variables.
_INFRA_HOSTNAME = re.compile(r"platform-[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}-\d+")


def on_artefacts_infra(environ=os.environ, hostname: str | None = None) -> bool:
    """True on the Artefacts remote infra, False on local runs."""
    if environ.get("AWS_BATCH_JOB_ID") and environ.get("ARTEFACTS_JOB_ID"):
        return True
    return _INFRA_HOSTNAME.fullmatch(hostname or socket.gethostname()) is not None


def isaac_ready_s(sim_log: str) -> float | None:
    """Isaac Sim startup time from the `Isaac Sim ready after` line of the sim log, None if the line is missing."""
    match = _ISAAC_READY.search(sim_log)
    return float(match.group(1)) if match else None


def material_cache_missing(sim_log: str) -> bool:
    """True if the `Isaac Sim ready after` line says that Kit rebuilt its material cache."""
    match = _ISAAC_READY.search(sim_log)
    return match is not None and "material cache missing" in match.group(2)


def measure(test_durations: list[float], isaac_s: float | None = None) -> dict[str, float]:
    """Metrics of a run from its test durations (setup + call + teardown, in run order).

    startup_s: the first test, which pays the session fixtures: the Yamcs server start (Yamcs scenarios) and the sim
               launch until its final startup marker.
    isaac_s:   the Isaac Sim part of startup_s, from the `Isaac Sim ready after` line (only if isaac_s is given).
    omnilrs_s: the rest of startup_s: OmniLRS builds the scene, plus the launch before Kit starts (pixi, Python imports,
               Yamcs server).
    tests_s:   all later tests.
    """
    startup_s = test_durations[0]
    metrics = {"startup_s": startup_s, "tests_s": sum(test_durations[1:])}
    if isaac_s is not None:
        metrics.update(isaac_s=isaac_s, omnilrs_s=startup_s - isaac_s)
    return metrics


def regressions(measured: dict, baseline: dict, ratio: float, min_increase_s: float) -> list[str]:
    """One message per metric that is more than `ratio` times its baseline AND more than `min_increase_s` above it."""
    flagged = []
    for metric in METRICS:
        now, ref = measured[metric], baseline[metric]
        if now > ratio * ref and now - ref > min_increase_s:
            flagged.append(f"{metric} {now:.0f} s vs baseline {ref:.0f} s (+{now - ref:.0f} s)")
    return flagged


def load_baseline(path: Path = BASELINE_FILE) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def gpu_name() -> str | None:
    """Name of GPU 0 from nvidia-smi (e.g. "Tesla T4"), None if nvidia-smi is not available."""
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"], capture_output=True, text=True, timeout=10
        )
    except (OSError, subprocess.SubprocessError):
        return None
    lines = out.stdout.strip().splitlines()
    return lines[0].strip() if out.returncode == 0 and lines else None
