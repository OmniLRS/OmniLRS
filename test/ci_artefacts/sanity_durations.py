"""Duration check of the sanity tests, on the Artefacts remote infra only.

Every sanity test file ends with test_durations_within_baseline, which compares the durations of its run with
durations_baseline.yaml (see conftest.py). Speeds differ between machines, so local runs (including a local
`artefacts run`) neither log nor check durations.

Standard library and PyYAML only: test/unit/test_sanity_durations.py loads this file by path.
"""

import os
import re
import shutil
import socket
import subprocess
from pathlib import Path

import yaml

BASELINE_FILE = Path(__file__).with_name("durations_baseline.yaml")
METRICS = ("isaac_s", "omnilrs_s", "tests_s")  # checked against the baseline

# Logged by startSim (src/environments_wrappers/__init__.py) once Isaac Sim has finished its startup work, with the
# time since the Kit start: `[<time>][src.environments_wrappers][INFO] - Isaac Sim ready after <seconds>s since the
# Kit start (...)`
_ISAAC_READY = re.compile(r"Isaac Sim ready after (\d+(?:\.\d+)?)s", re.M)

# Kit names its log file on the first lines of the sim log
_KIT_LOG_PATH = re.compile(r"\[carb\] Logging to file: (.+\.log)\s*$", re.M)

# RTX ray tracing pipeline compile on the first rendered frame: Kit logs it every 5 s, in the Kit log only (Info):
# `2026-11-12T06:01:58Z [29,300ms] [Info] [omni.rtx] *** Waiting for RtPso async group async compilation: 5 seconds
# so far`. Without a warm cache (every CI container) it takes about 5 min on the T4 (docs/todo_performance.md).
_RTPSO_WAIT = re.compile(r"Waiting for RtPso async group async compilation: (\d+) seconds so far")
_KIT_TIME_MS = re.compile(r"^\S+Z \[([\d,]+)ms\]")

# Failure hint when isaac_s is too long
ISAAC_SLOW_HINT = (
    "Hint: isaac_s is the Kit start until the `Isaac Sim ready after` line of the sim log, plus the RTX ray tracing "
    "pipeline compile after it (rtx_compile_s, from the `Waiting for RtPso` lines of the Kit log, uploaded with the "
    "run). The compile depends on the GPU, the driver, the Isaac Sim version and the render settings, not on the "
    "scene (docs/todo_performance.md)."
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


def kit_log_path(sim_log: str) -> Path | None:
    """Path of the Kit log file, from the `[carb] Logging to file:` line of the sim log."""
    match = _KIT_LOG_PATH.search(sim_log)
    return Path(match.group(1).strip()) if match else None


def rtx_compile_windows(kit_log: str) -> list[tuple[float, float]]:
    """(start, end) in seconds since the Kit start of each RTX pipeline compile in the Kit log.

    The `Waiting for RtPso` lines of one compile count up (5, 10, ... seconds so far). Start: the time of its first line
    minus that count. End: the next Kit log line after its last line (the main thread runs again), at most 5 s after it.
    """
    timed = []  # (seconds since the Kit start, "seconds so far" of a wait line or None)
    for line in kit_log.splitlines():
        time_match = _KIT_TIME_MS.match(line)
        if time_match:
            wait = _RTPSO_WAIT.search(line)
            timed.append((int(time_match.group(1).replace(",", "")) / 1000, int(wait.group(1)) if wait else None))
    compiles = []  # [start, index of the last wait line, its count]
    for i, (t, n) in enumerate(timed):
        if n is None:
            continue
        if compiles and n > compiles[-1][2]:
            compiles[-1][1:] = [i, n]
        else:
            compiles.append([t - n, i, n])
    windows = []
    for start, last, _ in compiles:
        last_t = timed[last][0]
        after = [t for t, n in timed[last + 1 :] if n is None]
        windows.append((start, min(after[0], last_t + 5) if after else last_t + 5))
    return windows


def measure(
    test_durations: list[float],
    isaac_ready_s: float | None = None,
    rtx_windows: list[tuple[float, float]] = (),
) -> dict[str, float]:
    """Metrics of a run from its test durations (setup + call + teardown, in run order).

    startup_s:     the first test, which pays the session fixtures: the Yamcs server start (Yamcs scenarios) and the
                   sim launch until its final startup marker.
    isaac_ready_s: Kit start until the `Isaac Sim ready after` line (only if isaac_ready_s is given).
    rtx_compile_s: total RTX ray tracing pipeline compile time, from the Kit log.
    isaac_s:       the Isaac Sim part of startup_s: isaac_ready_s plus the RTX compile time after the ready line.
    omnilrs_s:     the rest of startup_s: OmniLRS builds the scene, plus the launch before Kit starts (pixi, Python
                   imports, Yamcs server).
    tests_s:       all later tests.
    """
    startup_s = test_durations[0]
    metrics = {"startup_s": startup_s, "tests_s": sum(test_durations[1:])}
    if isaac_ready_s is not None:
        compile_after_ready = sum(max(0.0, end - max(start, isaac_ready_s)) for start, end in rtx_windows)
        isaac_s = isaac_ready_s + compile_after_ready
        metrics.update(
            isaac_ready_s=isaac_ready_s,
            rtx_compile_s=sum(end - start for start, end in rtx_windows),
            isaac_s=isaac_s,
            omnilrs_s=startup_s - isaac_s,
        )
    return metrics


def copy_kit_logs(upload_dir: Path) -> list[Path]:
    """Copy the Kit log of each sim log (sim_startup*.log) of upload_dir into upload_dir, so the run uploads it."""
    copied = []
    for sim_log in sorted(upload_dir.glob("sim_startup*.log")):
        path = kit_log_path(sim_log.read_text(errors="replace"))
        if path is not None and path.is_file():
            copied.append(Path(shutil.copy2(path, upload_dir / path.name)))
    return copied


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
