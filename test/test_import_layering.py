"""Import rule: only Yamcs mode imports the Yamcs TM/TC code (no Isaac Sim required).

src/tmtc and src/mission_specific/<robot>/tmtc are Yamcs code. The robot, the configurations, the environments and the
other modes (ROS2, Zenoh, SDG) must not import them, so they never depend on Yamcs naming or Yamcs controllers.

python -m pytest -v test/test_import_layering.py
"""

import ast
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"

# Packages allowed to import Yamcs code: the Yamcs code itself and the Yamcs mode wrapper
YAMCS_DIRS = [SRC / "tmtc", SRC / "environments_wrappers" / "yamcs", *SRC.glob("mission_specific/*/tmtc")]
OTHER_FILES = sorted(
    path for path in SRC.rglob("*.py") if not any(path.is_relative_to(yamcs_dir) for yamcs_dir in YAMCS_DIRS)
)


def _is_yamcs_module(module: str) -> bool:
    parts = module.split(".")
    return parts[:2] == ["src", "tmtc"] or (parts[:2] == ["src", "mission_specific"] and parts[3:4] == ["tmtc"])


def _imported_modules(path: Path) -> list[str]:
    modules = []
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), filename=str(path))):
        if isinstance(node, ast.Import):
            modules += [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.append(node.module)
            modules += [f"{node.module}.{alias.name}" for alias in node.names]  # from src import tmtc
    return modules


def test_the_rule_covers_the_robot_and_the_other_modes():
    covered = {path.relative_to(SRC).as_posix() for path in OTHER_FILES}
    assert {"robots/robot.py", "environments_wrappers/zenoh/telemetry/camera_bridge.py"} <= covered


@pytest.mark.parametrize("path", OTHER_FILES, ids=lambda path: path.relative_to(SRC).as_posix())
def test_no_yamcs_import_outside_yamcs_mode(path):
    yamcs_imports = [module for module in _imported_modules(path) if _is_yamcs_module(module)]
    assert not yamcs_imports, f"{path.relative_to(SRC)} imports Yamcs code: {yamcs_imports}"
