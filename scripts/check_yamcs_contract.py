"""Check a robot + controller config against the MDB of a running Yamcs server, without Isaac Sim.

Runs the same check as OmniLRS startup in Yamcs mode (docs/yamcs-omnilrs-sync.md, section 3) and prints its summary.
Exit code 1 if the server MDB misses anything OmniLRS sends or handles; extras are only warnings.

    pixi run -e yamcs check-yamcs-contract
    python scripts/check_yamcs_contract.py --robot pragyaan --controller pragyaan-controller \
        [--address localhost:8090 --instance workshop]
"""

import argparse
import importlib
import sys
from pathlib import Path

import yaml
from yamcs.client import YamcsClient

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from src.tmtc import yamcs_naming  # noqa: E402
from src.tmtc.mdb_parsing_service import MdbParsingService  # noqa: E402
from src.tmtc.yamcs_contract import check_contract  # noqa: E402

# Controller (cfg/controller/<name>.yaml) -> module with the COMMAND_ARGS its command handlers use
COMMAND_SPECS = {
    "pragyaan-controller": "src.mission_specific.pragyaan.tmtc.command_spec",
}


def load_robots_settings(kind: str, name: str) -> dict:
    with open(REPO_ROOT / "cfg" / kind / f"{name}.yaml") as f:
        return yaml.safe_load(f)["robots_settings"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", required=True, help="cfg/robot/<robot>.yaml")
    parser.add_argument("--controller", required=True, choices=sorted(COMMAND_SPECS), help="cfg/controller/<name>.yaml")
    parser.add_argument("--address", default="localhost:8090")
    parser.add_argument("--instance", default="workshop")
    args = parser.parse_args()

    robot = load_robots_settings("robot", args.robot)["parameters"]
    yamcs_conf = load_robots_settings("controller", args.controller)["yamcs_tmtc"]
    command_args = importlib.import_module(COMMAND_SPECS[args.controller]).COMMAND_ARGS

    mdb_client = YamcsClient(args.address).get_mdb(args.instance)
    mdb = MdbParsingService.load_from_server(mdb_client, yamcs_conf["mdb_space_systems"])
    report = check_contract(
        mdb, yamcs_naming.parse_cameras(robot.get("camera")), robot["robot_name"], yamcs_conf, command_args
    )
    print(report.summary(yamcs_conf["mdb_space_systems"]))
    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
