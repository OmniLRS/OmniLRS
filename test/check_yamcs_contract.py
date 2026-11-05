"""Check a robot + controller config against the MDB and buckets of a running Yamcs server, without Isaac Sim.

Runs the same check as OmniLRS startup in Yamcs mode (docs/yamcs-omnilrs-sync.md, section 3) and prints its summary,
except the argument names of the command handlers: they are defined in the controllers, which need Isaac Sim to import,
so only the startup check in the sim verifies them.
Exit code 1 if the server misses anything OmniLRS sends or handles; extras are only warnings.

    pixi run -e yamcs check-yamcs-contract
    python test/check_yamcs_contract.py --robot pragyaan --controller pragyaan-controller \
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


def load_robots_settings(kind: str, name: str) -> dict:
    with open(REPO_ROOT / "cfg" / kind / f"{name}.yaml") as f:
        return yaml.safe_load(f)["robots_settings"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", required=True, help="cfg/robot/<robot>.yaml")
    parser.add_argument("--controller", required=True, help="cfg/controller/<name>.yaml")
    parser.add_argument("--address", default="localhost:8090")
    parser.add_argument("--instance", default="workshop")
    args = parser.parse_args()

    robot = load_robots_settings("robot", args.robot)["parameters"]
    # Streaming telemetry fields of the robot's transmitter (standard-library module, importable without Isaac Sim)
    streaming_fields = importlib.import_module(f"src.mission_specific.{args.robot}.tmtc.enums").STREAMING_FIELDS
    yamcs_conf = load_robots_settings("controller", args.controller)["yamcs_tmtc"]

    client = YamcsClient(args.address)
    mdb = MdbParsingService.load_from_server(client.get_mdb(args.instance), yamcs_conf["mdb_space_systems"])
    report = check_contract(
        mdb=mdb,
        buckets={bucket.name for bucket in client.get_storage_client().list_buckets()},
        cameras=yamcs_naming.parse_cameras(robot.get("camera")),
        streaming_fields=streaming_fields,
        yamcs_conf=yamcs_conf,
        command_args=None,  # handler argument names: checked at sim startup only
    )
    print(report.summary(yamcs_conf["mdb_space_systems"]))
    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
