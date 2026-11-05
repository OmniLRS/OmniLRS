"""Startup check of OmniLRS against the Yamcs server MDB (see docs/yamcs-omnilrs-sync.md, section 3).

Compares what OmniLRS sends and handles (robot yaml cameras via yamcs_naming, controller yaml paths, and the
argument names of the controller's command handlers) with the MDB and the storage buckets of the server:
  - missing on the server -> failure (startup aborts, scripts/check_yamcs_contract.py exits 1),
  - on the server but not produced by the robot yaml (cameras, kinds, camera_name labels, image buckets) -> warning.

Standard library only (plus the MDB model from mdb_parsing_service), so it runs without Isaac Sim.
"""

from dataclasses import dataclass, field
from typing import Iterable, Mapping

from src.configurations.robot_camera_confs import RobotCameraConf
from src.tmtc import yamcs_naming
from src.tmtc.mdb_parsing_service import ServerMdb


@dataclass
class ContractReport:
    missing: list[str] = field(default_factory=list)
    extra: list[str] = field(default_factory=list)
    checked: int = 0

    @property
    def ok(self) -> bool:
        return not self.missing

    def summary(self, mdb_space_systems: list[str]) -> str:
        status = "OK" if self.ok else f"FAILED: {len(self.missing)} missing"
        lines = [
            f"[Yamcs] Contract check against the MDB ({', '.join(mdb_space_systems)}) and buckets: "
            f"{self.checked} items, {status}"
        ]
        lines += [f"[Yamcs]   missing: {item}" for item in self.missing]
        if self.extra:
            lines.append("[Yamcs]   warning, the server has items this robot config does not produce:")
            lines += [f"[Yamcs]   extra: {item}" for item in self.extra]
            lines.append("[Yamcs]   (server MDB and buckets generated from a different robot config?)")
        return "\n".join(lines)


def check_contract(
    mdb: ServerMdb,
    buckets: set[str],
    cameras: list[RobotCameraConf],
    streaming_fields: Iterable[str],
    yamcs_conf: Mapping,
    command_args: Mapping[str, list[str]] | None,
) -> ContractReport:
    """buckets: names of the storage buckets on the server;
    cameras: robot cameras (RobotParameters.camera, parsed by robot_camera_confs.parse_cameras);
    streaming_fields: streaming telemetry sent per rgba camera (STREAMING_FIELDS of the robot's tmtc/enums.py);
    yamcs_conf: controller yaml `yamcs_tmtc`;
    command_args: commands-yaml key -> argument names its handler takes (the controller's command_handlers()),
    or None to skip that check (the standalone check cannot import the controllers, which need Isaac Sim)."""
    report = ContractReport()

    def expect(present: bool, item: str) -> None:
        report.checked += 1
        if not present and item not in report.missing:
            report.missing.append(item)

    # Every bucket OmniLRS uploads to, with its image metadata (number, name, url_*)
    used_buckets = yamcs_naming.image_buckets(cameras, yamcs_conf)
    for bucket_name, path in used_buckets:
        expect(bucket_name in buckets, f"bucket {bucket_name}")
        expect(path in mdb.space_systems, f"space system {path}")
        for meta_field in yamcs_naming.IMAGE_META_FIELDS:
            expect(f"{path}/{meta_field}" in mdb.parameters, f"parameter {path}/{meta_field}")

    # Streaming telemetry of every rgba camera
    for camera in cameras:
        if camera.has_type("rgba"):
            for streaming_field in streaming_fields:
                param = yamcs_naming.streaming_param(camera.name, streaming_field)
                expect(param in mdb.parameters, f"parameter {param}")

    # Hand-written controller yaml paths
    for param in yamcs_conf["parameters"].values():
        expect(param in mdb.parameters, f"parameter {param}")
    commands_conf = yamcs_conf["commands"]
    for command in commands_conf.values():
        expect(command in mdb.commands, f"command {command}")

    # Argument names each command handler reads
    for key, arg_names in (command_args or {}).items():
        if key not in commands_conf:
            report.missing.append(f"commands.{key} in the controller yaml (a command handler needs it)")
            continue
        command = mdb.commands.get(commands_conf[key])
        if command is None:
            continue  # already reported above
        mdb_args = {arg["name"] for arg in command["args"]}
        for arg_name in arg_names:
            expect(arg_name in mdb_args, f"argument {arg_name} of command {commands_conf[key]}")

    # camera_name labels of the camera commands the controller handles
    for command_name in yamcs_naming.CAMERA_COMMAND_TYPE:
        path = f"{yamcs_naming.CAMERA_ROOT}/{command_name}"
        if path not in commands_conf.values() or path not in mdb.commands:
            continue
        arg = next((a for a in mdb.commands[path]["args"] if a["name"] == yamcs_naming.CAMERA_NAME_ARG), None)
        if arg is None:
            expect(False, f"argument {yamcs_naming.CAMERA_NAME_ARG} of command {path}")
            continue
        if arg["spec"]["kind"] != "enum":
            expect(False, f"enumerated argument {yamcs_naming.CAMERA_NAME_ARG} of command {path}")
            continue
        labels = set(arg["spec"]["mapping"].values())
        expected = yamcs_naming.cameras_for_command(cameras, command_name)
        for camera_name in expected:
            expect(camera_name in labels, f"{yamcs_naming.CAMERA_NAME_ARG} value {camera_name} of command {path}")
        for label in sorted(labels - set(expected)):
            report.extra.append(f"{yamcs_naming.CAMERA_NAME_ARG} value {label} of command {path}")

    # Image buckets this robot config does not use (e.g. a renamed camera, or a kind its type no longer produces)
    used_names = {bucket_name for bucket_name, _ in used_buckets}
    for bucket_name in sorted(buckets):
        if bucket_name.startswith(yamcs_naming.BUCKET_PREFIX) and bucket_name not in used_names:
            report.extra.append(f"bucket {bucket_name} (not used by this robot config)")

    # Camera space systems (and their kinds) the robot yaml does not produce
    kinds = {camera.name: set(yamcs_naming.camera_kinds(camera)) for camera in cameras}
    prefix = yamcs_naming.CAMERA_ROOT + "/"
    for system in sorted(mdb.space_systems):
        if not system.startswith(prefix):
            continue
        parts = system[len(prefix) :].split("/")
        if parts[0] not in kinds:
            if len(parts) == 1:
                report.extra.append(f"space system {system} (no such camera in the robot yaml)")
        elif len(parts) == 2 and parts[1] not in kinds[parts[0]]:
            report.extra.append(f"space system {system} (camera {parts[0]} does not produce {parts[1]} images)")

    return report
