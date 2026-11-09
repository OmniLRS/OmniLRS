import binascii
import posixpath
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field


@dataclass
class ServerMdb:
    """What OmniLRS uses from the Yamcs server MDB (XTCE exported by the server, see MdbParsingService).

    space_systems: qualified names of every space system, e.g. "/Rover/camera/NavCam_left/streaming"
    parameters: qualified names of every parameter, e.g. "/Rover/battery_charge"
    commands: qualified command name -> {"name", "full_name", "root_system", "args": [{"name", "type", "spec"}]}
    registry: wire command name (bytes) -> command, for decoding the TC payloads Yamcs sends over UDP
    """

    space_systems: set[str] = field(default_factory=set)
    parameters: set[str] = field(default_factory=set)
    commands: dict[str, dict] = field(default_factory=dict)
    registry: dict[bytes, dict] = field(default_factory=dict)

    def merge(self, other: "ServerMdb") -> None:
        for wire_name, spec in other.registry.items():
            existing = self.registry.get(wire_name)
            if existing is not None and existing["full_name"] != spec["full_name"]:
                raise ValueError(
                    f"Ambiguous command wire name across MDB space systems: {wire_name!r}\n"
                    f"Existing: {existing['full_name']}\n"
                    f"New:      {spec['full_name']}\n"
                    "UDP payload alone cannot disambiguate these commands."
                )
            self.registry[wire_name] = spec
        self.space_systems |= other.space_systems
        self.parameters |= other.parameters
        self.commands.update(other.commands)


class MdbParsingService:
    """Parses the XTCE that the Yamcs server exports (Yamcs >= 5.8, exportXTCE) into a ServerMdb.

    Element namespaces are ignored. Wire sizes are read from the data encodings (IntegerDataEncoding,
    FloatDataEncoding), as the server does not repeat them on the argument types.
    """

    @classmethod
    def load_from_server(cls, mdb_client, space_systems: list[str]) -> ServerMdb:
        """Export and parse each space system, e.g. ["/Rover", "/Lander"], from a yamcs-client MDBClient."""
        mdb = ServerMdb()
        for name in space_systems:
            mdb.merge(cls.parse_xtce(mdb_client.export_space_system(name), name))
        return mdb

    @classmethod
    def parse_xtce(cls, xml_text: str, qualified_name: str) -> ServerMdb:
        """Parse an exported space system. qualified_name is its MDB path (the root element only carries its name)."""
        root = ET.fromstring(xml_text)
        if _tag(root) != "SpaceSystem":
            raise ValueError(f"Expected a SpaceSystem root element for {qualified_name!r}, got {_tag(root)!r}")

        systems = []  # (qualified name, element)
        cls._collect_space_systems(root, "/" + qualified_name.strip("/"), systems)

        mdb = ServerMdb()
        type_table = {}
        for path, system in systems:
            mdb.space_systems.add(path)
            for meta in _children(system, "TelemetryMetaData"):
                for param_set in _children(meta, "ParameterSet"):
                    for param in _children(param_set, "Parameter"):
                        mdb.parameters.add(f"{path}/{param.attrib['name']}")
            for meta in _children(system, "CommandMetaData"):
                for type_set in _children(meta, "ArgumentTypeSet"):
                    for arg_type in type_set:
                        spec = cls._parse_argument_type(arg_type)
                        if spec is not None:
                            type_table[f"{path}/{arg_type.attrib['name']}"] = spec

        for path, system in systems:
            for meta in _children(system, "CommandMetaData"):
                for cmd_set in _children(meta, "MetaCommandSet"):
                    for cmd in _children(cmd_set, "MetaCommand"):
                        cls._add_command(mdb, cmd, path, type_table)
        return mdb

    @classmethod
    def _collect_space_systems(cls, system, path, out):
        out.append((path, system))
        for child in _children(system, "SpaceSystem"):
            cls._collect_space_systems(child, f"{path}/{child.attrib['name']}", out)

    @staticmethod
    def _parse_argument_type(arg_type) -> dict | None:
        kind = _tag(arg_type)

        if kind in ("EnumeratedArgumentType", "BooleanArgumentType", "IntegerArgumentType"):
            enc = _child(arg_type, "IntegerDataEncoding")
            if enc is None:
                return None
            size_bits = int(enc.attrib.get("sizeInBits", "8"))
            byte_order = _byte_order(enc)
            if kind == "EnumeratedArgumentType":
                mapping = {
                    int(enum.attrib["value"]): enum.attrib["label"]
                    for enum_list in _children(arg_type, "EnumerationList")
                    for enum in _children(enum_list, "Enumeration")
                }
                return {"kind": "enum", "size_bits": size_bits, "byte_order": byte_order, "mapping": mapping}
            if kind == "BooleanArgumentType":
                return {
                    "kind": "bool",
                    "size_bits": size_bits,
                    "byte_order": byte_order,
                    "mapping": {
                        0: arg_type.attrib.get("zeroStringValue", "False"),
                        1: arg_type.attrib.get("oneStringValue", "True"),
                    },
                }
            return {
                "kind": "int",
                "size_bits": size_bits,
                "byte_order": byte_order,
                "signed": enc.attrib.get("encoding", "unsigned") != "unsigned",
            }

        if kind == "FloatArgumentType":
            enc = _child(arg_type, "FloatDataEncoding")
            if enc is None:
                return None
            return {
                "kind": "float",
                "size_bits": int(enc.attrib.get("sizeInBits", "32")),
                "byte_order": _byte_order(enc),
            }

        if kind == "StringArgumentType":
            term = next((el for el in arg_type.iter() if _tag(el) == "TerminationChar"), None)
            termination_char = int(term.text.strip(), 16) if term is not None and term.text else None
            return {"kind": "string", "termination_char": termination_char}

        return None  # types OmniLRS cannot decode; an error only if a command uses them

    @classmethod
    def _add_command(cls, mdb: ServerMdb, cmd, path: str, type_table: dict) -> None:
        full_name = f"{path}/{cmd.attrib['name']}"

        args = []
        for arg_list in _children(cmd, "ArgumentList"):
            for arg in _children(arg_list, "Argument"):
                type_name = _resolve_ref(arg.attrib["argumentTypeRef"], path, type_table)
                if type_name is None:
                    raise ValueError(
                        f"Unsupported or unknown argument type: {arg.attrib['argumentTypeRef']} in command '{full_name}'"
                    )
                args.append({"name": arg.attrib["name"], "type": type_name, "spec": type_table[type_name]})

        command = {
            "name": cmd.attrib["name"],
            "full_name": full_name,
            "root_system": full_name.strip("/").split("/")[0],
            "args": args,
        }
        mdb.commands[full_name] = command

        # The wire command name is the FixedValueEntry Yamcs puts first in the UDP payload
        fixed = next((el for el in cmd.iter() if _tag(el) == "FixedValueEntry"), None)
        if fixed is None:
            return
        wire_name = binascii.unhexlify(fixed.attrib["binaryValue"])
        if wire_name in mdb.registry:
            raise ValueError(
                f"Duplicate wire command name {wire_name!r}. "
                f"Commands: '{mdb.registry[wire_name]['full_name']}' and '{full_name}'"
            )
        mdb.registry[wire_name] = command

    @staticmethod
    def decode_tc_payload(payload, registry):
        for wire_name, cmd in registry.items():
            if not payload.startswith(wire_name):
                continue

            pos = len(wire_name)
            args_out = {}
            raw_args = {}

            for arg in cmd["args"]:
                arg_name = arg["name"]
                spec = arg["spec"]
                arg_type = spec["kind"]

                remaining = payload[pos:]

                if arg_type == "enum":
                    raw, consumed = MdbParsingService._decode_int(
                        remaining, spec["size_bits"], False, spec["byte_order"]
                    )
                    pos += consumed
                    raw_args[arg_name] = raw
                    args_out[arg_name] = spec["mapping"].get(raw, f"UNKNOWN({raw})")

                elif arg_type == "bool":
                    raw, consumed = MdbParsingService._decode_int(
                        remaining, spec["size_bits"], False, spec["byte_order"]
                    )
                    pos += consumed
                    raw_args[arg_name] = raw
                    args_out[arg_name] = bool(raw)

                elif arg_type == "int":
                    raw, consumed = MdbParsingService._decode_int(
                        remaining, spec["size_bits"], spec["signed"], spec["byte_order"]
                    )
                    pos += consumed
                    raw_args[arg_name] = raw
                    args_out[arg_name] = raw

                elif arg_type == "float":
                    raw, consumed = MdbParsingService._decode_float(remaining, spec["size_bits"], spec["byte_order"])
                    pos += consumed
                    raw_args[arg_name] = raw
                    args_out[arg_name] = raw

                elif arg_type == "string":
                    raw, consumed = MdbParsingService._decode_string(remaining, spec["termination_char"])
                    pos += consumed
                    raw_args[arg_name] = raw
                    args_out[arg_name] = raw

                else:
                    raise ValueError(f"Unsupported decoded argument kind: {arg_type}")

            return {
                "name": cmd["name"],
                "full_name": cmd["full_name"],
                "root_system": cmd["root_system"],
                "arguments": args_out,
                "raw_arguments": raw_args,
            }

        return None

    @staticmethod
    def _decode_int(data: bytes, size_bits: int, signed: bool = False, byte_order: str = "big") -> tuple[int, int]:
        size_bytes = size_bits // 8
        if len(data) < size_bytes:
            raise ValueError("Not enough bytes to decode integer")
        value = int.from_bytes(data[:size_bytes], byteorder=byte_order, signed=signed)
        return value, size_bytes

    @staticmethod
    def _decode_float(data: bytes, size_bits: int, byte_order: str = "big") -> tuple[float, int]:
        if size_bits == 32:
            fmt = ">f" if byte_order == "big" else "<f"
            size_bytes = 4
        elif size_bits == 64:
            fmt = ">d" if byte_order == "big" else "<d"
            size_bytes = 8
        else:
            raise ValueError(f"Unsupported float size: {size_bits} bits")

        if len(data) < size_bytes:
            raise ValueError("Not enough bytes to decode float")

        value = struct.unpack(fmt, data[:size_bytes])[0]
        return value, size_bytes

    @staticmethod
    def _decode_string(data: bytes, termination_char: int | None) -> tuple[str, int]:
        if termination_char is None:
            raise ValueError("String argument missing termination char")

        try:
            end_idx = data.index(bytes([termination_char]))
        except ValueError as exc:
            raise ValueError("String terminator not found in payload") from exc

        value = data[:end_idx].decode("ascii")
        return value, end_idx + 1


def _tag(element) -> str:
    """Element name without its XML namespace."""
    return element.tag.rsplit("}", 1)[-1]


def _children(element, name: str) -> list:
    return [child for child in element if _tag(child) == name]


def _child(element, name: str):
    return next(iter(_children(element, name)), None)


def _byte_order(encoding) -> str:
    return "little" if encoding.attrib.get("byteOrder") == "leastSignificantByteFirst" else "big"


def _resolve_ref(ref: str, path: str, type_table: dict) -> str | None:
    """Qualified name of an XTCE type reference made from space system `path`.

    Absolute ("/Rover/x") and relative ("../x", "sub/x") references resolve directly; a bare name is looked up
    in `path`, then in each parent space system (as Yamcs does).
    """
    if ref.startswith("/"):
        candidates = [posixpath.normpath(ref)]
    elif "/" in ref:
        candidates = [posixpath.normpath(f"{path}/{ref}")]
    else:
        parts = path.strip("/").split("/")
        candidates = ["/" + "/".join(parts[:i] + [ref]) for i in range(len(parts), 0, -1)]
    return next((name for name in candidates if name in type_table), None)
