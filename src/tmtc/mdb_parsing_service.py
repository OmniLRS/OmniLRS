import binascii
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

class MdbParsingService():

    XTCE_NS = {"xtce": "http://www.omg.org/spec/XTCE/20180204"}

    @classmethod
    def load_mdb_registry(cls, mdb_dir):
        registry = {}
        mdb_dir = Path(mdb_dir)

        for xml_file in sorted(mdb_dir.glob("*.xml")):
            file_commands = cls._parse_mdb_file(xml_file)

            for wire_name, spec in file_commands.items():
                if wire_name in registry:
                    raise ValueError(
                        f"Ambiguous command wire name across MDB files: {wire_name!r}\n"
                        f"Existing: {registry[wire_name]['full_name']} ({registry[wire_name]['source_file']})\n"
                        f"New:      {spec['full_name']} ({spec['source_file']})\n"
                        "UDP payload alone cannot disambiguate these commands."
                    )
                registry[wire_name] = spec

        return registry

    @classmethod
    def _parse_mdb_file(cls, mdb_path):
        tree = ET.parse(mdb_path)
        root = tree.getroot()
        parent_map = MdbParsingService._find_parent_map(root)

        command_table = {}
        type_table = {}

        # -------- Parse enum argument types --------
        for enum_type in root.findall(".//xtce:EnumeratedArgumentType", cls.XTCE_NS):
            type_name = enum_type.attrib["name"]
            mapping = {}

            enc = enum_type.find("xtce:IntegerDataEncoding", cls.XTCE_NS)
            size_bits = int(enc.attrib.get("sizeInBits", "8")) if enc is not None else 8

            for enum in enum_type.findall(".//xtce:Enumeration", cls.XTCE_NS):
                val = int(enum.attrib["value"])
                label = enum.attrib["label"]
                mapping[val] = label

            type_table[type_name] = {
                "kind": "enum",
                "size_bits": size_bits,
                "mapping": mapping,
            }

        # -------- Parse integer argument types --------
        for int_type in root.findall(".//xtce:IntegerArgumentType", cls.XTCE_NS):
            type_name = int_type.attrib["name"]
            size_bits = int(int_type.attrib["sizeInBits"])
            signed = int_type.attrib.get("signed", "false").lower() == "true"

            type_table[type_name] = {
                "kind": "int",
                "size_bits": size_bits,
                "signed": signed,
            }

        # -------- Parse boolean argument types --------
        for bool_type in root.findall(".//xtce:BooleanArgumentType", cls.XTCE_NS):
            type_name = bool_type.attrib["name"]

            enc = bool_type.find("xtce:IntegerDataEncoding", cls.XTCE_NS)
            size_bits = int(enc.attrib.get("sizeInBits", "8")) if enc is not None else 8

            zero_string = bool_type.attrib.get("zeroStringValue", "False")
            one_string = bool_type.attrib.get("oneStringValue", "True")

            type_table[type_name] = {
                "kind": "bool",
                "size_bits": size_bits,
                "mapping": {
                    0: zero_string,
                    1: one_string,
                },
            }

        # -------- Parse float argument types --------
        for float_type in root.findall(".//xtce:FloatArgumentType", cls.XTCE_NS):
            type_name = float_type.attrib["name"]

            enc = float_type.find("xtce:FloatDataEncoding", cls.XTCE_NS)
            size_bits = int(enc.attrib.get("sizeInBits", float_type.attrib.get("sizeInBits", "32"))) if enc is not None else int(float_type.attrib.get("sizeInBits", "32"))
            byte_order = "big"
            if enc is not None:
                raw_order = enc.attrib.get("byteOrder", "mostSignificantByteFirst")
                byte_order = "big" if raw_order == "mostSignificantByteFirst" else "little"

            type_table[type_name] = {
                "kind": "float",
                "size_bits": size_bits,
                "byte_order": byte_order,
            }

        # -------- Parse string argument types --------
        for string_type in root.findall(".//xtce:StringArgumentType", cls.XTCE_NS):
            type_name = string_type.attrib["name"]
            termination_char = None

            term = string_type.find(".//xtce:TerminationChar", cls.XTCE_NS)
            if term is not None and term.text:
                termination_char = int(term.text.strip(), 16)

            type_table[type_name] = {
                "kind": "string",
                "termination_char": termination_char,
            }

        # -------- Parse commands --------
        for cmd in root.findall(".//xtce:MetaCommand", cls.XTCE_NS):
            cmd_name = cmd.attrib["name"]
            full_name = MdbParsingService._build_full_command_path(cmd, parent_map)
            root_system = MdbParsingService._get_root_system(full_name)

            # argument list
            args = []
            arg_list = cmd.find("xtce:ArgumentList", cls.XTCE_NS)
            if arg_list is not None:
                for arg in arg_list.findall("xtce:Argument", cls.XTCE_NS):
                    arg_name = arg.attrib["name"]
                    arg_type = arg.attrib["argumentTypeRef"]

                    if arg_type not in type_table:
                        raise ValueError(
                            f"Unsupported or unknown argument type: {arg_type} "
                            f"in command '{full_name}' from file '{mdb_path}'"
                        )

                    args.append({
                        "name": arg_name,
                        "type": arg_type,
                        "spec": type_table[arg_type],
                    })

            # wire command name
            fixed = cmd.find(".//xtce:FixedValueEntry", cls.XTCE_NS)
            if fixed is None:
                continue

            hex_name = fixed.attrib["binaryValue"]
            wire_name = binascii.unhexlify(hex_name)

            if wire_name in command_table:
                raise ValueError(
                    f"Duplicate wire command name {wire_name!r} inside file '{mdb_path}'. "
                    f"Commands: '{command_table[wire_name]['full_name']}' and '{full_name}'"
                )

            command_table[wire_name] = {
                "name": cmd_name,
                "full_name": full_name,
                "root_system": root_system,
                "source_file": str(mdb_path),
                "args": args,
            }

        return command_table
    
    @staticmethod
    def _find_parent_map(root):
        return {child: parent for parent in root.iter() for child in parent}

    @staticmethod
    def _build_full_command_path(meta_command, parent_map):
        parts = [meta_command.attrib["name"]]

        current = meta_command
        while current in parent_map:
            current = parent_map[current]
            if current.tag.endswith("SpaceSystem"):
                parts.append(current.attrib["name"])

        parts.reverse()
        return "/" + "/".join(parts)

    @staticmethod
    def _get_root_system(full_name: str) -> str:
        # "/Rover/system/go_nogo_command" -> "Rover"
        parts = full_name.strip("/").split("/")
        return parts[0] if parts else ""
    
    @staticmethod
    def decode_tc_payload( payload, registry):
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
                    raw, consumed = MdbParsingService._decode_int(remaining, spec["size_bits"], signed=False)
                    pos += consumed
                    raw_args[arg_name] = raw
                    args_out[arg_name] = spec["mapping"].get(raw, f"UNKNOWN({raw})")

                elif arg_type == "bool":
                    raw, consumed = MdbParsingService._decode_int(remaining, spec["size_bits"], signed=False)
                    pos += consumed
                    raw_args[arg_name] = raw
                    args_out[arg_name] = bool(raw)

                elif arg_type == "int":
                    raw, consumed = MdbParsingService._decode_int(remaining, spec["size_bits"], signed=spec["signed"])
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
                "source_file": cmd["source_file"],
                "arguments": args_out,
                "raw_arguments": raw_args,
            }

        return None

    @staticmethod
    def _decode_int(data: bytes, size_bits: int, signed: bool = False) -> tuple[int, int]:
        size_bytes = size_bits // 8
        if len(data) < size_bytes:
            raise ValueError("Not enough bytes to decode integer")
        value = int.from_bytes(data[:size_bytes], byteorder="big", signed=signed)
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