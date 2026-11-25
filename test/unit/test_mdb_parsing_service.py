"""Unit tests for the XTCE parsing of the Yamcs server MDB in src/tmtc/mdb_parsing_service.py (no Isaac Sim required).

python -m pytest -v test/unit/test_mdb_parsing_service.py
"""

import struct

import pytest

from src.tmtc.mdb_parsing_service import MdbParsingService, ServerMdb

# Layout of a space system as Yamcs (5.13) exports it with exportXTCE: default namespace, wire sizes only in the
# data encodings, upper-case hex, nested space systems, and an argument type defined in a parent space system.
ROVER_XTCE = """<?xml version="1.0" encoding="UTF-8"?>
<SpaceSystem xmlns="http://www.omg.org/spec/XTCE/20180204" name="Rover">
  <TelemetryMetaData>
    <ParameterTypeSet>
      <IntegerParameterType sizeInBits="8" signed="false" name="battery_charge"/>
    </ParameterTypeSet>
    <ParameterSet>
      <Parameter parameterTypeRef="battery_charge" name="battery_charge"/>
    </ParameterSet>
  </TelemetryMetaData>
  <CommandMetaData>
    <ArgumentTypeSet>
      <EnumeratedArgumentType name="camera_name">
        <IntegerDataEncoding encoding="unsigned" sizeInBits="8"/>
        <EnumerationList>
          <Enumeration label="NavCam_left" value="0"/>
          <Enumeration label="NavCam_right" value="1"/>
        </EnumerationList>
      </EnumeratedArgumentType>
    </ArgumentTypeSet>
  </CommandMetaData>
  <SpaceSystem name="camera">
    <CommandMetaData>
      <ArgumentTypeSet>
        <IntegerArgumentType sizeInBits="32" signed="false" name="bitrate">
          <IntegerDataEncoding encoding="unsigned" sizeInBits="32"/>
        </IntegerArgumentType>
        <IntegerArgumentType signed="true" name="crf">
          <IntegerDataEncoding encoding="twosComplement" sizeInBits="8"/>
        </IntegerArgumentType>
        <FloatArgumentType sizeInBits="32" name="window">
          <FloatDataEncoding encoding="IEEE754_1985" sizeInBits="32"/>
        </FloatArgumentType>
        <StringArgumentType name="path">
          <StringDataEncoding encoding="US-ASCII">
            <Variable maxSizeInBits="8388608">
              <DynamicValue><ParameterInstanceRef parameterRef="_yamcs_ignore"/></DynamicValue>
              <TerminationChar>00</TerminationChar>
            </Variable>
          </StringDataEncoding>
        </StringArgumentType>
        <BooleanArgumentType name="flag">
          <IntegerDataEncoding sizeInBits="8"/>
        </BooleanArgumentType>
      </ArgumentTypeSet>
      <MetaCommandSet>
        <MetaCommand name="set_camera_streaming_params">
          <ArgumentList>
            <Argument argumentTypeRef="camera_name" name="camera_name"/>
            <Argument argumentTypeRef="bitrate" name="bitrate"/>
            <Argument argumentTypeRef="crf" name="crf"/>
            <Argument argumentTypeRef="window" name="window"/>
            <Argument argumentTypeRef="path" name="path"/>
            <Argument argumentTypeRef="flag" name="flag"/>
          </ArgumentList>
          <CommandContainer name="set_camera_streaming_params">
            <EntryList>
              <FixedValueEntry name="command_name" binaryValue="7365745F706172616D73" sizeInBits="80"/>
              <ArgumentRefEntry argumentRef="camera_name"/>
            </EntryList>
          </CommandContainer>
        </MetaCommand>
        <MetaCommand name="camera_capture_high">
          <ArgumentList>
            <Argument argumentTypeRef="/Rover/camera_name" name="camera_name"/>
          </ArgumentList>
          <CommandContainer name="camera_capture_high">
            <EntryList>
              <FixedValueEntry name="command_name" binaryValue="63617074757265" sizeInBits="56"/>
            </EntryList>
          </CommandContainer>
        </MetaCommand>
        <MetaCommand name="abstract_base" abstract="true"/>
      </MetaCommandSet>
    </CommandMetaData>
    <SpaceSystem name="NavCam_left">
      <SpaceSystem name="streaming">
        <TelemetryMetaData>
          <ParameterSet>
            <Parameter parameterTypeRef="/Rover/battery_charge" name="number"/>
            <Parameter parameterTypeRef="/Rover/battery_charge" name="state"/>
          </ParameterSet>
        </TelemetryMetaData>
      </SpaceSystem>
    </SpaceSystem>
  </SpaceSystem>
</SpaceSystem>
"""

LANDER_XTCE = """<SpaceSystem xmlns="http://www.omg.org/spec/XTCE/20180204" name="Lander">
  <CommandMetaData>
    <MetaCommandSet>
      <MetaCommand name="lander_camera_capture">
        <CommandContainer name="lander_camera_capture">
          <EntryList><FixedValueEntry binaryValue="6C616E646572" sizeInBits="48"/></EntryList>
        </CommandContainer>
      </MetaCommand>
    </MetaCommandSet>
  </CommandMetaData>
</SpaceSystem>
"""


@pytest.fixture
def rover():
    return MdbParsingService.parse_xtce(ROVER_XTCE, "/Rover")


def test_space_systems_and_parameters(rover):
    assert rover.space_systems == {
        "/Rover",
        "/Rover/camera",
        "/Rover/camera/NavCam_left",
        "/Rover/camera/NavCam_left/streaming",
    }
    assert rover.parameters == {
        "/Rover/battery_charge",
        "/Rover/camera/NavCam_left/streaming/number",
        "/Rover/camera/NavCam_left/streaming/state",
    }


def test_commands_and_argument_specs(rover):
    assert set(rover.commands) == {
        "/Rover/camera/set_camera_streaming_params",
        "/Rover/camera/camera_capture_high",
        "/Rover/camera/abstract_base",
    }
    args = {arg["name"]: arg for arg in rover.commands["/Rover/camera/set_camera_streaming_params"]["args"]}
    assert list(args) == ["camera_name", "bitrate", "crf", "window", "path", "flag"]
    # Unqualified reference resolved in the parent space system
    assert args["camera_name"]["type"] == "/Rover/camera_name"
    assert args["camera_name"]["spec"]["mapping"] == {0: "NavCam_left", 1: "NavCam_right"}
    assert args["bitrate"]["spec"] == {"kind": "int", "size_bits": 32, "byte_order": "big", "signed": False}
    assert args["crf"]["spec"] == {"kind": "int", "size_bits": 8, "byte_order": "big", "signed": True}
    assert args["window"]["spec"] == {"kind": "float", "size_bits": 32, "byte_order": "big"}
    assert args["path"]["spec"] == {"kind": "string", "termination_char": 0}
    assert args["flag"]["spec"]["kind"] == "bool"
    capture_args = rover.commands["/Rover/camera/camera_capture_high"]["args"]
    assert capture_args[0]["type"] == "/Rover/camera_name"


def test_registry_holds_commands_with_a_wire_name(rover):
    assert set(rover.registry) == {b"set_params", b"capture"}
    assert rover.registry[b"capture"]["full_name"] == "/Rover/camera/camera_capture_high"
    assert rover.registry[b"capture"]["root_system"] == "Rover"


def test_decode_tc_payload(rover):
    payload = (
        b"set_params"
        + bytes([1])
        + (150_000).to_bytes(4, "big")
        + (-1).to_bytes(1, "big", signed=True)
        + struct.pack(">f", 2.5)
        + b"cfg.yaml\x00"
        + bytes([1])
    )
    decoded = MdbParsingService.decode_tc_payload(payload, rover.registry)
    assert decoded["full_name"] == "/Rover/camera/set_camera_streaming_params"
    assert decoded["arguments"] == {
        "camera_name": "NavCam_right",
        "bitrate": 150_000,
        "crf": -1,
        "window": 2.5,
        "path": "cfg.yaml",
        "flag": True,
    }
    assert MdbParsingService.decode_tc_payload(b"unknown", rover.registry) is None


def test_parse_nested_space_system_uses_its_qualified_name():
    # Exporting /Rover/camera: the root element only carries its own name; types may be referenced relatively.
    xml = """<SpaceSystem xmlns="http://www.omg.org/spec/XTCE/20180204" name="camera">
      <CommandMetaData>
        <MetaCommandSet>
          <MetaCommand name="camera_capture_depth">
            <ArgumentList><Argument argumentTypeRef="types/camera_name" name="camera_name"/></ArgumentList>
          </MetaCommand>
        </MetaCommandSet>
      </CommandMetaData>
      <SpaceSystem name="types">
        <CommandMetaData>
          <ArgumentTypeSet>
            <EnumeratedArgumentType name="camera_name">
              <IntegerDataEncoding sizeInBits="8"/>
              <EnumerationList><Enumeration label="cam_depth" value="0"/></EnumerationList>
            </EnumeratedArgumentType>
          </ArgumentTypeSet>
        </CommandMetaData>
      </SpaceSystem>
    </SpaceSystem>"""
    mdb = MdbParsingService.parse_xtce(xml, "/Rover/camera")
    assert mdb.space_systems == {"/Rover/camera", "/Rover/camera/types"}
    arg = mdb.commands["/Rover/camera/camera_capture_depth"]["args"][0]
    assert arg["type"] == "/Rover/camera/types/camera_name"
    assert arg["spec"]["mapping"] == {0: "cam_depth"}
    assert mdb.registry == {}  # no FixedValueEntry: not decodable from UDP


def test_unknown_argument_type_is_an_error():
    xml = ROVER_XTCE.replace('argumentTypeRef="bitrate"', 'argumentTypeRef="no_such_type"')
    with pytest.raises(ValueError, match="no_such_type"):
        MdbParsingService.parse_xtce(xml, "/Rover")


class FakeMdbClient:
    def __init__(self, exports):
        self.exports = exports

    def export_space_system(self, name):
        return self.exports[name]


def test_load_from_server_merges_space_systems():
    client = FakeMdbClient({"/Rover": ROVER_XTCE, "/Lander": LANDER_XTCE})
    mdb = MdbParsingService.load_from_server(client, ["/Rover", "/Lander"])
    assert {"/Rover", "/Lander"} <= mdb.space_systems
    assert "/Lander/lander_camera_capture" in mdb.commands
    assert set(mdb.registry) == {b"set_params", b"capture", b"lander"}


def test_load_from_server_rejects_ambiguous_wire_names():
    lander = LANDER_XTCE.replace("6C616E646572", "63617074757265")  # same wire name as /Rover camera_capture_high
    with pytest.raises(ValueError, match="Ambiguous command wire name"):
        MdbParsingService.load_from_server(
            FakeMdbClient({"/Rover": ROVER_XTCE, "/Lander": lander}), ["/Rover", "/Lander"]
        )


def test_merge_keeps_one_command_exported_twice():
    mdb = ServerMdb()
    mdb.merge(MdbParsingService.parse_xtce(ROVER_XTCE, "/Rover"))
    mdb.merge(MdbParsingService.parse_xtce(ROVER_XTCE, "/Rover"))
    assert len(mdb.registry) == 2
