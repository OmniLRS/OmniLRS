"""Arguments each Pragyaan command handler takes, by key of the controller yaml `commands:` section.

Single list used by PragyaanController.setup_command_callbacks (handler arguments, in call order) and by the Yamcs
contract check (src/tmtc/yamcs_contract.py), which verifies the server MDB command has every argument.
Standard library only: the standalone checker (scripts/check_yamcs_contract.py) imports it without Isaac Sim.
"""

COMMAND_ARGS = {
    "drive_straight": ["linear_velocity", "distance"],
    "drive_turn": ["angular_velocity", "angle"],
    "camera_capture_high": ["camera_name"],
    "camera_streaming_on_off": ["camera_name", "action"],
    "camera_capture_depth": ["camera_name"],
    "set_camera_streaming_params": [
        "camera_name",
        "bitrate",
        "crf",
        "keyframe_interval",
        "max_payload_size",
        "bandwidth_window_seconds",
    ],
    "power_electronics": ["subsystem_id", "power_state"],
    "solar_panel": ["deployment"],
    "go_nogo": ["decision"],
    "capture_apxs": [],
    "admin_water_detection": ["trigger_water_detection"],
    "admin_inject_fault": [],
    "lander_camera_capture_high": [],
    "admin_battery_percentage": ["battery_percentage"],
}
