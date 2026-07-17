"""CI1302 V5 command and passive-broadcast protocol helpers.

This module has no ROS dependency so the byte-level contract can be tested on
Windows and in CI.  Command IDs 0x01..0x30 and broadcast IDs 0x80..0xDA match
``Eggy小车_CI1302语音命令词播报词协议表_V5.xlsx``.
"""

from collections import namedtuple
import json


VoiceCommand = namedtuple("VoiceCommand", "function_id command_id action")


COMMANDS = {
    0x01: "emergency_stop",
    0x02: "cancel_task",
    0x03: "move_forward",
    0x04: "move_backward",
    0x05: "move_left",
    0x06: "move_right",
    0x07: "rotate_left",
    0x08: "rotate_right",
    0x09: "query_status",
    0x0A: "query_battery",
    0x0B: "query_profile",
    0x0C: "query_task",
    0x0D: "query_environment",
    0x0E: "camera_start",
    0x0F: "camera_stop",
    0x10: "profile_mapping",
    0x11: "profile_navigation",
    0x12: "profile_inspection",
    0x13: "mapping_start",
    0x14: "mapping_stop",
    0x15: "mapping_reset",
    0x16: "clear_costmaps",
    0x17: "return_home",
    0x18: "navigate_point_1",
    0x19: "navigate_point_2",
    0x1A: "navigate_point_3",
    0x1B: "navigate_point_4",
    0x1C: "start_navigation",
    0x1D: "start_inspection",
    0x1E: "recognize_any_meter",
    0x1F: "recognize_water_meter",
    0x20: "recognize_pressure_gauge",
    0x21: "recognize_retry",
    0x22: "repeat_result",
    0x23: "auto_mapping_start",
    0x24: "auto_mapping_pause",
    0x25: "auto_mapping_resume",
    0x26: "auto_mapping_stop",
    0x27: "auto_mapping_status",
    0x28: "save_map",
    0x29: "ai_inspection_enable",
    0x2A: "ai_inspection_disable",
    0x2B: "query_network",
    0x2C: "relocalize",
    0x2D: "pause_task",
    0x2E: "resume_task",
    0x2F: "query_camera",
    0x30: "query_safety",
}


BROADCAST = {
    "system_ready": 0x80,
    "accepted": 0x81,
    "cannot_execute": 0x82,
    "stopped": 0x83,
    "emergency_locked": 0x84,
    "safety_insufficient": 0x85,
    "action_complete": 0x86,
    "status_ok": 0x87,
    "status_abnormal": 0x88,
    "battery_ok": 0x89,
    "battery_low": 0x8A,
    "battery_critical": 0x8B,
    "profile_mapping": 0x8C,
    "profile_navigation": 0x8D,
    "profile_inspection": 0x8E,
    "task_idle": 0x8F,
    "task_running": 0x90,
    "task_cancelled": 0x91,
    "route_started": 0x92,
    "going_to_target": 0x93,
    "arrived": 0x94,
    "navigation_failed": 0x95,
    "path_blocked": 0x96,
    "returning_home": 0x97,
    "returned_home": 0x98,
    "mapping_started": 0x99,
    "mapping_stopped": 0x9A,
    "mapping_reset": 0x9B,
    "map_saved": 0x9C,
    "costmaps_cleared": 0x9D,
    "camera_online": 0x9E,
    "camera_closed": 0x9F,
    "inspection_started": 0xA0,
    "searching_target": 0xA1,
    "target_found": 0xA2,
    "aligning": 0xA3,
    "target_aligned": 0xA4,
    "recognizing": 0xA5,
    "water_meter_found": 0xA6,
    "pressure_gauge_found": 0xA7,
    "recognition_ok": 0xA8,
    "recognition_failed": 0xA9,
    "reading_ok": 0xAA,
    "reading_abnormal": 0xAB,
    "ai_running": 0xAC,
    "ai_complete": 0xAD,
    "ai_failed": 0xAE,
    "network_ok": 0xAF,
    "network_lost": 0xB0,
    "localization_ready": 0xB1,
    "localization_lost": 0xB2,
    "lidar_abnormal": 0xB3,
    "base_lost": 0xB4,
    "voice_ready": 0xB5,
    "micro_move_complete": 0xB6,
    "obstacle_stopped": 0xB7,
    "waiting_command": 0xB8,
    "task_complete": 0xB9,
    "inspection_complete": 0xBA,
    "temperature_ok": 0xBB,
    "temperature_abnormal": 0xBC,
    "humidity_ok": 0xBD,
    "humidity_abnormal": 0xBE,
    "no_route": 0xBF,
    "auto_mapping_started": 0xC0,
    "auto_mapping_paused": 0xC1,
    "auto_mapping_resumed": 0xC2,
    "auto_mapping_stopped": 0xC3,
    "safety_checking": 0xC4,
    "frontier_selecting": 0xC5,
    "exploring": 0xC6,
    "map_updating": 0xC7,
    "final_scan": 0xC8,
    "draft_saved": 0xC9,
    "auto_mapping_complete": 0xCA,
    "auto_mapping_failed": 0xCB,
    "ai_inspection_enabled": 0xCC,
    "ai_inspection_disabled": 0xCD,
    "wifi_connected": 0xCE,
    "mobile_connected": 0xCF,
    "remote_connected": 0xD0,
    "network_abnormal": 0xD1,
    "task_paused": 0xD2,
    "task_resumed": 0xD3,
    "relocalizing": 0xD4,
    "relocalize_ok": 0xD5,
    "relocalize_failed": 0xD6,
    "camera_ready": 0xD7,
    "camera_abnormal": 0xD8,
    "safety_ok": 0xD9,
    "safety_abnormal": 0xDA,
}


def _hex_byte(value, field):
    text = str(value).strip()
    if len(text) != 2:
        raise ValueError("%s must be a two-digit hex byte" % field)
    try:
        result = int(text, 16)
    except ValueError:
        raise ValueError("%s must be hexadecimal" % field)
    if not 0 <= result <= 0xFF:
        raise ValueError("%s is outside byte range" % field)
    return result


def decode_voice_command(text):
    try:
        payload = json.loads(text)
    except (TypeError, ValueError) as exc:
        raise ValueError("voice command must be JSON: %s" % exc)
    if not isinstance(payload, dict):
        raise ValueError("voice command must be an object")
    function_id = _hex_byte(payload.get("func", ""), "func")
    command_id = _hex_byte(payload.get("cmd", ""), "cmd")
    if function_id != 0x00:
        raise ValueError("voice command function must be 00")
    if command_id not in COMMANDS:
        raise ValueError("unsupported V5 voice command: %02X" % command_id)
    return VoiceCommand(function_id, command_id, COMMANDS[command_id])


def encode_playback_request(broadcast_id):
    broadcast_id = int(broadcast_id)
    if not 0 <= broadcast_id <= 0xFF:
        raise ValueError("broadcast id is outside byte range")
    return json.dumps({
        "func": "FF",
        "cmd": "%02X" % broadcast_id,
        "frame": "AA 55 FF %02X FB" % broadcast_id,
    }, ensure_ascii=False, separators=(",", ":"))


def decode_playback_request(text):
    try:
        payload = json.loads(text)
    except (TypeError, ValueError) as exc:
        raise ValueError("playback request must be JSON: %s" % exc)
    if not isinstance(payload, dict):
        raise ValueError("playback request must be an object")
    function_id = _hex_byte(payload.get("func", ""), "func")
    broadcast_id = _hex_byte(payload.get("cmd", ""), "cmd")
    if function_id != 0xFF:
        raise ValueError("playback function must be FF")
    if not 0x80 <= broadcast_id <= 0xDA:
        raise ValueError("unsupported V5 broadcast: %02X" % broadcast_id)
    return broadcast_id


def build_playback_frame(broadcast_id):
    broadcast_id = int(broadcast_id)
    if not 0x80 <= broadcast_id <= 0xDA:
        raise ValueError("broadcast id must be in the V5 range 80..DA")
    return bytes((0xAA, 0x55, 0xFF, broadcast_id, 0xFB))


def select_battery_broadcast(voltage):
    voltage = float(voltage)
    if voltage < 10.5:
        return BROADCAST["battery_critical"]
    if voltage < 11.0:
        return BROADCAST["battery_low"]
    return BROADCAST["battery_ok"]


def select_profile_broadcast(profile):
    return {
        "mapping": BROADCAST["profile_mapping"],
        "navigation": BROADCAST["profile_navigation"],
        "inspection": BROADCAST["profile_inspection"],
    }.get(str(profile).strip().lower(), BROADCAST["status_abnormal"])
