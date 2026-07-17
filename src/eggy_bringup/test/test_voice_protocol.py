#!/usr/bin/env python3
import json
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, PACKAGE_SRC)

from eggy_bringup.voice_protocol import (  # noqa: E402
    BROADCAST,
    COMMANDS,
    build_playback_frame,
    decode_playback_request,
    decode_voice_command,
    encode_playback_request,
    select_battery_broadcast,
    select_profile_broadcast,
)


class VoiceProtocolTest(unittest.TestCase):
    def test_v5_command_range_is_complete_and_unique(self):
        self.assertEqual(set(range(0x01, 0x31)), set(COMMANDS))
        self.assertEqual(len(COMMANDS), len(set(COMMANDS)))

    def test_v5_broadcast_range_is_complete_and_unique(self):
        self.assertEqual(set(range(0x80, 0xDB)), set(BROADCAST.values()))
        self.assertEqual(len(BROADCAST), len(set(BROADCAST.values())))

    def test_decodes_stm32_forwarded_voice_json(self):
        command = decode_voice_command('{"func":"00","cmd":"23"}')
        self.assertEqual(0x23, command.command_id)
        self.assertEqual("auto_mapping_start", command.action)

    def test_rejects_non_command_function_and_unknown_id(self):
        with self.assertRaises(ValueError):
            decode_voice_command('{"func":"FF","cmd":"80"}')
        with self.assertRaises(ValueError):
            decode_voice_command('{"func":"00","cmd":"31"}')

    def test_playback_request_matches_ci1302_frame(self):
        request = json.loads(encode_playback_request(BROADCAST["auto_mapping_started"]))
        self.assertEqual("FF", request["func"])
        self.assertEqual("C0", request["cmd"])
        self.assertEqual("AA 55 FF C0 FB", request["frame"])
        self.assertEqual(0xC0, decode_playback_request(json.dumps(request)))
        self.assertEqual(bytes((0xAA, 0x55, 0xFF, 0xC0, 0xFB)),
                         build_playback_frame(0xC0))

    def test_rejects_out_of_range_playback(self):
        with self.assertRaises(ValueError):
            decode_playback_request('{"func":"00","cmd":"C0"}')
        with self.assertRaises(ValueError):
            decode_playback_request('{"func":"FF","cmd":"DB"}')
        with self.assertRaises(ValueError):
            build_playback_frame(0x7F)

    def test_status_broadcast_selection(self):
        self.assertEqual(BROADCAST["battery_ok"], select_battery_broadcast(12.1))
        self.assertEqual(BROADCAST["battery_low"], select_battery_broadcast(10.8))
        self.assertEqual(BROADCAST["battery_critical"], select_battery_broadcast(10.2))
        self.assertEqual(BROADCAST["profile_mapping"], select_profile_broadcast("mapping"))
        self.assertEqual(BROADCAST["profile_navigation"], select_profile_broadcast("navigation"))
        self.assertEqual(BROADCAST["profile_inspection"], select_profile_broadcast("inspection"))
        self.assertEqual(BROADCAST["status_abnormal"], select_profile_broadcast("unknown"))


if __name__ == "__main__":
    unittest.main()
