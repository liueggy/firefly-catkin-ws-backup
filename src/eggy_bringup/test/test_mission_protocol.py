#!/usr/bin/env python3
import os
import sys
import unittest

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPTS)

from mission_protocol import build_goal_pose_mission, normalize_mission_request


class MissionProtocolTest(unittest.TestCase):
    def test_navigation_accepts_single_and_multi_point_routes(self):
        for size in (1, 3):
            request = normalize_mission_request({
                "request_id": "nav-%d" % size,
                "mission_type": "navigation",
                "inspection": {"enabled": False},
                "return_home": False,
                "route": [{"id": str(i), "x": i, "y": i + 1, "yaw": 0}
                          for i in range(size)],
            })
            self.assertEqual(size, len(request["route"]))
            self.assertFalse(request["inspection"]["enabled"])

    def test_legacy_inspection_request_enables_inspection(self):
        request = normalize_mission_request(
            {"route": [{"x": 1, "y": 2}]}, legacy_inspection=True)
        self.assertTrue(request["inspection"]["enabled"])
        self.assertEqual("inspection", request["mission_type"])

    def test_return_home_is_strict_boolean(self):
        with self.assertRaises(ValueError):
            normalize_mission_request({
                "return_home": "false",
                "route": [{"x": 1, "y": 2}],
            })

    def test_goal_pose_is_always_plain_navigation(self):
        request = build_goal_pose_mission("map", 1.2, -0.5, 0.25, "goal-1")
        self.assertEqual("goal-1", request["request_id"])
        self.assertEqual("navigation", request["mission_type"])
        self.assertFalse(request["inspection"]["enabled"])
        self.assertFalse(request["return_home"])
        self.assertEqual(1, len(request["route"]))

    def test_cancel_requires_identity(self):
        with self.assertRaises(ValueError):
            normalize_mission_request({"command": "cancel"})
        cancel = normalize_mission_request(
            {"command": "cancel", "request_id": "active-1"})
        self.assertEqual("active-1", cancel["request_id"])

    def test_rejects_unknown_schema_and_mission_type(self):
        route = [{"id": "a", "x": 0, "y": 0}]
        with self.assertRaises(ValueError):
            normalize_mission_request({"schema_version": 2, "route": route})
        with self.assertRaises(ValueError):
            normalize_mission_request({"mission_type": "patrol", "route": route})

    def test_rejects_duplicate_waypoint_ids(self):
        with self.assertRaises(ValueError):
            normalize_mission_request({
                "route": [
                    {"id": "same", "x": 0, "y": 0},
                    {"id": "same", "x": 1, "y": 1},
                ],
            })


if __name__ == "__main__":
    unittest.main()
