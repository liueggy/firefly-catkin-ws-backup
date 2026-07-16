import math
import os
import sys
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))

from eggy_bringup.auto_mapping_core import (  # noqa: E402
    completion_decision,
    dynamic_stop_distance,
    extract_frontier_clusters,
    rank_frontiers,
    safe_mapping_twist,
    validate_mapping_request,
)


class AutoMappingCoreTest(unittest.TestCase):
    def test_frontier_clusters_are_extracted_and_ranked(self):
        width = 12
        height = 10
        data = [-1] * (width * height)
        for y in range(2, 8):
            for x in range(2, 7):
                data[y * width + x] = 0
        data[4 * width + 4] = 100

        clusters = extract_frontier_clusters(
            data, width, height, resolution=0.1,
            origin_x=-0.6, origin_y=-0.5,
            min_cluster_cells=2,
        )
        self.assertGreaterEqual(len(clusters), 1)
        self.assertTrue(all(item["cell_count"] >= 2 for item in clusters))

        ranked = rank_frontiers(clusters, robot_xy=(0.0, 0.0))
        self.assertEqual(len(ranked), len(clusters))
        self.assertGreaterEqual(ranked[0]["score"], ranked[-1]["score"])

    def test_failed_and_visited_frontiers_are_penalized(self):
        candidates = [
            {"x": 1.0, "y": 0.0, "cell_count": 20, "information_gain": 0.2, "clearance": 0.8},
            {"x": 2.0, "y": 0.0, "cell_count": 20, "information_gain": 0.2, "clearance": 0.8},
        ]
        ranked = rank_frontiers(
            candidates,
            robot_xy=(0.0, 0.0),
            failed_points=[(1.0, 0.0)],
            visited_points=[(1.0, 0.0)],
        )
        self.assertEqual(2.0, ranked[0]["x"])

    def test_dynamic_stopping_distance_increases_with_speed(self):
        slow = dynamic_stop_distance(0.1, 0.18, 0.15, 0.5, 0.05)
        fast = dynamic_stop_distance(0.3, 0.18, 0.15, 0.5, 0.05)
        self.assertGreater(fast, slow)

    def test_safety_gate_blocks_each_motion_direction(self):
        clear = {"front": 2.0, "rear": 2.0, "left": 2.0, "right": 2.0, "rotation": 2.0}
        vx, vy, wz, reason = safe_mapping_twist(0.2, 0.0, 0.0, clear)
        self.assertAlmostEqual(0.2, vx)
        self.assertEqual("clear", reason)

        blocked = dict(clear, rear=0.1)
        vx, vy, wz, reason = safe_mapping_twist(-0.1, 0.0, 0.0, blocked)
        self.assertEqual((0.0, 0.0, 0.0), (vx, vy, wz))
        self.assertEqual("rear_blocked", reason)

        blocked = dict(clear, left=0.1)
        vx, vy, wz, reason = safe_mapping_twist(0.0, 0.1, 0.0, blocked)
        self.assertEqual((0.0, 0.0, 0.0), (vx, vy, wz))
        self.assertEqual("left_blocked", reason)

        blocked = dict(clear, rotation=0.1)
        vx, vy, wz, reason = safe_mapping_twist(0.0, 0.0, 0.3, blocked)
        self.assertEqual((0.0, 0.0, 0.0), (vx, vy, wz))
        self.assertEqual("rotation_blocked", reason)

    def test_completion_requires_repeated_no_reachable_frontier(self):
        self.assertEqual("", completion_decision(
            elapsed_sec=120, known_cells=2000, no_frontier_cycles=2,
            map_stable_sec=40, min_elapsed_sec=60, min_known_cells=800,
            required_no_frontier_cycles=3, required_stable_sec=30,
        ))
        self.assertEqual("map_complete", completion_decision(
            elapsed_sec=120, known_cells=2000, no_frontier_cycles=3,
            map_stable_sec=40, min_elapsed_sec=60, min_known_cells=800,
            required_no_frontier_cycles=3, required_stable_sec=30,
        ))

    def test_mapping_request_contract(self):
        start = validate_mapping_request({
            "schema_version": 1,
            "request_id": "abc",
            "command": "start",
            "options": {"max_duration_sec": 600, "return_home": True},
        })
        self.assertEqual("start", start["command"])
        self.assertEqual(600.0, start["options"]["max_duration_sec"])
        with self.assertRaises(ValueError):
            validate_mapping_request({"schema_version": 2, "command": "start"})
        with self.assertRaises(ValueError):
            validate_mapping_request({"schema_version": 1, "command": "unknown"})
        with self.assertRaises(ValueError):
            validate_mapping_request({
                "schema_version": 1, "command": "start",
                "options": {"return_home": "false"},
            })
        with self.assertRaises(ValueError):
            validate_mapping_request({
                "schema_version": 1, "command": "start",
                "options": {"max_linear_speed": float("nan")},
            })


if __name__ == "__main__":
    unittest.main()
