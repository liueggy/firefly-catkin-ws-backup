import os
import sys
import unittest

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPTS)

from profile_contract import build_profile_contract


class ProfileContractTest(unittest.TestCase):
    def test_profile_is_explicit_not_inferred_from_nodes(self):
        status = build_profile_contract(
            "navigation", {"amcl": False, "gmapping": True, "move_base": True},
            map_available=True)
        self.assertEqual("navigation", status["profile"])
        self.assertEqual("static_nav", status["mode"])

    def test_capabilities_are_profile_authoritative(self):
        nav = build_profile_contract("navigation", {}, map_available=True)
        inspection = build_profile_contract("inspection", {}, map_available=True)
        self.assertFalse(nav["capabilities"]["inspection"])
        self.assertTrue(nav["capabilities"]["camera"])
        self.assertTrue(inspection["capabilities"]["inspection"])
        self.assertTrue(inspection["capabilities"]["meter_detection"])

    def test_missing_required_runtime_degrades_but_does_not_change_profile(self):
        status = build_profile_contract("inspection", {"amcl": True}, map_available=True)
        self.assertEqual("inspection", status["profile"])
        self.assertEqual("degraded", status["state"])

    def test_inspection_requires_both_kimi_service_and_bridge(self):
        observed = {
            "amcl": True,
            "move_base": True,
            "mission_runner": True,
            "camera": True,
            "meter_detection": True,
            "kimi_server": False,
            "kimi_bridge": True,
        }
        self.assertEqual(
            "degraded",
            build_profile_contract("inspection", observed, map_available=True)["state"],
        )
        observed["kimi_server"] = True
        self.assertEqual(
            "ready",
            build_profile_contract("inspection", observed, map_available=True)["state"],
        )


if __name__ == "__main__":
    unittest.main()
