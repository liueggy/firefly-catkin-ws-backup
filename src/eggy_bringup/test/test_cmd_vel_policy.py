import os
import sys
import unittest

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPTS)

from cmd_vel_policy import select_source


class CmdVelPolicyTest(unittest.TestCase):
    def test_highest_live_priority_wins(self):
        sources = {
            "navigation": {"stamp": 9.9, "timeout": 0.5, "priority": 40},
            "mission": {"stamp": 9.8, "timeout": 0.5, "priority": 60},
            "manual": {"stamp": 9.7, "timeout": 0.5, "priority": 80},
        }
        self.assertEqual("manual", select_source(sources, 10.0))

    def test_expired_source_releases_control(self):
        sources = {
            "manual": {"stamp": 8.0, "timeout": 0.5, "priority": 80},
            "navigation": {"stamp": 9.9, "timeout": 0.5, "priority": 40},
        }
        self.assertEqual("navigation", select_source(sources, 10.0))

    def test_no_live_source_means_stop(self):
        self.assertIsNone(select_source({}, 10.0))


if __name__ == "__main__":
    unittest.main()
