#!/usr/bin/env python3

import math
import os
import sys
import unittest

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "src"))

from eggy_bringup.voice_motion import (  # noqa: E402
    bounded_speed,
    normalize_angle,
    projected_progress,
    target_reached,
)


class VoiceMotionTest(unittest.TestCase):
    def test_forward_progress_uses_starting_heading(self):
        start = (1.0, 2.0, math.pi / 2.0)
        self.assertAlmostEqual(
            projected_progress("x", start, (1.0, 2.3, math.pi / 2.0)),
            0.3)

    def test_lateral_progress_uses_starting_body_axis(self):
        start = (0.0, 0.0, math.pi / 2.0)
        self.assertAlmostEqual(
            projected_progress("y", start, (-0.25, 0.0, math.pi / 2.0)),
            0.25)

    def test_rotation_progress_crosses_pi_boundary(self):
        start = (0.0, 0.0, math.radians(170.0))
        current = (0.0, 0.0, math.radians(-160.0))
        self.assertAlmostEqual(
            projected_progress("yaw", start, current),
            math.radians(30.0))

    def test_target_reached_accepts_tolerance_and_overshoot(self):
        self.assertTrue(target_reached(0.30, 0.28, 0.025))
        self.assertTrue(target_reached(-0.25, -0.27, 0.01))
        self.assertFalse(target_reached(0.30, 0.20, 0.025))

    def test_bounded_speed_decelerates_and_preserves_direction(self):
        self.assertAlmostEqual(bounded_speed(1.0, 0.22, 0.07, 1.2), 0.22)
        self.assertAlmostEqual(bounded_speed(0.02, 0.22, 0.07, 1.2), 0.07)
        self.assertAlmostEqual(bounded_speed(-0.10, 0.22, 0.07, 1.2), -0.12)

    def test_normalize_angle(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)


if __name__ == "__main__":
    unittest.main()
