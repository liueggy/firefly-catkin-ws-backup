#!/usr/bin/env python3

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from eggy_bringup.gps_nmea import enu_velocity, parse_sentence, valid_checksum


class GpsNmeaTest(unittest.TestCase):
    def test_no_fix_gga_from_real_module(self):
        parsed = parse_sentence("$GNGGA,,,,,,0,00,25.5,,,,,,*64")
        self.assertFalse(parsed["fix"])
        self.assertEqual(parsed["satellites"], 0)
        self.assertIsNone(parsed["latitude"])

    def test_valid_fix_and_coordinates(self):
        parsed = parse_sentence(
            "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
        self.assertTrue(parsed["fix"])
        self.assertAlmostEqual(parsed["latitude"], 48.1173, places=5)
        self.assertAlmostEqual(parsed["longitude"], 11.5166667, places=5)
        self.assertEqual(parsed["satellites"], 8)

    def test_checksum_rejects_corruption(self):
        self.assertFalse(valid_checksum("$GNGGA,,,,,,0,00,25.5,,,,,,*00"))
        self.assertIsNone(parse_sentence("not-nmea"))

    def test_nmea_course_to_enu(self):
        east, north = enu_velocity(2.0, 90.0)
        self.assertAlmostEqual(east, 2.0, places=6)
        self.assertAlmostEqual(north, 0.0, places=6)
        east, north = enu_velocity(2.0, 0.0)
        self.assertAlmostEqual(east, 0.0, places=6)
        self.assertAlmostEqual(north, 2.0, places=6)


if __name__ == "__main__":
    unittest.main()
