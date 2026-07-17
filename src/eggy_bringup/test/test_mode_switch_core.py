import os
import sys
import tempfile
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, PACKAGE_SRC)

from eggy_bringup.mode_switch_core import (
    SwitchGate,
    activate_map_directory,
    restore_map_directory,
)


def make_map(directory, name):
    target = os.path.join(directory, name)
    os.makedirs(target)
    for filename in ("map.yaml", "map.pgm", "metadata.json"):
        with open(os.path.join(target, filename), "wb") as stream:
            stream.write(b"test")
    return target


class ModeSwitchCoreTest(unittest.TestCase):
    def test_switch_gate_rejects_overlapping_requests(self):
        gate = SwitchGate()
        self.assertTrue(gate.try_begin("first"))
        self.assertEqual("first", gate.owner)
        self.assertFalse(gate.try_begin("second"))
        gate.finish()
        self.assertTrue(gate.try_begin("second"))
        gate.finish()

    def test_active_map_switch_is_atomic_and_can_roll_back(self):
        with tempfile.TemporaryDirectory() as root:
            first = make_map(root, "first")
            second = make_map(root, "second")
            active = os.path.join(root, "active")
            os.symlink(first, active)

            previous = activate_map_directory(active, second)
            self.assertEqual(os.path.realpath(second), os.path.realpath(active))
            self.assertEqual(os.path.realpath(first), previous)

            restore_map_directory(active, previous)
            self.assertEqual(os.path.realpath(first), os.path.realpath(active))

    def test_incomplete_map_cannot_replace_active_map(self):
        with tempfile.TemporaryDirectory() as root:
            first = make_map(root, "first")
            incomplete = os.path.join(root, "incomplete")
            os.makedirs(incomplete)
            active = os.path.join(root, "active")
            os.symlink(first, active)

            with self.assertRaises(ValueError):
                activate_map_directory(active, incomplete)
            self.assertEqual(os.path.realpath(first), os.path.realpath(active))


if __name__ == "__main__":
    unittest.main()
