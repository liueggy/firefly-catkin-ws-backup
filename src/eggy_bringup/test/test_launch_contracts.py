import os
import unittest
import xml.etree.ElementTree as ET


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read(relative):
    with open(os.path.join(ROOT, relative), "r", encoding="utf-8") as stream:
        return stream.read()


class LaunchContractTest(unittest.TestCase):
    def test_profiles_declare_authoritative_identity(self):
        for name in ("mapping", "navigation", "inspection"):
            text = read("launch/%s_profile.launch" % name)
            self.assertIn('<arg name="profile" value="%s"' % name, text)

    def test_move_base_uses_arbiter_navigation_input(self):
        for name in ("move_base_nav.launch", "move_base_only.launch"):
            text = read("launch/%s" % name)
            self.assertIn('name="cmd_vel_topic" default="/cmd_vel/navigation"', text)
            self.assertIn('from="cmd_vel" to="$(arg cmd_vel_topic)"', text)
            self.assertNotIn('to="/cmd_vel"', text)

    def test_system_launches_authoritative_cmd_vel_arbiter(self):
        system = read("launch/eggy_system.launch")
        self.assertIn('type="cmd_vel_arbiter.py"', system)
        self.assertIn('name="eggy_cmd_vel_arbiter"', system)

    def test_mapping_profile_routes_move_base_through_fail_safe_guard(self):
        profile = read("launch/mapping_profile.launch")
        system = read("launch/eggy_system.launch")
        move_base = read("launch/move_base_only.launch")
        self.assertIn('use_auto_mapping" value="true"', profile)
        self.assertIn('navigation_cmd_vel_topic" value="/cmd_vel/mapping_raw"', profile)
        self.assertIn('type="auto_mapping_safety.py"', system)
        self.assertIn('type="auto_mapping_manager.py"', system)
        self.assertIn('to="$(arg cmd_vel_topic)"', move_base)
        self.assertIn('"mapping": ("/cmd_vel/mapping", 70, 0.4)',
                      read("scripts/cmd_vel_arbiter.py"))

    def test_auto_mapping_uses_deployed_ros_message_types(self):
        manager = read("scripts/auto_mapping_manager.py")
        safety = read("scripts/auto_mapping_safety.py")
        self.assertIn("from nav_msgs.srv import GetPlan", manager)
        self.assertIn('Subscriber("/base/flag_stop", UInt8', manager)
        self.assertIn('Subscriber("/base/flag_stop", UInt8', safety)

    def test_raw_camera_relay_and_legacy_adapter_switch_are_explicit(self):
        system = read("launch/eggy_system.launch")
        self.assertIn("eggy_camera_raw_to_qt", system)
        self.assertIn('legacy_qt_visual_relays" default="false"', system)
        self.assertIn('legacy_qt_camera_relay" default="false"', system)

    def test_camera_start_requires_a_real_frame_on_the_stable_raw_topic(self):
        command_center = read("scripts/eggy_command_center.py")
        self.assertIn(
            "'~camera_stream_topic', '/camera/front/image_source/compressed'",
            command_center,
        )
        self.assertIn("rospy.wait_for_message(", command_center)
        self.assertIn("self.camera_stream_topic, CompressedImage", command_center)
        self.assertIn("ok = node_ok and frame_ok", command_center)

    def test_meter_input_and_overlay_topics_cannot_form_default_feedback_loop(self):
        root = ET.fromstring(read("launch/eggy_system.launch"))
        args = {item.attrib["name"]: item.attrib.get("default") for item in root.findall("arg")}
        self.assertEqual("/camera/front/image_source/compressed", args["meter_image_topic"])
        self.assertEqual("/camera/front/image/compressed", args["meter_overlay_topic"])
        self.assertNotEqual(args["meter_image_topic"], args["meter_overlay_topic"])

        meter = root.find(".//node[@name='meter_rknn_detect_cpp']")
        self.assertIsNotNone(meter)
        params = {item.attrib["name"]: item.attrib["value"] for item in meter.findall("param")}
        self.assertEqual("$(arg meter_image_topic)", params["image_topic"])
        self.assertEqual("$(arg meter_overlay_topic)", params["overlay_comp_topic"])


if __name__ == "__main__":
    unittest.main()
