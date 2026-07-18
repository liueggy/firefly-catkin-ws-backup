import os
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read(relative):
    with open(os.path.join(ROOT, relative), "r", encoding="utf-8") as stream:
        return stream.read()


class LaunchContractTest(unittest.TestCase):
    def test_teb_prefers_smooth_forward_motion_with_bounded_lateral_trim(self):
        config = yaml.safe_load(read("config/nav/teb_local_planner_params.yaml"))[
            "TebLocalPlannerROS"
        ]
        self.assertLessEqual(config["max_vel_y"], 0.05)
        self.assertLessEqual(config["max_vel_theta"], 0.65)
        self.assertLessEqual(config["acc_lim_x"], 0.40)
        self.assertLessEqual(config["acc_lim_y"], 0.20)
        self.assertLessEqual(config["acc_lim_theta"], 0.90)
        self.assertGreaterEqual(config["weight_kinematics_forward_drive"], 10.0)
        self.assertGreaterEqual(config["global_plan_viapoint_sep"], 0.30)
        self.assertLessEqual(config["weight_viapoint"], 15.0)

    def test_non_mapping_profiles_hard_disable_mapping_runtime(self):
        manager = read("scripts/auto_mapping_manager.py")
        safety = read("scripts/auto_mapping_safety.py")
        self.assertIn("profile_allows_mapping", manager)
        self.assertIn("set_sensor_subscriptions", manager)
        self.assertIn('"profile_rejected"', manager)
        self.assertIn("profile_allows_mapping", safety)
        self.assertIn("self.mapping_profile_active", safety)

    def test_mission_navigation_is_gated_by_fresh_scan_and_tf(self):
        runner = read("scripts/inspection_servo_route_runner.py")
        self.assertIn("navigation_sensor_health", runner)
        self.assertIn('"navigation_sensor_stale"', runner)
        self.assertIn("navigation_scan_timeout", runner)
        self.assertIn("navigation_tf_timeout", runner)
        self.assertIn("navigation_stale_grace", runner)

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

    def test_static_navigation_uses_saved_map_costmap_and_goal_tolerance(self):
        navigation = read("launch/move_base_nav.launch")
        self.assertIn("global_costmap_params.yaml", navigation)
        self.assertNotIn("global_costmap_explore_params.yaml", navigation)
        self.assertIn('name="NavfnROS/default_tolerance" value="0.30"',
                      navigation)
        mapping = read("launch/move_base_only.launch")
        self.assertIn('name="NavfnROS/default_tolerance" value="0.30"',
                      mapping)
        costmap = read("config/nav/global_costmap_params.yaml")
        self.assertIn('type: "costmap_2d::StaticLayer"', costmap)
        self.assertIn("rolling_window: false", costmap)
        command_center = read("scripts/eggy_command_center.py")
        self.assertIn("global_costmap_params.yaml' if navigation", command_center)
        self.assertIn("rospy.set_param('/move_base', move_base_params)",
                      command_center)
        self.assertIn("name.startswith('/map_server_')", command_center)

    def test_system_launches_authoritative_cmd_vel_arbiter(self):
        system = read("launch/eggy_system.launch")
        self.assertIn('type="cmd_vel_arbiter.py"', system)
        self.assertIn('name="eggy_cmd_vel_arbiter"', system)
        self.assertIn('<param name="manual_timeout" value="0.75"', system)
        self.assertIn('rospy.get_param("~manual_timeout", 0.75)',
                      read("scripts/cmd_vel_arbiter.py"))

    def test_cpp_odom_fuser_has_bounded_online_drift_correction(self):
        system = read("launch/eggy_system.launch")
        self.assertIn('<param name="rate" value="30.0"', system)
        for name, value in (
                ("wheel_yaw_correction_rate", "0.35"),
                ("stationary_linear_threshold", "0.015"),
                ("stationary_angular_threshold", "0.025"),
                ("bias_learning_rate", "0.002")):
            self.assertIn('<param name="%s" value="%s"' % (name, value),
                          system)

    def test_rplidar_recovers_after_serial_reenumeration(self):
        root = ET.fromstring(read("../rplidar_ros/launch/rplidar_a1.launch"))
        node = root.find(".//node[@name='rplidarNode']")
        self.assertIsNotNone(node)
        self.assertEqual("true", node.attrib.get("respawn"))
        self.assertEqual("2.0", node.attrib.get("respawn_delay"))
        params = {
            item.attrib["name"]: item.attrib["value"]
            for item in node.findall("param")
        }
        self.assertEqual("/dev/rplidar", params["serial_port"])
        self.assertEqual("3.0", params["scan_failure_exit_timeout"])
        driver = read("../rplidar_ros/src/node.cpp")
        self.assertIn("failure_age >= scan_failure_exit_timeout", driver)
        self.assertIn("RPLIDAR scan stalled for %.1fs", driver)

    def test_rplidar_omits_unused_intensity_samples_on_the_robot(self):
        root = ET.fromstring(read("../rplidar_ros/launch/rplidar_a1.launch"))
        node = root.find(".//node[@name='rplidarNode']")
        self.assertIsNotNone(node)
        params = {
            item.attrib["name"]: item.attrib["value"]
            for item in node.findall("param")
        }
        self.assertEqual("false", params["publish_intensity"])

        driver = read("../rplidar_ros/src/node.cpp")
        self.assertIn('param<bool>("publish_intensity"', driver)
        self.assertIn("if (publish_intensity)", driver)

    def test_external_lidar_watchdog_covers_blocked_driver_reads(self):
        root = ET.fromstring(read("launch/eggy_system.launch"))
        lidar_group = root.find(".//group[@if='$(arg use_lidar)']")
        self.assertIsNotNone(lidar_group)
        watchdog = lidar_group.find(".//node[@name='eggy_lidar_watchdog']")
        self.assertIsNotNone(watchdog)
        self.assertEqual("lidar_watchdog.py", watchdog.attrib.get("type"))
        self.assertEqual("true", watchdog.attrib.get("respawn"))
        params = {
            item.attrib["name"]: item.attrib["value"]
            for item in watchdog.findall("param")
        }
        self.assertEqual("/scan", params["scan_topic"])
        self.assertEqual("8.0", params["startup_grace"])
        self.assertEqual("2.0", params["scan_timeout"])
        self.assertEqual("10.0", params["recovery_cooldown"])

        cmake = read("CMakeLists.txt")
        self.assertIn("scripts/lidar_watchdog.py", cmake)

    def test_idle_services_do_not_deserialize_or_process_full_sensor_frames(self):
        manager = read("scripts/auto_mapping_manager.py")
        safety = read("scripts/auto_mapping_safety.py")
        voice = read("scripts/eggy_voice_controller.py")
        health = read("scripts/eggy_health_aggregator.py")
        self.assertIn('Subscriber("/scan", rospy.AnyMsg', manager)
        self.assertIn("if not active:", manager)
        self.assertIn("self.scan_subscriber = None", safety)
        self.assertIn("self.scan_subscriber.unregister()", safety)
        self.assertIn('Subscriber("/scan", rospy.AnyMsg', voice)
        self.assertIn("node_refresh_period", health)
        self.assertIn("self.alive_nodes_cache", health)
        self.assertIn("'/eggy/nav_mode/status'", health)
        self.assertIn("self.active_profile", health)
        self.assertIn("active_profile == 'inspection'", health)

        command_center = read("scripts/eggy_command_center.py")
        self.assertIn("free_thresh < occupied_thresh", command_center)
        self.assertIn("parsed['occupied_thresh'] = 0.65", command_center)
        self.assertIn("parsed['free_thresh'] = 0.196", command_center)

    def test_voice_controller_is_enabled_in_every_runtime_profile(self):
        for name in ("mapping", "navigation", "inspection"):
            profile = read("launch/%s_profile.launch" % name)
            self.assertIn('use_voice_controller" value="true"', profile)
            self.assertNotIn('use_voice_controller" value="false"', profile)

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
        self.assertIn('<param name="manual_timeout" value="0.75"', system)

    def test_auto_mapping_uses_deployed_ros_message_types(self):
        manager = read("scripts/auto_mapping_manager.py")
        safety = read("scripts/auto_mapping_safety.py")
        self.assertIn("from nav_msgs.srv import GetPlan", manager)
        self.assertIn('Subscriber("/base/flag_stop", UInt8', manager)
        self.assertIn('Subscriber("/base/flag_stop", UInt8', safety)

    def test_mapping_profiles_publish_fresh_maps_without_excessive_scan_work(self):
        expected = {
            "map_update_interval": "1.0",
            "linear_update": "0.15",
            "angular_update": "0.15",
            "temporal_update": "1.0",
            "delta": "0.05",
        }
        for filename, prefix in (
                ("mapping_light.launch", ""),
                ("auto_mapping_light.launch", ""),
                ("auto_explore_mapping.launch", "gmapping_")):
            root = ET.fromstring(read("launch/%s" % filename))
            defaults = {
                item.attrib["name"]: item.attrib.get("default")
                for item in root.findall("arg")
            }
            for name, value in expected.items():
                self.assertEqual(value, defaults[prefix + name],
                                 "%s:%s" % (filename, prefix + name))

    def test_fast_mapping_switch_uses_the_tuned_grid_resolution(self):
        command_center = read("scripts/eggy_command_center.py")
        self.assertIn("_delta:=0.05", command_center)

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
        self.assertIn("self.camera_output_topic, CompressedImage", command_center)
        self.assertIn("output_frame_ok", command_center)

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

    def test_runtime_profile_support_matches_navigation_and_inspection_contracts(self):
        root = ET.fromstring(read("launch/runtime_profile_support.launch"))
        camera = root.find(".//node[@name='eggy_camera']")
        runner = root.find(".//node[@name='inspection_servo_route_runner']")
        inspection_group = root.find(".//group[@if='$(arg inspection)']")
        navigation_group = root.find(".//group[@unless='$(arg inspection)']")
        self.assertIsNotNone(camera)
        self.assertIsNotNone(runner)
        self.assertIsNotNone(inspection_group)
        self.assertIsNotNone(navigation_group)
        camera_params = {
            item.attrib["name"]: item.attrib["value"]
            for item in camera.findall("param")
        }
        self.assertEqual(
            "/camera/front/image_source/compressed",
            camera_params["compressed_topic"],
        )
        inspection_nodes = {
            item.attrib["name"] for item in inspection_group.findall("node")
        }
        self.assertEqual(
            {
                "meter_rknn_detect_cpp",
                "kimi_inspection_server",
                "kimi_inspection_bridge",
            },
            inspection_nodes,
        )
        relay = navigation_group.find("node[@name='eggy_camera_raw_to_qt']")
        self.assertIsNotNone(relay)
        self.assertEqual("topic_tools", relay.attrib["pkg"])

    def test_fast_navigation_profile_starts_and_stops_camera_relay(self):
        source = read("scripts/eggy_command_center.py")
        self.assertIn("rosrun topic_tools relay", source)
        self.assertIn("'/eggy_camera_raw_to_qt'", source)

    def test_fast_profile_switch_preserves_the_persistent_base_stack(self):
        source = read("scripts/eggy_command_center.py")
        self.assertIn("'roslaunch --skip-log-check '", source)
        fast_switch = source.split("    def switch_mode_fast(", 1)[1].split(
            "    def handle_switch_nav_mode(", 1)[0]
        self.assertNotIn("eggy-stack-start", fast_switch)
        self.assertNotIn("_cleanup_ros_master()", fast_switch)
        self.assertIn("_stop_runtime_mode()", fast_switch)
        self.assertIn("_wait_map_matches", fast_switch)
        self.assertIn("_wait_mode_ready", fast_switch)

    def test_periodic_command_status_uses_compact_payload(self):
        source = read("scripts/eggy_command_center.py")
        self.assertIn("def build_status(self, detailed=True):", source)
        self.assertIn("status = self.build_status(detailed=False)", source)
        self.assertIn("if detailed:", source)


if __name__ == "__main__":
    unittest.main()
