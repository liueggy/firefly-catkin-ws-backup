#!/usr/bin/env python3
import importlib.util
import os
import sys
import threading
import types
import unittest


class _Message:
    def __init__(self, data=None, *args):
        self.data = data


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Client:
    def __init__(self):
        self.cancel_count = 0

    def cancel_goal(self):
        self.cancel_count += 1


def _install_ros_stubs():
    rospy = types.ModuleType("rospy")
    rospy.is_shutdown = lambda: False
    rospy.sleep = lambda _duration: None
    rospy.Duration = lambda value: value
    rospy.Time = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 1.0))
    sys.modules["rospy"] = rospy
    actionlib = types.ModuleType("actionlib")
    actionlib.SimpleActionClient = object
    sys.modules["actionlib"] = actionlib
    goal_status = type("GoalStatus", (), dict(
        PENDING=0, ACTIVE=1, PREEMPTED=2, SUCCEEDED=3, ABORTED=4,
        REJECTED=5, PREEMPTING=6, RECALLING=7, RECALLED=8, LOST=9))
    actionlib_msgs = types.ModuleType("actionlib_msgs")
    actionlib_msgs_msg = types.ModuleType("actionlib_msgs.msg")
    actionlib_msgs_msg.GoalStatus = goal_status
    sys.modules["actionlib_msgs"] = actionlib_msgs
    sys.modules["actionlib_msgs.msg"] = actionlib_msgs_msg
    geometry = types.ModuleType("geometry_msgs")
    geometry_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msg.PoseStamped = _Message
    geometry_msg.Quaternion = _Message
    geometry_msg.Twist = _Message
    sys.modules["geometry_msgs"] = geometry
    sys.modules["geometry_msgs.msg"] = geometry_msg
    move_base = types.ModuleType("move_base_msgs")
    move_base_msg = types.ModuleType("move_base_msgs.msg")
    move_base_msg.MoveBaseAction = object
    move_base_msg.MoveBaseGoal = _Message
    sys.modules["move_base_msgs"] = move_base
    sys.modules["move_base_msgs.msg"] = move_base_msg
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = _Message
    sys.modules["std_msgs"] = std
    sys.modules["std_msgs.msg"] = std_msg
    tf = types.ModuleType("tf")
    tf.TransformListener = object
    tf.transformations = types.SimpleNamespace(euler_from_quaternion=lambda _q: (0, 0, 0))
    sys.modules["tf"] = tf


SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPTS)
_install_ros_stubs()
spec = importlib.util.spec_from_file_location(
    "mission_runner", os.path.join(SCRIPTS, "inspection_servo_route_runner.py"))
runner_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner_module)


class MissionRunnerStateTest(unittest.TestCase):
    def make_runner(self):
        runner = runner_module.InspectionServoRouteRunner.__new__(
            runner_module.InspectionServoRouteRunner)
        runner.default_frame = "map"
        runner.dry_run = True
        runner.busy = True
        runner.cancel_requested = False
        runner.lock = threading.Lock()
        runner.current_mission = {}
        runner.current_point_index = None
        runner.current_point_id = None
        runner.enable_kimi_after_search = True
        runner.stop_on_kimi_fail = False
        runner.inspection_capable = True
        runner.seen_request_ids = set()
        runner.search_settle_sec = 0
        runner.status_events = []
        runner.publish_status = lambda state, message, extra, mission=None: runner.status_events.append(state)
        runner.stop_robot = lambda: None
        runner.result_pub = _Publisher()
        runner.legacy_result_pub = _Publisher()
        runner.client = _Client()
        runner.calls = {"home": 0, "nav": 0, "search": 0, "kimi": 0}

        def home():
            runner.calls["home"] += 1
            return {"id": "home", "frame_id": "map", "x": 0, "y": 0, "yaw": 0}

        runner.current_pose_waypoint = home
        runner.navigate_to = lambda _wp: runner.calls.__setitem__("nav", runner.calls["nav"] + 1) or {"ok": True, "state_text": "OK"}
        runner.search_target_at_waypoint = lambda _wp: runner.calls.__setitem__("search", runner.calls["search"] + 1) or {"ok": True, "target": {"class_name": "any"}}
        runner.wait_for_detection_window = lambda _seconds: {"class_name": "passive"}
        runner.run_kimi_inspection = lambda _wp: runner.calls.__setitem__("kimi", runner.calls["kimi"] + 1) or {"ok": True}
        return runner

    def mission(self, return_home=False, inspection_enabled=False):
        mission = {
            "request_id": "mission-1", "mission_type": "navigation",
            "return_home": return_home, "loop": False,
            "on_nav_failure": "stop",
            "inspection": {
                "enabled": inspection_enabled,
                "vision_search": inspection_enabled,
                "ai_analysis": inspection_enabled,
            },
            "route": [{"id": "a", "frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.0},
                      {"id": "b", "frame_id": "map", "x": 2.0, "y": 3.0, "yaw": 0.0}],
        }
        return mission

    def test_inspection_disabled_calls_no_search_or_ai(self):
        runner = self.make_runner()
        runner.run_route(self.mission(False))
        self.assertEqual(2, runner.calls["nav"])
        self.assertEqual(0, runner.calls["search"])
        self.assertEqual(0, runner.calls["kimi"])
        self.assertEqual(0, runner.calls["home"])

    def test_return_home_true_records_and_navigates_home_once(self):
        runner = self.make_runner()
        runner.run_route(self.mission(True))
        self.assertEqual(1, runner.calls["home"])
        self.assertEqual(3, runner.calls["nav"])

    def test_inspection_enabled_runs_search_and_ai_per_point(self):
        runner = self.make_runner()
        runner.run_route(self.mission(False, True))
        self.assertEqual(2, runner.calls["nav"])
        self.assertEqual(2, runner.calls["search"])
        self.assertEqual(2, runner.calls["kimi"])

    def test_visual_search_can_run_without_ai_analysis(self):
        runner = self.make_runner()
        mission = self.mission(False, True)
        mission["inspection"]["ai_analysis"] = False
        runner.run_route(mission)
        self.assertEqual(2, runner.calls["search"])
        self.assertEqual(0, runner.calls["kimi"])

    def test_passive_detection_does_not_call_motion_search(self):
        runner = self.make_runner()
        mission = self.mission(False, True)
        mission["inspection"]["vision_search"] = False
        runner.search_active = False
        runner.detection_candidate = None
        runner.detection_stable_count = 0
        runner.last_detection_class = ""
        runner.run_route(mission)
        self.assertEqual(0, runner.calls["search"])
        self.assertEqual(2, runner.calls["kimi"])
        self.assertFalse(runner.search_active)

    def test_loop_repeats_route_until_cancelled(self):
        runner = self.make_runner()
        mission = self.mission(False)
        mission["loop"] = True

        def navigate(_wp):
            runner.calls["nav"] += 1
            if runner.calls["nav"] == 3:
                runner.cancel_requested = True
            return {"ok": True, "state_text": "OK"}

        runner.navigate_to = navigate
        runner.run_route(mission)
        self.assertEqual(3, runner.calls["nav"])
        self.assertIn("cancelled", runner.status_events)

    def test_cancel_only_affects_matching_active_request(self):
        runner = self.make_runner()
        runner.current_mission = self.mission(False)
        runner.handle_request(_Message('{"command":"cancel","request_id":"old"}'))
        self.assertEqual(0, runner.client.cancel_count)
        self.assertFalse(runner.cancel_requested)
        runner.handle_request(_Message('{"command":"cancel","request_id":"mission-1"}'))
        self.assertEqual(1, runner.client.cancel_count)
        self.assertTrue(runner.cancel_requested)

    def test_duplicate_request_id_is_rejected(self):
        runner = self.make_runner()
        runner.busy = False
        runner.seen_request_ids.add("mission-1")
        runner.handle_request(_Message(__import__("json").dumps(self.mission(False))))
        self.assertIn("rejected", runner.status_events)
        self.assertFalse(runner.busy)


if __name__ == "__main__":
    unittest.main()
