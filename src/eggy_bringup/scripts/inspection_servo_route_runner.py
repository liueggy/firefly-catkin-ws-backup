#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unified mission runner for plain navigation and opt-in inspection routes.

The runner owns one mission at a time, reports identity-rich feedback, and
keeps camera search and Kimi analysis dormant unless inspection is explicitly
enabled by the request and supported by the active launch profile.
"""

import json
import itertools
import math
import threading
import time
import uuid

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
import tf

from mission_protocol import normalize_mission_request


GOAL_STATUS_TEXT = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


def quat_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def now_sec():
    return rospy.Time.now().to_sec() if not rospy.is_shutdown() else time.time()


class InspectionServoRouteRunner:
    def __init__(self):
        self.default_frame = rospy.get_param("~default_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.route_file = rospy.get_param("~route_file", "")
        self.move_base_timeout = float(rospy.get_param("~move_base_timeout", 120.0))
        self.current_pose_timeout = float(rospy.get_param("~current_pose_timeout", 3.0))
        self.enable_kimi_after_search = bool(rospy.get_param("~enable_kimi_after_search", True))
        self.kimi_timeout = float(rospy.get_param("~kimi_timeout", 90.0))
        self.kimi_task = rospy.get_param("~kimi_task", "meter")
        self.stop_on_nav_fail = bool(rospy.get_param("~stop_on_nav_fail", True))
        self.stop_on_kimi_fail = bool(rospy.get_param("~stop_on_kimi_fail", False))
        self.search_min_score = float(rospy.get_param("~search_min_score", 0.65))
        self.search_stable_frames = int(rospy.get_param("~search_stable_frames", 2))
        self.search_max_age = float(rospy.get_param("~search_max_age", 0.8))
        self.search_timeout = float(rospy.get_param("~search_timeout", 30.0))
        self.search_angular_speed_deg = float(rospy.get_param("~search_angular_speed_deg", 18.0))
        self.search_max_rotation_deg = float(rospy.get_param("~search_max_rotation_deg", 360.0))
        self.search_step_deg = float(rospy.get_param("~search_step_deg", 90.0))
        self.search_step_pause_sec = float(rospy.get_param("~search_step_pause_sec", 1.0))
        self.search_settle_sec = float(rospy.get_param("~search_settle_sec", 2.5))
        self.align_timeout = float(rospy.get_param("~align_timeout", 8.0))
        self.align_center_deadband = float(rospy.get_param("~align_center_deadband", 0.12))
        self.align_hold_sec = float(rospy.get_param("~align_hold_sec", 0.45))
        self.align_kp = float(rospy.get_param("~align_kp", 1.8))
        self.align_max_wz = float(rospy.get_param("~align_max_wz", 0.42))
        self.align_min_wz = float(rospy.get_param("~align_min_wz", 0.08))
        self.angular_accel = float(rospy.get_param("~angular_accel", 0.9))
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.6))
        self.rotation_clearance = float(rospy.get_param("~rotation_clearance", 0.32))
        self.control_rate_hz = max(5.0, float(rospy.get_param("~control_rate_hz", 12.0)))
        self.roi_padding_ratio = float(rospy.get_param("~roi_padding_ratio", 0.18))
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.inspection_capable = bool(rospy.get_param("~inspection_capable", True))

        self.lock = threading.Lock()
        self.busy = False
        self.cancel_requested = False
        self.kimi_results = {}
        self.navigation_active = False
        self.search_active = False
        self.expected_class = "any"
        self.detection_candidate = None
        self.detection_stable_count = 0
        self.last_detection_class = ""
        self.last_cmd_wz = 0.0
        self.last_cmd_vx = 0.0
        self.last_cmd_time = 0.0
        self.last_scan = None
        self.last_scan_time = 0.0
        self.current_mission = {}
        self.current_point_index = None
        self.current_point_id = None
        self.seen_request_ids = set()

        self.status_pub = rospy.Publisher("/eggy/mission/status", String, queue_size=10, latch=True)
        self.result_pub = rospy.Publisher("/eggy/mission/result", String, queue_size=10, latch=True)
        self.legacy_status_pub = rospy.Publisher("/inspection_servo_route/status", String, queue_size=10, latch=True)
        self.legacy_result_pub = rospy.Publisher("/inspection_servo_route/result", String, queue_size=10, latch=True)
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel/mission")
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.kimi_request_pub = rospy.Publisher("/kimi_inspection/request", String, queue_size=5)

        self.tf_listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        rospy.Subscriber("/eggy/mission/request", String, self.on_request, queue_size=5)
        rospy.Subscriber("/eggy/emergency_stop", Bool,
                         self.on_emergency_stop, queue_size=1)
        rospy.Subscriber("/inspection_servo_route/request", String, self.on_legacy_request, queue_size=5)
        rospy.Subscriber("/kimi_inspection/result", String, self.on_kimi_result, queue_size=10)
        rospy.Subscriber("/meter/detection", String, self.on_detection, queue_size=10)
        rospy.Subscriber(self.scan_topic, LaserScan, self.on_scan, queue_size=1)

        self.publish_status("ready", "inspection servo route runner ready", {})
        rospy.loginfo("inspection_servo_route_runner ready dry_run=%s", self.dry_run)

    def on_kimi_result(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        request_id = data.get("request_id")
        if request_id:
            with self.lock:
                self.kimi_results[str(request_id)] = data

    def on_detection(self, msg):
        try:
            payload = json.loads(msg.data)
            detections = payload.get("detections") or []
            image_width = float(payload.get("image_width") or 0.0)
            image_height = float(payload.get("image_height") or 0.0)
        except Exception:
            return

        with self.lock:
            if not (self.navigation_active or self.search_active):
                return
            if not self.current_mission.get("inspection", {}).get("enabled", False):
                return
            expected = self.expected_class

        valid = []
        for detection in detections:
            class_name = str(detection.get("class_name", ""))
            score = float(detection.get("score", 0.0))
            if class_name not in ("water_meter", "pressure_gauge"):
                continue
            if expected not in ("", "any", class_name):
                continue
            if score < self.search_min_score:
                continue
            valid.append(detection)

        best = max(valid, key=lambda item: float(item.get("score", 0.0))) if valid else None
        with self.lock:
            if best is None:
                self.detection_stable_count = 0
                self.last_detection_class = ""
                self.detection_candidate = None
                return
            class_name = str(best.get("class_name", ""))
            previous = self.detection_candidate
            same_track = (
                previous is not None
                and class_name == self.last_detection_class
                and self.box_iou(previous, best) >= 0.25
            )
            if same_track:
                self.detection_stable_count += 1
            else:
                self.last_detection_class = class_name
                self.detection_stable_count = 1
            self.detection_candidate = {
                "class_name": class_name,
                "score": float(best.get("score", 0.0)),
                "stable_frames": self.detection_stable_count,
                "stamp": time.time(),
                "image_width": image_width,
                "image_height": image_height,
                "x1": float(best.get("x1", 0.0)),
                "y1": float(best.get("y1", 0.0)),
                "x2": float(best.get("x2", 0.0)),
                "y2": float(best.get("y2", 0.0)),
                "box": {
                    key: best.get(key)
                    for key in ("x1", "y1", "x2", "y2")
                    if key in best
                },
            }

    @staticmethod
    def box_iou(left, right):
        try:
            ax1, ay1 = float(left.get("x1", 0.0)), float(left.get("y1", 0.0))
            ax2, ay2 = float(left.get("x2", 0.0)), float(left.get("y2", 0.0))
            bx1, by1 = float(right.get("x1", 0.0)), float(right.get("y1", 0.0))
            bx2, by2 = float(right.get("x2", 0.0)), float(right.get("y2", 0.0))
            ix1, iy1 = max(ax1, bx1), max(ay1, by1)
            ix2, iy2 = min(ax2, bx2), min(ay2, by2)
            intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
            union = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
            union += max(0.0, bx2 - bx1) * max(0.0, by2 - by1) - intersection
            return intersection / union if union > 1e-6 else 0.0
        except (TypeError, ValueError):
            return 0.0

    def on_scan(self, msg):
        self.last_scan = msg
        self.last_scan_time = time.time()

    def scan_is_safe_for_rotation(self):
        if self.last_scan is None or time.time() - self.last_scan_time > self.scan_timeout:
            return False, "scan_timeout"
        valid = [
            value for value in self.last_scan.ranges
            if math.isfinite(value) and self.last_scan.range_min <= value <= self.last_scan.range_max
        ]
        if not valid:
            return False, "scan_empty"
        clearance = min(valid)
        return clearance >= self.rotation_clearance, "obstacle_%.2f" % clearance

    def current_yaw(self):
        trans, rot = self.tf_listener.lookupTransform(
            self.default_frame, self.base_frame, rospy.Time(0))
        return tf.transformations.euler_from_quaternion(rot)[2]

    @staticmethod
    def angle_delta(current, initial):
        return math.atan2(math.sin(current - initial), math.cos(current - initial))

    def publish_smooth_command(self, linear_x=0.0, angular_z=0.0):
        now = time.time()
        dt = max(0.02, min(0.2, now - self.last_cmd_time)) if self.last_cmd_time else 0.08
        max_delta = max(0.05, self.angular_accel * dt)
        delta = max(-max_delta, min(max_delta, angular_z - self.last_cmd_wz))
        self.last_cmd_wz += delta
        self.last_cmd_vx = linear_x
        self.last_cmd_time = now
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = self.last_cmd_wz
        self.cmd_pub.publish(cmd)

    def align_target_at_waypoint(self, wp):
        deadline = time.time() + float(wp.get("align_timeout", self.align_timeout))
        hold_started = None
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "message": "cancelled"}
            safe, reason = self.scan_is_safe_for_rotation()
            if not safe:
                self.stop_robot()
                self.publish_status("rotation_sensor_stop", "rotation stopped by sensor safety check", {
                    "waypoint": wp, "reason": reason,
                })
                return {"ok": False, "state": "rotation_sensor_stop", "message": reason}
            candidate = self.current_detection_candidate()
            if not candidate:
                self.stop_robot()
                self.publish_status("target_lost", "target lost while aligning", {"waypoint": wp})
                return {"ok": False, "state": "target_lost", "message": "target lost while aligning"}
            width = float(candidate.get("image_width", 0.0))
            if width <= 0.0:
                self.stop_robot()
                self.publish_status("invalid_detection", "detection frame dimensions are invalid", {
                    "waypoint": wp,
                })
                return {"ok": False, "state": "invalid_detection", "message": "missing image dimensions"}
            center_x = (float(candidate.get("x1", 0.0)) + float(candidate.get("x2", 0.0))) * 0.5
            error = (center_x - width * 0.5) / max(width * 0.5, 1.0)
            stable = abs(error) <= self.align_center_deadband
            if stable:
                if hold_started is None:
                    hold_started = time.time()
                self.publish_smooth_command(0.0, 0.0)
                if time.time() - hold_started >= self.align_hold_sec:
                    self.stop_robot()
                    self.publish_status("aligned", "target aligned within camera center tolerance", {
                        "waypoint": wp,
                        "target": candidate,
                        "center_error": round(error, 4),
                    })
                    return {
                        "ok": True,
                        "state": "aligned",
                        "target": candidate,
                        "center_error": round(error, 4),
                    }
            else:
                hold_started = None
                target_wz = max(-self.align_max_wz,
                                min(self.align_max_wz, -self.align_kp * error))
                if abs(target_wz) < self.align_min_wz:
                    target_wz = self.align_min_wz if target_wz >= 0.0 else -self.align_min_wz
                self.publish_smooth_command(0.0, target_wz)
            rospy.sleep(1.0 / self.control_rate_hz)
        self.stop_robot()
        self.publish_status("align_timeout", "target alignment timed out", {"waypoint": wp})
        return {"ok": False, "state": "align_timeout", "message": "target alignment timed out"}

    def on_request(self, msg):
        self.handle_request(msg, legacy_inspection=False)

    def on_emergency_stop(self, msg):
        if not msg.data:
            return
        self.cancel_requested = True
        self.client.cancel_all_goals()
        self.stop_robot()
        if self.busy:
            self.publish_status(
                "emergency_stopped", "mission cancelled by software emergency stop", {})

    def on_legacy_request(self, msg):
        rospy.logwarn("/inspection_servo_route/request is deprecated; use /eggy/mission/request")
        self.handle_request(msg, legacy_inspection=True)

    def handle_request(self, msg, legacy_inspection=False):
        raw = {}
        try:
            raw = self.parse_payload(msg.data)
            payload = normalize_mission_request(
                raw, self.default_frame, legacy_inspection=legacy_inspection)
        except Exception as exc:
            self.publish_status("bad_request", str(exc), {}, mission=raw)
            return
        command = payload["command"]
        if command in ("stop", "cancel"):
            active_id = self.current_mission.get("request_id")
            if not self.busy or payload["request_id"] != active_id:
                self.publish_status(
                    "cancel_ignored", "cancel request does not match active mission",
                    {}, mission=payload)
                return
            self.cancel_requested = True
            self.client.cancel_goal()
            self.stop_robot()
            self.publish_status("cancelling", "mission cancel requested", {})
            return
        if payload["inspection"]["enabled"] and not self.inspection_capable:
            self.publish_status(
                "rejected", "inspection capability is disabled in this launch profile",
                {}, mission=payload)
            return
        with self.lock:
            if payload["request_id"] in self.seen_request_ids:
                self.publish_status("rejected", "duplicate request_id", {}, mission=payload)
                return
            if self.busy:
                self.publish_status("busy", "another mission already owns motion", {}, mission=payload)
                return
            self.busy = True
            self.cancel_requested = False
            self.current_mission = payload
            if len(self.seen_request_ids) >= 512:
                self.seen_request_ids.pop()
            self.seen_request_ids.add(payload["request_id"])
        self.publish_status("accepted", "mission accepted", {}, mission=payload)
        threading.Thread(target=self.run_route, args=(payload,), daemon=True).start()

    def parse_payload(self, text):
        text = (text or "").strip()
        if text:
            try:
                data = json.loads(text)
                if isinstance(data, dict):
                    return data
                raise ValueError("mission JSON root must be an object")
            except Exception as exc:
                raise ValueError("invalid mission JSON: %s" % exc)
        return {"route": self.load_route_file()}

    def load_route_file(self):
        if not self.route_file:
            return []
        try:
            with open(self.route_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            return data.get("route") or data.get("waypoints") or []
        except Exception as exc:
            self.publish_status("route_file_error", str(exc), {"route_file": self.route_file})
            return []

    def normalize_waypoints(self, payload):
        route = payload["route"]
        out = []
        for i, wp in enumerate(route):
            if not isinstance(wp, dict):
                continue
            item = dict(wp)
            item.setdefault("id", "wp_%02d" % (i + 1))
            item.setdefault("frame_id", self.default_frame)
            item["x"] = float(item["x"])
            item["y"] = float(item["y"])
            item["yaw"] = float(item.get("yaw", 0.0))
            expected = str(item.get("expected_class", "any")).strip().lower()
            aliases = {
                "water": "water_meter",
                "meter": "water_meter",
                "pressure": "pressure_gauge",
                "gauge": "pressure_gauge",
            }
            item["expected_class"] = aliases.get(expected, expected)
            if item["expected_class"] not in ("any", "water_meter", "pressure_gauge"):
                item["expected_class"] = "any"
            out.append(item)
        return out

    def current_pose_waypoint(self):
        if self.dry_run:
            return {"id": "home", "frame_id": self.default_frame, "x": 0.0, "y": 0.0, "yaw": 0.0}
        deadline = time.time() + max(0.1, self.current_pose_timeout)
        last_error = None
        while not rospy.is_shutdown() and time.time() < deadline:
            try:
                self.tf_listener.waitForTransform(
                    self.default_frame,
                    self.base_frame,
                    rospy.Time(0),
                    rospy.Duration(0.3),
                )
                trans, rot = self.tf_listener.lookupTransform(
                    self.default_frame,
                    self.base_frame,
                    rospy.Time(0),
                )
                yaw = tf.transformations.euler_from_quaternion(rot)[2]
                return {
                    "id": "home",
                    "frame_id": self.default_frame,
                    "x": float(trans[0]),
                    "y": float(trans[1]),
                    "yaw": float(yaw),
                }
            except Exception as exc:
                last_error = exc
                rospy.sleep(0.05)
        raise RuntimeError("failed to get current pose %s->%s: %s" % (
            self.default_frame,
            self.base_frame,
            last_error,
        ))

    def run_route(self, payload):
        started = time.time()
        waypoints = []
        results = []
        home_nav = None
        ok = False
        state = "error"
        message = ""
        current_index = None
        current_wp = None
        def store_point_result(index, point):
            if payload["loop"] and index < len(results):
                results[index] = point
            else:
                results.append(point)
        try:
            waypoints = self.normalize_waypoints(payload)
            if not waypoints:
                raise RuntimeError("empty route; publish JSON with route:[{id,x,y,yaw}]")
            home = None
            if payload["return_home"]:
                home = self.current_pose_waypoint()
                self.publish_status("home_recorded", "current pose recorded as home", {"home": home})
            if not self.dry_run:
                self.publish_status("waiting_move_base", "waiting for move_base", {})
                if not self.client.wait_for_server(rospy.Duration(10.0)):
                    raise RuntimeError("move_base action server not available")
            route_iterator = (itertools.cycle(enumerate(waypoints))
                              if payload["loop"] else enumerate(waypoints))
            for index, wp in route_iterator:
                current_index = index
                current_wp = wp
                self.current_point_index = index
                self.current_point_id = wp["id"]
                if self.cancel_requested or rospy.is_shutdown():
                    raise RuntimeError("cancelled")
                self.publish_status("navigating", "going to waypoint %s" % wp["id"], {
                    "index": index,
                    "waypoint": wp,
                })
                nav = self.navigate_to(wp)
                point = {
                    "request_id": payload["request_id"],
                    "mission_type": payload["mission_type"],
                    "point_index": index,
                    "point_id": wp["id"],
                    "waypoint": wp,
                    "navigation": nav,
                    "search": None,
                    "target": nav.get("detection"),
                    "kimi": None,
                }
                if self.cancel_requested:
                    raise RuntimeError("cancelled")
                if not nav.get("ok"):
                    store_point_result(index, point)
                    if payload["on_nav_failure"] == "stop":
                        raise RuntimeError("navigation failed at %s: %s" % (wp["id"], nav.get("state_text")))
                    continue
                self.stop_robot()
                self.publish_status("arrived", "arrived at waypoint %s" % wp["id"], {
                    "index": index,
                    "waypoint": wp,
                    "navigation": nav,
                })
                inspection = payload["inspection"]
                if not inspection["enabled"]:
                    store_point_result(index, point)
                    self.publish_status("point_complete", "navigation point complete", {
                        "index": index, "waypoint": wp,
                    })
                    continue
                self.publish_status("searching_target", "searching target at %s" % wp["id"], {
                    "index": index,
                    "waypoint": wp,
                    "detection": nav.get("detection"),
                })
                if inspection["vision_search"]:
                    search = self.search_target_at_waypoint(wp)
                else:
                    with self.lock:
                        self.search_active = True
                        self.detection_candidate = None
                        self.detection_stable_count = 0
                        self.last_detection_class = ""
                    try:
                        candidate = self.wait_for_detection_window(
                            self.search_settle_sec)
                        search = {
                            "ok": bool(candidate),
                            "state": "target_found" if candidate else "target_not_found",
                            "target": candidate,
                            "rotated_deg": 0.0,
                        }
                    finally:
                        with self.lock:
                            self.search_active = False
                point["search"] = search
                if search.get("target"):
                    point["target"] = search["target"]
                self.stop_robot()
                if search.get("ok"):
                    self.publish_status("target_confirmed", "reliable target confirmed at %s" % wp["id"], {
                        "index": index,
                        "waypoint": wp,
                        "target": point.get("target"),
                        "search": search,
                    })
                else:
                    self.publish_status("target_skipped", "no reliable target found at %s; skipping Kimi" % wp["id"], {
                        "index": index,
                        "waypoint": wp,
                        "search": search,
                    })
                if (search.get("ok") and inspection["ai_analysis"] and
                        self.enable_kimi_after_search):
                    self.publish_status("kimi_running", "running kimi inspection at %s" % wp["id"], {
                        "index": index,
                        "waypoint": wp,
                        "target": point.get("target"),
                    })
                    kimi = self.run_kimi_inspection(wp, point.get("target"))
                    point["kimi"] = kimi
                    self.publish_status("kimi_complete", "kimi inspection finished at %s" % wp["id"], {
                        "index": index,
                        "waypoint": wp,
                        "kimi": kimi,
                    })
                    if not kimi.get("ok") and self.stop_on_kimi_fail:
                        store_point_result(index, point)
                        raise RuntimeError("kimi inspection failed at %s: %s" % (wp["id"], kimi.get("error") or kimi.get("state")))
                store_point_result(index, point)
            if payload["return_home"] and not self.cancel_requested:
                self.publish_status("returning_home", "returning to start pose", {
                    "index": len(waypoints), "waypoint": home,
                })
                home_nav = self.navigate_to(home)
                if not home_nav.get("ok"):
                    raise RuntimeError("return home failed: %s" % home_nav.get("state_text"))
            ok = True
            state = "completed"
            message = "mission complete"
        except Exception as exc:
            message = str(exc)
            state = "cancelled" if message == "cancelled" else "error"
            self.publish_status(state, message, {"completed_points": len(results)})
        finally:
            self.stop_robot()
            result = {
                "ok": ok,
                "state": state,
                "stage": state,
                "message": message,
                "schema_version": 1,
                "request_id": payload.get("request_id"),
                "mission_type": payload.get("mission_type"),
                "point_index": current_index,
                "point_id": current_wp.get("id") if current_wp else None,
                "stamp": now_sec(),
                "elapsed_sec": round(time.time() - started, 3),
                "waypoint_count": len(waypoints),
                "home_navigation": home_nav,
                "results": results,
                "points": results,
            }
            encoded = String(json.dumps(result, ensure_ascii=False))
            self.result_pub.publish(encoded)
            self.legacy_result_pub.publish(encoded)
            self.publish_status(state, message, result)
            with self.lock:
                self.busy = False
                self.current_mission = {}
                self.current_point_index = None
                self.current_point_id = None

    def navigate_to(self, wp):
        if self.dry_run:
            rospy.sleep(0.5)
            return {"ok": True, "state": GoalStatus.SUCCEEDED, "state_text": "DRY_RUN"}
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = wp.get("frame_id", self.default_frame)
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["x"]
        goal.target_pose.pose.position.y = wp["y"]
        goal.target_pose.pose.orientation = quat_from_yaw(wp.get("yaw", 0.0))
        with self.lock:
            self.navigation_active = True
            self.expected_class = str(wp.get("expected_class", "any"))
            self.detection_candidate = None
            self.detection_stable_count = 0
            self.last_detection_class = ""
        self.client.send_goal(goal)
        deadline = time.time() + float(wp.get("nav_timeout", self.move_base_timeout))
        try:
            while not rospy.is_shutdown() and time.time() < deadline:
                if self.cancel_requested:
                    self.client.cancel_goal()
                    return {"ok": False, "state": -1, "state_text": "CANCELLED"}
                if self.client.wait_for_result(rospy.Duration(0.1)):
                    state = self.client.get_state()
                    return {
                        "ok": state == GoalStatus.SUCCEEDED,
                        "state": int(state),
                        "state_text": GOAL_STATUS_TEXT.get(state, str(state)),
                    }
            self.client.cancel_goal()
            return {"ok": False, "state": -1, "state_text": "TIMEOUT"}
        finally:
            with self.lock:
                self.navigation_active = False

    def current_detection_candidate(self):
        with self.lock:
            candidate = dict(self.detection_candidate) if self.detection_candidate else None
        if not candidate:
            return None
        if time.time() - candidate.get("stamp", 0.0) > self.search_max_age:
            return None
        if candidate.get("stable_frames", 0) < self.search_stable_frames:
            return None
        return candidate

    def search_target_at_waypoint(self, wp):
        if self.dry_run:
            rospy.sleep(0.3)
            return {"ok": True, "state": "DRY_RUN", "target": {"class_name": "any"}}
        deadline = time.time() + float(wp.get("search_timeout", self.search_timeout))
        angular_speed_deg = abs(float(wp.get("search_angular_speed_deg", self.search_angular_speed_deg)))
        angular_speed = math.radians(max(1.0, angular_speed_deg))
        max_rotation_deg = abs(float(wp.get("search_max_rotation_deg", self.search_max_rotation_deg)))
        step_deg = max(3.0, abs(float(wp.get("search_step_deg", self.search_step_deg))))
        step_rad = math.radians(step_deg)
        pause_sec = max(0.1, float(wp.get("search_step_pause_sec", self.search_step_pause_sec)))
        settle_sec = max(0.0, float(wp.get("search_settle_sec", self.search_settle_sec)))
        rotated_rad = 0.0
        with self.lock:
            self.search_active = True
            self.detection_candidate = None
            self.detection_stable_count = 0
            self.last_detection_class = ""
        try:
            if settle_sec > 0:
                self.publish_status("search_settling", "waiting briefly for static detection", {
                    "waypoint": wp,
                    "settle_sec": round(settle_sec, 2),
                })
                candidate = self.wait_for_detection_window(settle_sec)
                if candidate:
                    aligned = self.align_target_at_waypoint(wp)
                    if aligned.get("ok"):
                        return dict(aligned, rotated_deg=0.0)
            while not rospy.is_shutdown() and time.time() < deadline:
                if self.cancel_requested:
                    return {"ok": False, "state": "cancelled", "message": "cancelled"}
                if rotated_rad >= math.radians(max_rotation_deg):
                    self.stop_robot()
                    return {
                        "ok": False,
                        "state": "target_not_found",
                        "message": "no reliable target within stepped search window",
                        "rotated_deg": round(math.degrees(rotated_rad), 2),
                    }
                self.publish_status("search_rotating", "rotating to next search step", {
                    "waypoint": wp,
                    "step_deg": round(step_deg, 2),
                    "rotated_deg": round(math.degrees(rotated_rad), 2),
                })
                step_result = self.rotate_step(step_rad, angular_speed, wp)
                rotated_rad += step_result["rotated_rad"]
                if step_result.get("target"):
                    aligned = self.align_target_at_waypoint(wp)
                    if aligned.get("ok"):
                        return dict(aligned, rotated_deg=round(math.degrees(rotated_rad), 2))
                    if aligned.get("state") in ("cancelled", "invalid_detection", "align_timeout"):
                        return dict(aligned, rotated_deg=round(math.degrees(rotated_rad), 2))
                if not step_result["ok"]:
                    return {
                        "ok": False,
                        "state": step_result["state"],
                        "message": step_result.get("message", step_result["state"]),
                        "rotated_deg": round(math.degrees(rotated_rad), 2),
                    }
                self.publish_status("search_paused", "holding position for static recognition", {
                    "waypoint": wp,
                    "pause_sec": round(pause_sec, 2),
                    "rotated_deg": round(math.degrees(rotated_rad), 2),
                })
                candidate = self.wait_for_detection_window(pause_sec)
                if candidate:
                    aligned = self.align_target_at_waypoint(wp)
                    if aligned.get("ok"):
                        return dict(aligned, rotated_deg=round(math.degrees(rotated_rad), 2))
                    if aligned.get("state") in ("cancelled", "invalid_detection", "align_timeout"):
                        return dict(aligned, rotated_deg=round(math.degrees(rotated_rad), 2))
            return {
                "ok": False,
                "state": "search_timeout",
                "message": "target search timed out",
                "rotated_deg": round(math.degrees(rotated_rad), 2),
            }
        finally:
            self.stop_robot()
            with self.lock:
                self.search_active = False

    def wait_for_detection_window(self, duration_sec):
        deadline = time.time() + max(0.05, duration_sec)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return None
            candidate = self.current_detection_candidate()
            if candidate:
                return candidate
            rospy.sleep(0.08)
        return self.current_detection_candidate()

    def rotate_step(self, target_rad, angular_speed, wp):
        try:
            start_yaw = self.current_yaw()
        except Exception as exc:
            return {"ok": False, "state": "tf_unavailable", "message": str(exc), "rotated_rad": 0.0}
        rotated_rad = 0.0
        direction = 1.0
        while not rospy.is_shutdown() and rotated_rad < target_rad:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "message": "cancelled", "rotated_rad": rotated_rad}
            safe, reason = self.scan_is_safe_for_rotation()
            if not safe:
                self.stop_robot()
                return {"ok": False, "state": "rotation_sensor_stop", "message": reason, "rotated_rad": rotated_rad}
            candidate = self.current_detection_candidate()
            if candidate:
                self.stop_robot()
                self.publish_status("target_confirmed", "target detected while rotating", {
                    "waypoint": wp,
                    "target": candidate,
                    "rotated_deg": round(math.degrees(rotated_rad), 2),
                })
                return {"ok": True, "state": "target_found", "target": candidate, "rotated_rad": rotated_rad}
            self.publish_smooth_command(0.0, direction * angular_speed)
            try:
                current = self.current_yaw()
                rotated_rad = abs(self.angle_delta(current, start_yaw))
            except Exception as exc:
                self.stop_robot()
                return {"ok": False, "state": "tf_unavailable", "message": str(exc), "rotated_rad": rotated_rad}
            rospy.sleep(1.0 / self.control_rate_hz)
        self.stop_robot()
        return {"ok": True, "state": "step_complete", "rotated_rad": rotated_rad}

    def run_kimi_inspection(self, wp, target=None):
        if self.dry_run:
            rospy.sleep(0.2)
            return {"ok": True, "state": "DRY_RUN", "task": wp.get("kimi_task", self.kimi_task)}
        request_id = str(uuid.uuid4())
        payload = {
            "request_id": request_id,
            "task": str(wp.get("kimi_task", self.kimi_task)),
            "waypoint_id": wp.get("id"),
            "expected_class": wp.get("expected_class", "any"),
            "roi": self.roi_from_target(target),
        }
        with self.lock:
            self.kimi_results.pop(request_id, None)
        self.kimi_request_pub.publish(String(json.dumps(payload, ensure_ascii=False)))
        deadline = time.time() + float(wp.get("kimi_timeout", self.kimi_timeout))
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "request_id": request_id}
            with self.lock:
                data = self.kimi_results.get(request_id)
            if data:
                return data
            rospy.sleep(0.1)
        return {"ok": False, "state": "kimi_timeout", "request_id": request_id, "task": payload["task"]}

    def roi_from_target(self, target):
        if not isinstance(target, dict):
            return None
        try:
            width = float(target.get("image_width", 0.0))
            height = float(target.get("image_height", 0.0))
            x1, y1 = float(target["x1"]), float(target["y1"])
            x2, y2 = float(target["x2"]), float(target["y2"])
        except (KeyError, TypeError, ValueError):
            return None
        if width <= 0.0 or height <= 0.0 or x2 <= x1 or y2 <= y1:
            return None
        pad_x = (x2 - x1) * self.roi_padding_ratio
        pad_y = (y2 - y1) * self.roi_padding_ratio
        return {
            "x1": max(0.0, x1 - pad_x),
            "y1": max(0.0, y1 - pad_y),
            "x2": min(width, x2 + pad_x),
            "y2": min(height, y2 + pad_y),
            "image_width": width,
            "image_height": height,
            "padding_ratio": self.roi_padding_ratio,
        }

    def stop_robot(self):
        z = Twist()
        for _ in range(8):
            self.cmd_pub.publish(z)
            rospy.sleep(0.02)
        self.last_cmd_vx = 0.0
        self.last_cmd_wz = 0.0
        self.last_cmd_time = time.time()

    def publish_status(self, state, message, extra, mission=None):
        mission = mission or self.current_mission or {}
        waypoint = extra.get("waypoint") if isinstance(extra, dict) else None
        point_index = extra.get("index") if isinstance(extra, dict) else None
        if point_index is None and isinstance(extra, dict):
            point_index = extra.get("point_index")
        if point_index is None:
            point_index = self.current_point_index
        payload = {
            "schema_version": 1,
            "stamp": now_sec(),
            "state": state,
            "stage": state,
            "message": message,
            "busy": self.busy,
            "request_id": mission.get("request_id"),
            "mission_type": mission.get("mission_type"),
            "point_index": point_index,
            "point_id": (waypoint.get("id") if isinstance(waypoint, dict)
                         else self.current_point_id),
            "extra": extra,
        }
        encoded = String(json.dumps(payload, ensure_ascii=False))
        self.status_pub.publish(encoded)
        self.legacy_status_pub.publish(encoded)


def main():
    rospy.init_node("inspection_servo_route_runner")
    InspectionServoRouteRunner()
    rospy.spin()


if __name__ == "__main__":
    main()
