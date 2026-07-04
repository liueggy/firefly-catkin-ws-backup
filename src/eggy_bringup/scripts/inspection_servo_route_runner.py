#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Route runner that calls meter_visual_servo.py at each waypoint.

This node is intentionally narrow: it does not change move_base, Qt's existing
single-goal bridge, or meter_visual_servo.py. It only sequences them.
"""

import json
import math
import os
import signal
import subprocess
import threading
import time
import uuid

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String


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
        self.servo_timeout = float(rospy.get_param("~servo_timeout", 60.0))
        self.servo_stable_sec = float(rospy.get_param("~servo_stable_sec", 0.6))
        self.servo_startup_wait = float(rospy.get_param("~servo_startup_wait", 0.35))
        self.servo_shutdown_timeout = float(rospy.get_param("~servo_shutdown_timeout", 3.0))
        self.enable_kimi_after_servo = bool(rospy.get_param("~enable_kimi_after_servo", True))
        self.kimi_timeout = float(rospy.get_param("~kimi_timeout", 90.0))
        self.kimi_task = rospy.get_param("~kimi_task", "meter")
        self.stop_on_nav_fail = bool(rospy.get_param("~stop_on_nav_fail", True))
        self.stop_on_servo_fail = bool(rospy.get_param("~stop_on_servo_fail", True))
        self.stop_on_kimi_fail = bool(rospy.get_param("~stop_on_kimi_fail", False))
        self.intercept_during_navigation = bool(
            rospy.get_param("~intercept_during_navigation", True)
        )
        self.intercept_min_score = float(rospy.get_param("~intercept_min_score", 0.75))
        self.intercept_stable_frames = int(rospy.get_param("~intercept_stable_frames", 4))
        self.intercept_max_age = float(rospy.get_param("~intercept_max_age", 0.7))
        self.intercept_cooldown = float(rospy.get_param("~intercept_cooldown", 4.0))
        self.servo_min_score = float(rospy.get_param("~servo_min_score", 0.65))
        self.servo_initial_search_timeout = float(
            rospy.get_param("~servo_initial_search_timeout", 25.0)
        )
        self.verify_localization_between_points = bool(
            rospy.get_param("~verify_localization_between_points", True)
        )
        self.localization_confirm_timeout = float(
            rospy.get_param("~localization_confirm_timeout", 6.0)
        )
        self.localization_xy_variance_max = float(
            rospy.get_param("~localization_xy_variance_max", 0.20)
        )
        self.localization_yaw_variance_max = float(
            rospy.get_param("~localization_yaw_variance_max", 0.12)
        )
        self.localization_stable_samples = int(
            rospy.get_param("~localization_stable_samples", 3)
        )
        self.auto_relocalize_between_points = bool(
            rospy.get_param("~auto_relocalize_between_points", True)
        )
        self.auto_relocalize_timeout = float(rospy.get_param("~auto_relocalize_timeout", 45.0))
        self.auto_relocalize_angular_speed_deg = float(
            rospy.get_param("~auto_relocalize_angular_speed_deg", 30.0)
        )
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        self.lock = threading.Lock()
        self.busy = False
        self.cancel_requested = False
        self.latest_servo_status = {}
        self.kimi_results = {}
        self.servo_proc = None
        self.navigation_active = False
        self.expected_class = "any"
        self.detection_candidate = None
        self.detection_stable_count = 0
        self.last_detection_class = ""
        self.last_intercept_time = 0.0
        self.latest_amcl_quality = None
        self.latest_relocalization_status = {}

        self.status_pub = rospy.Publisher("/inspection_servo_route/status", String, queue_size=10, latch=True)
        self.result_pub = rospy.Publisher("/inspection_servo_route/result", String, queue_size=10, latch=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.servo_request_pub = rospy.Publisher("/meter_visual_servo/request", String, queue_size=5)
        self.kimi_request_pub = rospy.Publisher("/kimi_inspection/request", String, queue_size=5)
        self.relocalization_request_pub = rospy.Publisher(
            "/eggy/relocalization/request", String, queue_size=5
        )

        rospy.Subscriber("/inspection_servo_route/request", String, self.on_request, queue_size=5)
        rospy.Subscriber("/meter_visual_servo/status", String, self.on_servo_status, queue_size=20)
        rospy.Subscriber("/kimi_inspection/result", String, self.on_kimi_result, queue_size=10)
        rospy.Subscriber("/meter/detection", String, self.on_detection, queue_size=10)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.on_amcl_pose, queue_size=10)
        rospy.Subscriber(
            "/eggy/relocalization/status", String, self.on_relocalization_status, queue_size=10
        )

        self.tf_listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.publish_status("ready", "inspection servo route runner ready", {})
        rospy.loginfo("inspection_servo_route_runner ready dry_run=%s", self.dry_run)

    def on_servo_status(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.latest_servo_status = data

    def on_kimi_result(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        request_id = data.get("request_id")
        if request_id:
            with self.lock:
                self.kimi_results[str(request_id)] = data

    def on_amcl_pose(self, msg):
        cov = msg.pose.covariance
        quality = {
            "stamp": time.time(),
            "xy_variance": max(float(cov[0]), float(cov[7])),
            "yaw_variance": float(cov[35]),
        }
        with self.lock:
            self.latest_amcl_quality = quality

    def on_relocalization_status(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.latest_relocalization_status = data

    def on_detection(self, msg):
        try:
            payload = json.loads(msg.data)
            detections = payload.get("detections") or []
        except Exception:
            return

        with self.lock:
            if not self.navigation_active:
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
            if score < self.intercept_min_score:
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
            if class_name == self.last_detection_class:
                self.detection_stable_count += 1
            else:
                self.last_detection_class = class_name
                self.detection_stable_count = 1
            self.detection_candidate = {
                "class_name": class_name,
                "score": float(best.get("score", 0.0)),
                "stable_frames": self.detection_stable_count,
                "stamp": time.time(),
                "box": {
                    key: best.get(key)
                    for key in ("x1", "y1", "x2", "y2")
                    if key in best
                },
            }

    def on_request(self, msg):
        payload = self.parse_payload(msg.data)
        command = str(payload.get("command", "start")).lower()
        if command in ("stop", "cancel"):
            self.cancel_requested = True
            self.client.cancel_all_goals()
            self.stop_servo_node()
            self.stop_robot()
            self.publish_status("cancelled", "route cancel requested", {})
            return
        with self.lock:
            if self.busy:
                self.publish_status("busy", "inspection servo route already running", {})
                return
            self.busy = True
            self.cancel_requested = False
        threading.Thread(target=self.run_route, args=(payload,), daemon=True).start()

    def parse_payload(self, text):
        text = (text or "").strip()
        if text:
            try:
                data = json.loads(text)
                if isinstance(data, dict):
                    return data
            except Exception as exc:
                self.publish_status("bad_request", str(exc), {"raw": text})
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
        route = payload.get("route") or payload.get("waypoints") or []
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
        try:
            waypoints = self.normalize_waypoints(payload)
            if not waypoints:
                raise RuntimeError("empty route; publish JSON with route:[{id,x,y,yaw}]")
            home = self.current_pose_waypoint()
            self.publish_status("home_recorded", "current pose recorded as home", {"home": home})
            if not self.dry_run:
                self.publish_status("waiting_move_base", "waiting for move_base", {})
                if not self.client.wait_for_server(rospy.Duration(10.0)):
                    raise RuntimeError("move_base action server not available")
            for index, wp in enumerate(waypoints):
                if self.cancel_requested or rospy.is_shutdown():
                    raise RuntimeError("cancelled")
                self.publish_status("navigating", "going to waypoint %s" % wp["id"], {
                    "index": index,
                    "waypoint": wp,
                })
                nav = self.navigate_to(wp)
                point = {
                    "waypoint": wp,
                    "navigation": nav,
                    "servo": None,
                    "target": nav.get("detection"),
                    "kimi": None,
                    "localization": None,
                }
                if not nav.get("ok"):
                    results.append(point)
                    if self.stop_on_nav_fail:
                        raise RuntimeError("navigation failed at %s: %s" % (wp["id"], nav.get("state_text")))
                    continue
                self.stop_robot()
                self.publish_status("servo_starting", "starting visual servo at %s" % wp["id"], {
                    "index": index,
                    "waypoint": wp,
                    "intercepted": bool(nav.get("intercepted")),
                    "detection": nav.get("detection"),
                })
                servo = self.run_visual_servo(wp)
                point["servo"] = servo
                if servo.get("status"):
                    point["target"] = servo["status"]
                self.stop_servo_node()
                self.stop_robot()
                if not servo.get("ok") and self.stop_on_servo_fail:
                    results.append(point)
                    raise RuntimeError("visual servo failed at %s: %s" % (wp["id"], servo.get("state")))
                self.publish_status("target_confirmed", "reliable target confirmed at %s" % wp["id"], {
                    "index": index,
                    "waypoint": wp,
                    "target": point.get("target"),
                    "servo": servo,
                })
                if self.enable_kimi_after_servo:
                    self.publish_status("kimi_running", "running kimi inspection at %s" % wp["id"], {
                        "index": index,
                        "waypoint": wp,
                        "target": point.get("target"),
                    })
                    kimi = self.run_kimi_inspection(wp)
                    point["kimi"] = kimi
                    self.publish_status("kimi_complete", "kimi inspection finished at %s" % wp["id"], {
                        "index": index,
                        "waypoint": wp,
                        "kimi": kimi,
                    })
                    if not kimi.get("ok") and self.stop_on_kimi_fail:
                        results.append(point)
                        raise RuntimeError("kimi inspection failed at %s: %s" % (wp["id"], kimi.get("error") or kimi.get("state")))
                localization = self.ensure_localization_before_next(wp, index)
                point["localization"] = localization
                if not localization.get("ok"):
                    results.append(point)
                    raise RuntimeError("localization check failed after %s: %s" % (wp["id"], localization.get("message")))
                results.append(point)
            if not self.cancel_requested:
                self.publish_status("returning_home", "returning to start pose", {"home": home})
                home_nav = self.navigate_to(home)
                if not home_nav.get("ok"):
                    raise RuntimeError("return home failed: %s" % home_nav.get("state_text"))
            ok = True
            state = "complete"
            message = "inspection servo route complete"
        except Exception as exc:
            message = str(exc)
            self.publish_status("error", message, {"completed_points": len(results)})
        finally:
            self.stop_servo_node()
            self.stop_robot()
            result = {
                "ok": ok,
                "state": state,
                "stage": state,
                "message": message,
                "stamp": now_sec(),
                "elapsed_sec": round(time.time() - started, 3),
                "waypoint_count": len(waypoints),
                "home_navigation": home_nav,
                "results": results,
                "points": results,
            }
            self.result_pub.publish(String(json.dumps(result, ensure_ascii=False)))
            self.publish_status(state, message, result)
            with self.lock:
                self.busy = False

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
                        "intercepted": False,
                    }
                candidate = None
                with self.lock:
                    if self.detection_candidate is not None:
                        candidate = dict(self.detection_candidate)
                fresh = candidate and time.time() - candidate["stamp"] <= self.intercept_max_age
                cooled_down = time.time() - self.last_intercept_time >= self.intercept_cooldown
                stable = candidate and candidate["stable_frames"] >= self.intercept_stable_frames
                allow_intercept = bool(
                    wp.get("allow_vision_intercept", wp.get("id") != "home")
                )
                if allow_intercept and self.intercept_during_navigation and fresh and cooled_down and stable:
                    self.publish_status(
                        "target_intercept",
                        "stable meter target detected; pausing navigation",
                        {"waypoint": wp, "detection": candidate},
                    )
                    self.client.cancel_goal()
                    self.client.wait_for_result(rospy.Duration(1.5))
                    self.stop_robot()
                    self.last_intercept_time = time.time()
                    return {
                        "ok": True,
                        "state": GoalStatus.PREEMPTED,
                        "state_text": "VISION_INTERCEPT",
                        "intercepted": True,
                        "detection": candidate,
                    }
            self.client.cancel_goal()
            return {"ok": False, "state": -1, "state_text": "TIMEOUT"}
        finally:
            with self.lock:
                self.navigation_active = False

    def run_visual_servo(self, wp):
        if self.dry_run:
            rospy.sleep(0.5)
            return {"ok": True, "state": "DRY_RUN", "status": {}}
        self.start_servo_node()
        deadline = time.time() + float(wp.get("servo_timeout", self.servo_timeout))
        hold_started = None
        last_status = {}
        good_states = set(["target_size_reached", "target_size_hold"])
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "status": last_status}
            with self.lock:
                last_status = dict(self.latest_servo_status)
            state = str(last_status.get("state", ""))
            if state in good_states:
                if hold_started is None:
                    hold_started = time.time()
                if time.time() - hold_started >= float(wp.get("servo_stable_sec", self.servo_stable_sec)):
                    return {
                        "ok": True,
                        "state": state,
                        "stable_sec": round(time.time() - hold_started, 3),
                        "status": last_status,
                    }
            else:
                hold_started = None
            rospy.sleep(0.1)
        return {"ok": False, "state": "servo_timeout", "status": last_status}

    def run_kimi_inspection(self, wp):
        if self.dry_run:
            rospy.sleep(0.2)
            return {"ok": True, "state": "DRY_RUN", "task": wp.get("kimi_task", self.kimi_task)}
        request_id = str(uuid.uuid4())
        payload = {
            "request_id": request_id,
            "task": str(wp.get("kimi_task", self.kimi_task)),
            "waypoint_id": wp.get("id"),
            "expected_class": wp.get("expected_class", "any"),
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

    def current_localization_quality(self):
        with self.lock:
            quality = dict(self.latest_amcl_quality) if self.latest_amcl_quality else None
        if not quality:
            return None
        quality["age"] = round(time.time() - quality.get("stamp", 0.0), 3)
        quality["ok"] = (
            quality["age"] <= 1.0
            and quality["xy_variance"] <= self.localization_xy_variance_max
            and quality["yaw_variance"] <= self.localization_yaw_variance_max
        )
        return quality

    def wait_for_localization_stable(self, timeout_sec):
        if self.dry_run:
            return {"ok": True, "state": "DRY_RUN"}
        deadline = time.time() + max(0.1, timeout_sec)
        stable = 0
        last_quality = None
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "message": "cancelled"}
            quality = self.current_localization_quality()
            if quality:
                last_quality = quality
                stable = stable + 1 if quality.get("ok") else 0
                if stable >= self.localization_stable_samples:
                    quality["stable_samples"] = stable
                    return {"ok": True, "state": "stable", "quality": quality}
            rospy.sleep(0.2)
        return {
            "ok": False,
            "state": "unstable",
            "message": "AMCL pose covariance is not stable",
            "quality": last_quality,
        }

    def request_auto_relocalization(self, wp):
        request = {
            "command": "start",
            "timeout": self.auto_relocalize_timeout,
            "angular_speed_deg": self.auto_relocalize_angular_speed_deg,
            "reason": "inspection_between_waypoints",
            "waypoint_id": wp.get("id"),
        }
        start_time = time.time()
        with self.lock:
            self.latest_relocalization_status = {}
        self.relocalization_request_pub.publish(String(json.dumps(request, ensure_ascii=False)))
        deadline = time.time() + self.auto_relocalize_timeout + 5.0
        last_status = {}
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "message": "cancelled"}
            with self.lock:
                last_status = dict(self.latest_relocalization_status)
            if float(last_status.get("stamp", 0.0)) >= start_time - 0.5:
                state = str(last_status.get("state", ""))
                if state in ("success", "dry_run_complete"):
                    return {"ok": True, "state": state, "status": last_status}
                if state in ("failed", "rejected", "cancelled"):
                    return {
                        "ok": False,
                        "state": state,
                        "message": last_status.get("message", state),
                        "status": last_status,
                    }
            rospy.sleep(0.2)
        return {
            "ok": False,
            "state": "timeout",
            "message": "auto relocalization timed out",
            "status": last_status,
        }

    def ensure_localization_before_next(self, wp, index):
        if not self.verify_localization_between_points:
            return {"ok": True, "state": "skipped", "message": "disabled"}
        self.publish_status("localization_checking", "checking AMCL localization after %s" % wp["id"], {
            "index": index,
            "waypoint": wp,
        })
        stable = self.wait_for_localization_stable(self.localization_confirm_timeout)
        if stable.get("ok"):
            self.publish_status("localization_stable", "AMCL localization is stable", {
                "index": index,
                "waypoint": wp,
                "localization": stable,
            })
            return stable
        if not self.auto_relocalize_between_points:
            return stable
        self.publish_status("relocalizing", "AMCL unstable; requesting automatic relocalization", {
            "index": index,
            "waypoint": wp,
            "localization": stable,
        })
        relocalized = self.request_auto_relocalization(wp)
        if not relocalized.get("ok"):
            return relocalized
        confirmed = self.wait_for_localization_stable(self.localization_confirm_timeout)
        confirmed["relocalization"] = relocalized
        return confirmed

    def start_servo_node(self):
        self.stop_servo_node()
        with self.lock:
            self.latest_servo_status = {}
        initial_search_timeout = self.servo_initial_search_timeout
        env = os.environ.copy()
        self.servo_proc = subprocess.Popen(
            [
                "rosrun",
                "eggy_bringup",
                "meter_visual_servo.py",
                "_target_class:=%s" % self.expected_class,
                "_min_score:=%.3f" % self.servo_min_score,
                "_initial_search_timeout:=%.3f" % initial_search_timeout,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            env=env,
        )
        rospy.sleep(self.servo_startup_wait)
        self.servo_request_pub.publish(String(json.dumps({
            "command": "start",
            "target_class": self.expected_class,
        })))

    def stop_servo_node(self):
        self.servo_request_pub.publish(String("stop"))
        if self.servo_proc is not None:
            proc = self.servo_proc
            self.servo_proc = None
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    deadline = time.time() + self.servo_shutdown_timeout
                    while proc.poll() is None and time.time() < deadline:
                        time.sleep(0.05)
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception as exc:
                rospy.logwarn("failed to stop meter_visual_servo process: %s", exc)

    def stop_robot(self):
        z = Twist()
        for _ in range(8):
            self.cmd_pub.publish(z)
            time.sleep(0.025)

    def publish_status(self, state, message, extra):
        payload = {
            "stamp": now_sec(),
            "state": state,
            "stage": state,
            "message": message,
            "busy": self.busy,
            "extra": extra,
        }
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))


def main():
    rospy.init_node("inspection_servo_route_runner")
    InspectionServoRouteRunner()
    rospy.spin()


if __name__ == "__main__":
    main()
