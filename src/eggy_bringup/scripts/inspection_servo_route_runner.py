#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Route runner that navigates to each waypoint, searches target, then calls Kimi.

This node is intentionally narrow: it does not change move_base or Qt's
existing single-goal bridge. It only sequences navigation, target search, and
Kimi analysis.
"""

import json
import math
import threading
import time
import uuid

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import tf


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
        self.search_stable_frames = int(rospy.get_param("~search_stable_frames", 3))
        self.search_max_age = float(rospy.get_param("~search_max_age", 0.8))
        self.search_timeout = float(rospy.get_param("~search_timeout", 24.0))
        self.search_angular_speed_deg = float(rospy.get_param("~search_angular_speed_deg", 18.0))
        self.search_max_rotation_deg = float(rospy.get_param("~search_max_rotation_deg", 360.0))
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        self.lock = threading.Lock()
        self.busy = False
        self.cancel_requested = False
        self.kimi_results = {}
        self.navigation_active = False
        self.expected_class = "any"
        self.detection_candidate = None
        self.detection_stable_count = 0
        self.last_detection_class = ""

        self.status_pub = rospy.Publisher("/inspection_servo_route/status", String, queue_size=10, latch=True)
        self.result_pub = rospy.Publisher("/inspection_servo_route/result", String, queue_size=10, latch=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.kimi_request_pub = rospy.Publisher("/kimi_inspection/request", String, queue_size=5)

        rospy.Subscriber("/inspection_servo_route/request", String, self.on_request, queue_size=5)
        rospy.Subscriber("/kimi_inspection/result", String, self.on_kimi_result, queue_size=10)
        rospy.Subscriber("/meter/detection", String, self.on_detection, queue_size=10)

        self.tf_listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
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
                    "search": None,
                    "target": nav.get("detection"),
                    "kimi": None,
                }
                if not nav.get("ok"):
                    results.append(point)
                    if self.stop_on_nav_fail:
                        raise RuntimeError("navigation failed at %s: %s" % (wp["id"], nav.get("state_text")))
                    continue
                self.stop_robot()
                self.publish_status("searching_target", "searching target at %s" % wp["id"], {
                    "index": index,
                    "waypoint": wp,
                    "detection": nav.get("detection"),
                })
                search = self.search_target_at_waypoint(wp)
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
                if search.get("ok") and self.enable_kimi_after_search:
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
        rotated_rad = 0.0
        prev_time = time.time()
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "message": "cancelled"}
            candidate = self.current_detection_candidate()
            if candidate:
                self.stop_robot()
                return {
                    "ok": True,
                    "state": "target_found",
                    "target": candidate,
                    "rotated_deg": round(math.degrees(rotated_rad), 2),
                }
            now = time.time()
            dt = max(0.0, now - prev_time)
            prev_time = now
            rotated_rad += angular_speed * dt
            if rotated_rad >= math.radians(max_rotation_deg):
                self.stop_robot()
                return {
                    "ok": False,
                    "state": "target_not_found",
                    "message": "no reliable target within one rotation",
                    "rotated_deg": round(math.degrees(rotated_rad), 2),
                }
            cmd = Twist()
            cmd.angular.z = angular_speed
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
        self.stop_robot()
        return {
            "ok": False,
            "state": "search_timeout",
            "message": "target search timed out",
            "rotated_deg": round(math.degrees(rotated_rad), 2),
        }

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
