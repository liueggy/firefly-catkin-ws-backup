#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Closed-loop inspection cycle manager.

This node owns the high-level inspection state machine:

Qt route -> waypoint navigation -> target search/alignment -> reading request
-> rule evaluation -> optional pressure-A scan -> next waypoint -> home -> wait.

It reuses the existing inspection_target_aligner node as the vision/20 cm
alignment primitive and keeps cloud/OCR reading logic outside the robot.
"""

import base64
import json
import math
import os
import threading
import time
import uuid

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import CompressedImage
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


TASK_ALIASES = {
    "pressure": "pressure_gauge",
    "pressure_gauge": "pressure_gauge",
    "gauge": "pressure_gauge",
    "water": "water_meter",
    "water_meter": "water_meter",
    "meter": "water_meter",
}


def quat_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def now_sec():
    return rospy.Time.now().to_sec() if not rospy.is_shutdown() else time.time()


class InspectionCycleManager:
    def __init__(self):
        self.route_file = rospy.get_param("~route_file", "")
        self.history_file = rospy.get_param(
            "~history_file", "/root/catkin_ws/runtime/inspection_history.json"
        )
        self.capture_dir = rospy.get_param("~capture_dir", "/tmp/inspection_scan")
        self.default_frame = rospy.get_param("~default_frame", "map")
        self.loop_enabled_default = bool(rospy.get_param("~loop_enabled_default", True))
        self.default_interval_sec = float(rospy.get_param("~default_interval_sec", 360.0))
        self.fast_interval_sec = float(rospy.get_param("~fast_interval_sec", 240.0))
        self.move_base_timeout = float(rospy.get_param("~move_base_timeout", 120.0))
        self.align_timeout = float(rospy.get_param("~align_timeout", 70.0))
        self.reading_timeout = float(rospy.get_param("~reading_timeout", 45.0))
        self.pressure_low = float(rospy.get_param("~pressure_low", 6.0))
        self.pressure_high = float(rospy.get_param("~pressure_high", 8.0))
        self.pressure_near_margin = float(rospy.get_param("~pressure_near_margin", 0.5))
        self.water_delta_threshold = float(rospy.get_param("~water_delta_threshold", 100.0))
        self.scan_lateral_speed = abs(float(rospy.get_param("~scan_lateral_speed", 0.12)))
        self.scan_capture_interval = float(rospy.get_param("~scan_capture_interval", 0.5))
        self.scan_left_positive = bool(rospy.get_param("~scan_left_positive", True))
        self.image_topic = rospy.get_param("~image_topic", "/camera/front/image/compressed")
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        self.lock = threading.Lock()
        self.busy = False
        self.cancel_requested = False
        self.fast_mode = False
        self.history = self.load_history()
        self.align_results = {}
        self.reading_results = {}
        self.latest_image = None
        self.latest_image_time = 0.0

        self.status_pub = rospy.Publisher("/inspection_cycle/status", String, queue_size=10, latch=True)
        self.result_pub = rospy.Publisher("/inspection_cycle/result", String, queue_size=10, latch=True)
        self.alert_pub = rospy.Publisher("/inspection/alert", String, queue_size=10, latch=True)
        self.frequency_pub = rospy.Publisher("/inspection/frequency/status", String, queue_size=10, latch=True)
        self.scan_capture_pub = rospy.Publisher("/inspection_scan/capture", String, queue_size=50)
        self.reading_request_pub = rospy.Publisher("/inspection_reading/request", String, queue_size=10)
        self.align_request_pub = rospy.Publisher("/inspection_align/request", String, queue_size=10)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/inspection_cycle/request", String, self.on_request, queue_size=5)
        rospy.Subscriber("/inspection_route/request", String, self.on_request, queue_size=5)
        rospy.Subscriber("/inspection_align/result", String, self.on_align_result, queue_size=20)
        rospy.Subscriber("/inspection_reading/result", String, self.on_reading_result, queue_size=20)
        rospy.Subscriber(self.image_topic, CompressedImage, self.on_image, queue_size=1)

        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.publish_status("ready", "inspection cycle manager ready", {})
        self.publish_frequency("normal", self.default_interval_sec, "startup")
        rospy.loginfo("inspection_cycle_manager ready dry_run=%s", self.dry_run)

    def on_image(self, msg):
        with self.lock:
            self.latest_image = msg
            self.latest_image_time = time.time()

    def on_align_result(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        rid = data.get("request_id")
        if rid:
            with self.lock:
                self.align_results[str(rid)] = data

    def on_reading_result(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        rid = data.get("request_id") or data.get("reading_request_id")
        if rid:
            with self.lock:
                self.reading_results[str(rid)] = data

    def on_request(self, msg):
        payload = self.parse_payload(msg.data)
        command = str(payload.get("command", "start")).lower()
        if command in ("stop", "cancel"):
            self.cancel_requested = True
            self.client.cancel_all_goals()
            self.stop_robot()
            self.align_request_pub.publish(String(json.dumps({"command": "cancel"}, ensure_ascii=False)))
            self.publish_status("cancelled", "cycle cancel requested", {})
            return
        with self.lock:
            if self.busy:
                self.publish_status("busy", "inspection cycle already running", {})
                return
            self.busy = True
            self.cancel_requested = False
        threading.Thread(target=self.run_cycles, args=(payload,), daemon=True).start()

    def parse_payload(self, text):
        text = (text or "").strip()
        if text:
            try:
                data = json.loads(text)
                if isinstance(data, dict):
                    return data
            except Exception:
                pass
        return {"route": self.load_route_file(), "loop": self.loop_enabled_default}

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
            item.setdefault("tasks", self.tasks_from_waypoint(item, payload))
            item["x"] = float(item["x"])
            item["y"] = float(item["y"])
            item["yaw"] = float(item.get("yaw", 0.0))
            out.append(item)
        return out

    def tasks_from_waypoint(self, wp, payload):
        raw = wp.get("tasks", wp.get("task", wp.get("target", payload.get("target", "any"))))
        if isinstance(raw, list):
            values = raw
        elif str(raw).lower() in ("both", "all", "any", "auto", "*"):
            values = ["pressure_gauge", "water_meter"]
        else:
            values = [raw]
        tasks = []
        for val in values:
            key = str(val).strip().lower()
            tasks.append(TASK_ALIASES.get(key, key))
        return [t for t in tasks if t in ("pressure_gauge", "water_meter")]

    def home_from_payload(self, payload, waypoints):
        home = payload.get("home") or payload.get("start") or payload.get("start_pose")
        if not home:
            for wp in waypoints:
                if wp.get("is_home") or str(wp.get("id", "")).lower() in ("home", "start", "origin"):
                    home = wp
                    break
        if not home:
            return None
        h = dict(home)
        h.setdefault("id", "home")
        h.setdefault("frame_id", self.default_frame)
        h["x"] = float(h["x"])
        h["y"] = float(h["y"])
        h["yaw"] = float(h.get("yaw", 0.0))
        return h

    def run_cycles(self, payload):
        loop_enabled = bool(payload.get("loop", payload.get("repeat", self.loop_enabled_default)))
        max_cycles = int(payload.get("max_cycles", 0))
        waypoints = self.normalize_waypoints(payload)
        home = self.home_from_payload(payload, waypoints)
        cycle_index = 0
        all_results = []
        try:
            if not waypoints:
                raise RuntimeError("empty route")
            self.publish_status("waiting_move_base", "waiting for move_base", {})
            if not self.client.wait_for_server(rospy.Duration(10.0)):
                raise RuntimeError("move_base action server not available")
            while not rospy.is_shutdown() and not self.cancel_requested:
                cycle_index += 1
                cycle_id = payload.get("cycle_id") or time.strftime("%Y%m%d_%H%M%S")
                cycle_id = "%s_%03d" % (cycle_id, cycle_index)
                result = self.run_one_cycle(cycle_id, waypoints, home, payload)
                all_results.append(result)
                self.result_pub.publish(String(json.dumps(result, ensure_ascii=False)))
                if max_cycles > 0 and cycle_index >= max_cycles:
                    break
                if not loop_enabled:
                    break
                interval = self.fast_interval_sec if self.fast_mode else self.default_interval_sec
                self.publish_status("waiting_next_cycle", "cycle complete, waiting next interval", {
                    "cycle_id": cycle_id,
                    "interval_sec": interval,
                    "fast_mode": self.fast_mode,
                })
                self.sleep_interruptible(interval)
        except Exception as exc:
            self.publish_status("error", str(exc), {"completed_cycles": len(all_results)})
        finally:
            self.stop_robot()
            with self.lock:
                self.busy = False

    def run_one_cycle(self, cycle_id, waypoints, home, payload):
        started = time.time()
        cycle_fast_triggered = False
        point_results = []
        ok = True
        for index, wp in enumerate(waypoints):
            if self.cancel_requested or rospy.is_shutdown():
                raise RuntimeError("cancelled")
            self.publish_status("navigating", "going to waypoint %s" % wp["id"], {
                "cycle_id": cycle_id,
                "index": index,
                "waypoint": wp,
            })
            nav = self.navigate_to(wp)
            if not nav.get("ok"):
                ok = False
                point_results.append({"waypoint": wp, "navigation": nav, "state": "navigation_failed"})
                if bool(payload.get("stop_on_nav_fail", True)):
                    break
                continue
            self.stop_robot()
            point_result = self.handle_waypoint(cycle_id, wp)
            point_results.append(point_result)
            if point_result.get("fast_triggered"):
                cycle_fast_triggered = True
            if not point_result.get("ok"):
                ok = False
                if bool(payload.get("stop_on_point_fail", False)):
                    break
        if home and not self.cancel_requested:
            self.publish_status("returning_home", "returning to home", {"cycle_id": cycle_id, "home": home})
            home_nav = self.navigate_to(home)
        else:
            home_nav = None
        self.fast_mode = bool(cycle_fast_triggered)
        self.publish_frequency(
            "fast" if self.fast_mode else "normal",
            self.fast_interval_sec if self.fast_mode else self.default_interval_sec,
            "cycle_rule_evaluation",
        )
        return {
            "ok": ok,
            "cycle_id": cycle_id,
            "stamp": now_sec(),
            "elapsed_sec": round(time.time() - started, 3),
            "fast_mode": self.fast_mode,
            "home_navigation": home_nav,
            "points": point_results,
        }

    def handle_waypoint(self, cycle_id, wp):
        tasks = wp.get("tasks") or []
        align_target = "both" if len(set(tasks)) > 1 else (tasks[0] if tasks else "any")
        result = {
            "ok": False,
            "cycle_id": cycle_id,
            "waypoint_id": wp["id"],
            "tasks": tasks,
            "align": None,
            "reading": None,
            "evaluation": None,
            "scan": None,
            "fast_triggered": False,
        }
        self.publish_status("aligning", "searching and aligning at waypoint %s" % wp["id"], result)
        align = self.call_aligner(wp, align_target)
        result["align"] = align
        if not align.get("ok"):
            result["evaluation"] = {"state": "align_failed", "reason": align.get("state")}
            return result
        reading = self.request_reading(cycle_id, wp, tasks, align)
        result["reading"] = reading
        if not reading.get("ok"):
            result["evaluation"] = {"state": "reading_failed", "reason": reading.get("state")}
            return result
        evaluation = self.evaluate_readings(cycle_id, wp, reading)
        result["evaluation"] = evaluation
        result["fast_triggered"] = bool(evaluation.get("fast_triggered"))
        if evaluation.get("pressure_state") == "abnormal_a":
            result["scan"] = self.run_pressure_a_scan(cycle_id, wp)
        if evaluation.get("pressure_state") == "abnormal_b":
            self.publish_alert("pressure_high", cycle_id, wp, evaluation)
        result["ok"] = True
        self.save_history()
        return result

    def navigate_to(self, wp):
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = wp.get("frame_id", self.default_frame)
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["x"]
        goal.target_pose.pose.position.y = wp["y"]
        goal.target_pose.pose.orientation = quat_from_yaw(wp.get("yaw", 0.0))
        if self.dry_run:
            rospy.sleep(0.5)
            return {"ok": True, "state": GoalStatus.SUCCEEDED, "state_text": "DRY_RUN"}
        self.client.send_goal(goal)
        done = self.client.wait_for_result(rospy.Duration(float(wp.get("nav_timeout", self.move_base_timeout))))
        if not done:
            self.client.cancel_goal()
            return {"ok": False, "state": -1, "state_text": "TIMEOUT"}
        state = self.client.get_state()
        return {"ok": state == GoalStatus.SUCCEEDED, "state": int(state), "state_text": GOAL_STATUS_TEXT.get(state, str(state))}

    def call_aligner(self, wp, target):
        request_id = str(uuid.uuid4())
        req = {
            "request_id": request_id,
            "waypoint_id": wp["id"],
            "target": target,
            "search_timeout": float(wp.get("search_timeout", 25.0)),
            "align_timeout": float(wp.get("align_timeout", 35.0)),
        }
        if "desired_bbox_height_px" in wp:
            req["desired_bbox_height_px"] = float(wp["desired_bbox_height_px"])
        with self.lock:
            self.align_results.pop(request_id, None)
        for _ in range(3):
            self.align_request_pub.publish(String(json.dumps(req, ensure_ascii=False)))
            rospy.sleep(0.1)
        deadline = time.time() + float(wp.get("full_align_timeout", self.align_timeout))
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "request_id": request_id}
            with self.lock:
                data = self.align_results.get(request_id)
            if data:
                return data
            rospy.sleep(0.1)
        return {"ok": False, "state": "align_timeout", "request_id": request_id}

    def request_reading(self, cycle_id, wp, tasks, align):
        request_id = str(uuid.uuid4())
        req = {
            "request_id": request_id,
            "cycle_id": cycle_id,
            "waypoint_id": wp["id"],
            "tasks": tasks,
            "stamp": now_sec(),
            "image_topic": self.image_topic,
            "detection_topic": "/meter/detection",
            "alignment": align,
        }
        with self.lock:
            self.reading_results.pop(request_id, None)
        self.publish_status("reading", "requesting reading at waypoint %s" % wp["id"], req)
        for _ in range(3):
            self.reading_request_pub.publish(String(json.dumps(req, ensure_ascii=False)))
            rospy.sleep(0.1)
        deadline = time.time() + float(wp.get("reading_timeout", self.reading_timeout))
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "request_id": request_id}
            with self.lock:
                data = self.reading_results.get(request_id)
            if data:
                data.setdefault("ok", True)
                return data
            rospy.sleep(0.1)
        return {"ok": False, "state": "reading_timeout", "request_id": request_id}

    def evaluate_readings(self, cycle_id, wp, reading):
        evaluation = {
            "pressure_state": None,
            "water_state": None,
            "fast_triggered": False,
            "details": [],
        }
        for item in reading.get("readings", []) or []:
            rtype = str(item.get("type") or item.get("class_name") or "").lower()
            value = item.get("value")
            try:
                value = float(value)
            except Exception:
                continue
            if rtype in ("pressure", "pressure_gauge"):
                state = self.evaluate_pressure(value)
                evaluation["pressure_state"] = state["state"]
                evaluation["fast_triggered"] = evaluation["fast_triggered"] or state["fast_triggered"]
                evaluation["details"].append({"type": "pressure_gauge", "value": value, "rule": state})
            elif rtype in ("water", "water_meter"):
                state = self.evaluate_water(wp, item, value)
                evaluation["water_state"] = state["state"]
                evaluation["fast_triggered"] = evaluation["fast_triggered"] or state["fast_triggered"]
                evaluation["details"].append({"type": "water_meter", "value": value, "rule": state})
        return evaluation

    def evaluate_pressure(self, value):
        near = (self.pressure_low <= value < self.pressure_low + self.pressure_near_margin) or (
            self.pressure_high - self.pressure_near_margin < value <= self.pressure_high
        )
        if 0.0 <= value < self.pressure_low:
            return {"state": "abnormal_a", "fast_triggered": False, "reason": "pressure_low"}
        if self.pressure_high < value <= 10.0:
            return {"state": "abnormal_b", "fast_triggered": False, "reason": "pressure_high"}
        if self.pressure_low <= value <= self.pressure_high:
            return {"state": "near_limit" if near else "normal", "fast_triggered": near, "reason": "near_limit" if near else "normal"}
        return {"state": "out_of_range", "fast_triggered": False, "reason": "outside_0_10"}

    def evaluate_water(self, wp, item, value):
        meter_id = str(item.get("meter_id") or "%s_water_meter" % wp["id"])
        old = self.history.get("water", {}).get(meter_id)
        self.history.setdefault("water", {})[meter_id] = {
            "value": value,
            "waypoint_id": wp["id"],
            "stamp": now_sec(),
        }
        if old is None:
            return {"state": "first_record", "fast_triggered": False, "meter_id": meter_id}
        delta = abs(value - float(old.get("value", 0.0)))
        return {
            "state": "abnormal_delta" if delta > self.water_delta_threshold else "normal",
            "fast_triggered": delta > self.water_delta_threshold,
            "meter_id": meter_id,
            "last_value": old.get("value"),
            "delta": delta,
            "threshold": self.water_delta_threshold,
        }

    def run_pressure_a_scan(self, cycle_id, wp):
        self.publish_status("pressure_a_scan", "executing pressure A lateral scan", {
            "cycle_id": cycle_id,
            "waypoint_id": wp["id"],
        })
        os.makedirs(self.capture_dir, exist_ok=True)
        captures = []
        for name, distance in (("left_0p6", 0.6), ("right_1p2", -1.2), ("left_0p6_return", 0.6)):
            captures.extend(self.move_lateral_with_captures(cycle_id, wp, name, distance))
        self.stop_robot()
        result = {"ok": True, "capture_count": len(captures), "captures": captures}
        self.publish_status("pressure_a_scan_complete", "pressure A scan complete", result)
        return result

    def move_lateral_with_captures(self, cycle_id, wp, phase, distance):
        speed = self.scan_lateral_speed
        duration = abs(distance) / max(0.01, speed)
        sign = 1.0 if distance >= 0 else -1.0
        if not self.scan_left_positive:
            sign *= -1.0
        started = time.time()
        next_capture = 0.0
        captures = []
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() - started < duration:
            if self.cancel_requested:
                break
            cmd = Twist()
            cmd.linear.y = sign * speed
            self.publish_cmd(cmd)
            elapsed = time.time() - started
            if elapsed >= next_capture:
                captures.append(self.capture_scan_image(cycle_id, wp, phase))
                next_capture += self.scan_capture_interval
            rate.sleep()
        self.stop_robot()
        return captures

    def capture_scan_image(self, cycle_id, wp, phase):
        with self.lock:
            img = self.latest_image
        stamp = now_sec()
        safe_wp = str(wp["id"]).replace("/", "_")
        filename = "%s_%s_%s_%03d.jpg" % (cycle_id, safe_wp, phase, int(stamp * 1000) % 100000)
        path = os.path.join(self.capture_dir, filename)
        size = 0
        if img is not None:
            try:
                with open(path, "wb") as f:
                    f.write(bytes(img.data))
                size = len(img.data)
            except Exception as exc:
                path = ""
                rospy.logwarn("failed to save scan capture: %s", exc)
        payload = {
            "cycle_id": cycle_id,
            "waypoint_id": wp["id"],
            "phase": phase,
            "stamp": stamp,
            "image_topic": self.image_topic,
            "image_path": path,
            "byte_size": size,
        }
        self.scan_capture_pub.publish(String(json.dumps(payload, ensure_ascii=False)))
        return payload

    def publish_cmd(self, cmd):
        if not self.dry_run:
            self.cmd_pub.publish(cmd)

    def stop_robot(self):
        z = Twist()
        for _ in range(8):
            self.cmd_pub.publish(z)
            time.sleep(0.025)

    def sleep_interruptible(self, seconds):
        end = time.time() + max(0.0, seconds)
        while not rospy.is_shutdown() and not self.cancel_requested and time.time() < end:
            rospy.sleep(min(0.5, end - time.time()))

    def load_history(self):
        try:
            with open(self.history_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except Exception:
            return {}

    def save_history(self):
        try:
            directory = os.path.dirname(self.history_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            tmp = self.history_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self.history, f, ensure_ascii=False, indent=2)
            os.replace(tmp, self.history_file)
        except Exception as exc:
            rospy.logwarn("failed to save inspection history: %s", exc)

    def publish_status(self, state, message, extra):
        payload = {"stamp": now_sec(), "state": state, "message": message, "fast_mode": self.fast_mode, "extra": extra}
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))

    def publish_frequency(self, mode, interval_sec, reason):
        payload = {"stamp": now_sec(), "mode": mode, "interval_sec": interval_sec, "reason": reason}
        self.frequency_pub.publish(String(json.dumps(payload, ensure_ascii=False)))

    def publish_alert(self, alert_type, cycle_id, wp, detail):
        payload = {
            "stamp": now_sec(),
            "alert_type": alert_type,
            "cycle_id": cycle_id,
            "waypoint_id": wp["id"],
            "detail": detail,
        }
        self.alert_pub.publish(String(json.dumps(payload, ensure_ascii=False)))


def main():
    rospy.init_node("inspection_cycle_manager")
    InspectionCycleManager()
    rospy.spin()


if __name__ == "__main__":
    main()
