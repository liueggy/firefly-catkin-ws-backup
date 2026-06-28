#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Align the robot to a detected pressure gauge / water meter at an inspection point.

Request topic: /inspection_align/request std_msgs/String JSON
Result topic:  /inspection_align/result  std_msgs/String JSON

This node deliberately uses the existing /meter/detection JSON output so the
RKNN/C++ detector remains the single source of vision truth.
"""

import json
import math
import threading
import time
import uuid

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


CLASS_ALIASES = {
    "pressure": "pressure_gauge",
    "gauge": "pressure_gauge",
    "pressure_gauge": "pressure_gauge",
    "water": "water_meter",
    "meter": "water_meter",
    "water_meter": "water_meter",
}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class InspectionTargetAligner:
    def __init__(self):
        self.detection_topic = rospy.get_param("~detection_topic", "/meter/detection")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.center_tolerance_px = float(rospy.get_param("~center_tolerance_px", 35.0))
        self.size_tolerance_ratio = float(rospy.get_param("~size_tolerance_ratio", 0.18))
        self.search_timeout = float(rospy.get_param("~search_timeout", 20.0))
        self.align_timeout = float(rospy.get_param("~align_timeout", 25.0))
        self.search_angular_speed = float(rospy.get_param("~search_angular_speed", 0.22))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.28))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.08))
        self.angular_kp = float(rospy.get_param("~angular_kp", 0.55))
        self.linear_kp = float(rospy.get_param("~linear_kp", 0.18))
        self.min_score = float(rospy.get_param("~min_score", 0.45))
        self.detection_max_age = float(rospy.get_param("~detection_max_age", 1.2))
        self.stable_frames_required = int(rospy.get_param("~stable_frames_required", 5))
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        # These are camera/image-size heuristics for the "about 20cm" stop point.
        # Tune them after one real measurement at 20cm.
        self.desired_heights = {
            "pressure_gauge": float(rospy.get_param("~pressure_gauge_height_px_at_20cm", 115.0)),
            "water_meter": float(rospy.get_param("~water_meter_height_px_at_20cm", 130.0)),
            "both": float(rospy.get_param("~both_height_px_at_20cm", 130.0)),
        }

        self.lock = threading.Lock()
        self.latest_detection = None
        self.latest_detection_time = 0.0
        self.busy = False
        self.cancel_requested = False

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.status_pub = rospy.Publisher("/inspection_align/status", String, queue_size=10, latch=True)
        self.result_pub = rospy.Publisher("/inspection_align/result", String, queue_size=10, latch=True)
        self.ready_pub = rospy.Publisher("/inspection_align/ready_for_reading", String, queue_size=10, latch=True)

        rospy.Subscriber(self.detection_topic, String, self.on_detection, queue_size=10)
        rospy.Subscriber("/inspection_align/request", String, self.on_request, queue_size=5)

        self.publish_status("ready", "inspection target aligner ready", {})
        rospy.loginfo("inspection_target_aligner ready: detection=%s dry_run=%s", self.detection_topic, self.dry_run)

    def on_detection(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.latest_detection = data
            self.latest_detection_time = time.time()

    def on_request(self, msg):
        payload = self.parse_request(msg.data)
        cmd = str(payload.get("command", "start")).lower()
        if cmd in ("stop", "cancel"):
            self.cancel_requested = True
            self.stop_robot()
            self.publish_status("cancelled", "cancel requested", payload)
            return
        with self.lock:
            if self.busy:
                self.publish_status("busy", "aligner is already running", payload)
                return
            self.busy = True
            self.cancel_requested = False
        thread = threading.Thread(target=self.run_alignment, args=(payload,), daemon=True)
        thread.start()

    def parse_request(self, text):
        text = (text or "").strip()
        if not text:
            return {}
        try:
            payload = json.loads(text)
            return payload if isinstance(payload, dict) else {}
        except Exception:
            return {"target": text}

    def normalized_targets(self, target):
        target = str(target or "any").strip().lower()
        if target in ("any", "auto", "*"):
            return {"pressure_gauge", "water_meter"}, False
        if target in ("both", "all"):
            return {"pressure_gauge", "water_meter"}, True
        if target in CLASS_ALIASES:
            return {CLASS_ALIASES[target]}, False
        return {"pressure_gauge", "water_meter"}, False

    def detection_snapshot(self):
        with self.lock:
            det = self.latest_detection
            age = time.time() - self.latest_detection_time if self.latest_detection_time else 999.0
        return det, age

    def filtered_detections(self, det, targets):
        if not det:
            return []
        out = []
        for item in det.get("detections", []) or []:
            name = str(item.get("class_name", ""))
            score = float(item.get("score", 0.0))
            if name in targets and score >= self.min_score:
                out.append(item)
        return out

    def select_target(self, det, targets, require_both):
        matches = self.filtered_detections(det, targets)
        if not matches:
            return None, matches
        if require_both:
            names = {m.get("class_name") for m in matches}
            if not targets.issubset(names):
                return None, matches
            x1 = min(float(m["x1"]) for m in matches)
            y1 = min(float(m["y1"]) for m in matches)
            x2 = max(float(m["x2"]) for m in matches)
            y2 = max(float(m["y2"]) for m in matches)
            score = min(float(m.get("score", 0.0)) for m in matches)
            return {
                "class_id": -1,
                "class_name": "both",
                "score": score,
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
            }, matches
        width = float(det.get("image_width", 640) or 640)
        cx0 = width * 0.5
        matches.sort(key=lambda m: (float(m.get("score", 0.0)) - 0.15 * abs(((float(m["x1"]) + float(m["x2"])) * 0.5 - cx0) / max(1.0, cx0))), reverse=True)
        return matches[0], matches

    def run_alignment(self, payload):
        started = time.time()
        request_id = str(payload.get("request_id") or uuid.uuid4())
        waypoint_id = str(payload.get("waypoint_id") or payload.get("id") or "")
        targets, require_both = self.normalized_targets(payload.get("target", "any"))
        search_timeout = float(payload.get("search_timeout", self.search_timeout))
        align_timeout = float(payload.get("align_timeout", self.align_timeout))
        desired_override = payload.get("desired_bbox_height_px")
        stable = 0
        last_target = None
        last_matches = []
        result = None
        try:
            self.stop_robot()
            self.publish_status("searching", "searching target", {
                "request_id": request_id,
                "waypoint_id": waypoint_id,
                "targets": sorted(targets),
                "require_both": require_both,
            })

            search_deadline = time.time() + search_timeout
            rate = rospy.Rate(12)
            while not rospy.is_shutdown() and time.time() < search_deadline:
                if self.cancel_requested:
                    raise RuntimeError("cancelled")
                det, age = self.detection_snapshot()
                target, matches = self.select_target(det, targets, require_both) if age <= self.detection_max_age else (None, [])
                if target:
                    last_target = target
                    last_matches = matches
                    break
                cmd = Twist()
                cmd.angular.z = self.search_angular_speed
                self.publish_cmd(cmd)
                rate.sleep()
            self.stop_robot()

            if last_target is None:
                result = self.make_result(False, request_id, waypoint_id, "not_found", started, None, [], None)
                return

            self.publish_status("aligning", "target found, aligning", {
                "request_id": request_id,
                "waypoint_id": waypoint_id,
                "target": last_target,
            })

            align_deadline = time.time() + align_timeout
            while not rospy.is_shutdown() and time.time() < align_deadline:
                if self.cancel_requested:
                    raise RuntimeError("cancelled")
                det, age = self.detection_snapshot()
                target, matches = self.select_target(det, targets, require_both) if age <= self.detection_max_age else (None, [])
                if target is None:
                    stable = 0
                    cmd = Twist()
                    cmd.angular.z = self.search_angular_speed * 0.7
                    self.publish_cmd(cmd)
                    rate.sleep()
                    continue

                last_target = target
                last_matches = matches
                metrics = self.compute_metrics(det, target, desired_override)
                cmd = Twist()
                if abs(metrics["center_error_px"]) > self.center_tolerance_px:
                    norm_err = metrics["center_error_px"] / max(1.0, metrics["image_width"] * 0.5)
                    cmd.angular.z = clamp(-self.angular_kp * norm_err, -self.max_angular_speed, self.max_angular_speed)
                elif abs(metrics["size_error_ratio"]) > self.size_tolerance_ratio:
                    cmd.linear.x = clamp(self.linear_kp * metrics["size_error_ratio"], -self.max_linear_speed, self.max_linear_speed)
                self.publish_cmd(cmd)

                if abs(metrics["center_error_px"]) <= self.center_tolerance_px and abs(metrics["size_error_ratio"]) <= self.size_tolerance_ratio:
                    stable += 1
                else:
                    stable = 0
                if stable >= self.stable_frames_required:
                    self.stop_robot()
                    result = self.make_result(True, request_id, waypoint_id, "aligned", started, target, matches, metrics)
                    return
                rate.sleep()

            metrics = self.compute_metrics(self.detection_snapshot()[0], last_target, desired_override) if last_target else None
            result = self.make_result(False, request_id, waypoint_id, "timeout", started, last_target, last_matches, metrics)
        except Exception as exc:
            self.stop_robot()
            result = self.make_result(False, request_id, waypoint_id, str(exc), started, last_target, last_matches, None)
        finally:
            self.stop_robot()
            if result is not None:
                self.publish_result(result)
            with self.lock:
                self.busy = False

    def compute_metrics(self, det, target, desired_override=None):
        width = float((det or {}).get("image_width", 640) or 640)
        height = float((det or {}).get("image_height", 480) or 480)
        x1 = float(target["x1"])
        y1 = float(target["y1"])
        x2 = float(target["x2"])
        y2 = float(target["y2"])
        box_h = max(1.0, y2 - y1)
        box_w = max(1.0, x2 - x1)
        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        desired = float(desired_override) if desired_override is not None else self.desired_heights.get(str(target.get("class_name")), self.desired_heights["both"])
        return {
            "image_width": width,
            "image_height": height,
            "bbox_width_px": box_w,
            "bbox_height_px": box_h,
            "bbox_center_x": cx,
            "bbox_center_y": cy,
            "center_error_px": cx - width * 0.5,
            "desired_bbox_height_px": desired,
            "size_error_ratio": (desired - box_h) / max(1.0, desired),
            "distance_mode": "bbox_height_proxy_for_20cm",
        }

    def make_result(self, ok, request_id, waypoint_id, state, started, target, matches, metrics):
        payload = {
            "ok": bool(ok),
            "state": state,
            "stamp": rospy.Time.now().to_sec(),
            "elapsed_sec": round(time.time() - started, 3),
            "request_id": request_id,
            "waypoint_id": waypoint_id,
            "target": target,
            "all_visible_targets": matches or [],
            "metrics": metrics,
            "handoff": {
                "ready_for_reading": bool(ok),
                "recommended_next_action": "capture_and_read_meter" if ok else "manual_check_or_retry",
                "topics_for_teammate": {
                    "image": "/camera/front/image/compressed",
                    "detection_json": "/meter/detection",
                    "align_result": "/inspection_align/result",
                    "ready_event": "/inspection_align/ready_for_reading",
                },
            },
        }
        return payload

    def publish_cmd(self, cmd):
        if not self.dry_run:
            self.cmd_pub.publish(cmd)

    def stop_robot(self):
        z = Twist()
        for _ in range(8):
            self.cmd_pub.publish(z)
            time.sleep(0.025)

    def publish_status(self, state, message, extra):
        payload = {
            "stamp": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
            "state": state,
            "message": message,
            "dry_run": self.dry_run,
            "extra": extra,
        }
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))

    def publish_result(self, result):
        text = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(String(text))
        if result.get("ok"):
            self.ready_pub.publish(String(text))
        self.publish_status(result.get("state", "done"), "alignment finished", result)


def main():
    rospy.init_node("inspection_target_aligner")
    InspectionTargetAligner()
    rospy.spin()


if __name__ == "__main__":
    main()
