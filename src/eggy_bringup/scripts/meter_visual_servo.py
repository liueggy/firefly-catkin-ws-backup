#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Visual servo for keeping meter/gauge detection box at a calibrated size.

This node is intentionally standalone. It does not modify existing navigation
nodes and only publishes /cmd_vel while it is running and enabled.
"""

import json
import math
import os
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def clamp(value, low, high):
    return max(low, min(high, value))


class MeterVisualServo:
    def __init__(self):
        self.detection_topic = rospy.get_param("~detection_topic", "/meter/detection")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.target_file = rospy.get_param(
            "~target_file",
            "/root/catkin_ws/src/eggy_bringup/config/meter_visual_servo_target.json",
        )
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.require_scan = bool(rospy.get_param("~require_scan", True))
        self.target_class = str(rospy.get_param("~target_class", "any"))
        self.min_score = float(rospy.get_param("~min_score", 0.65))

        self.area_tolerance = float(rospy.get_param("~area_tolerance", 0.15))
        self.area_unlock_tolerance = float(rospy.get_param("~area_unlock_tolerance", 0.22))
        self.near_area_error = float(rospy.get_param("~near_area_error", 0.40))
        self.max_linear = float(rospy.get_param("~max_linear", 0.26))
        self.min_linear = float(rospy.get_param("~min_linear", 0.065))
        self.min_linear_near = float(rospy.get_param("~min_linear_near", 0.04))
        self.linear_gain = float(rospy.get_param("~linear_gain", 0.42))
        self.max_angular = float(rospy.get_param("~max_angular", 0.42))
        self.center_kp = float(rospy.get_param("~center_kp", 0.55))
        self.center_deadband = float(rospy.get_param("~center_deadband", 0.06))
        self.search_wz = float(rospy.get_param("~search_wz", 0.45))
        self.search_timeout = float(rospy.get_param("~search_timeout", 1.2))
        self.initial_search_wz = float(rospy.get_param("~initial_search_wz", 0.35))
        self.initial_search_timeout = float(rospy.get_param("~initial_search_timeout", 12.0))
        self.cmd_rate = float(rospy.get_param("~cmd_rate", 25.0))
        self.detection_timeout = float(rospy.get_param("~detection_timeout", 0.55))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 1.0))
        self.obstacle_stop = float(rospy.get_param("~obstacle_stop", 0.35))
        self.obstacle_slow = float(rospy.get_param("~obstacle_slow", 0.55))
        self.braking_time = float(rospy.get_param("~braking_time", 0.45))
        self.side_stop = float(rospy.get_param("~side_stop", 0.24))
        self.front_sector_deg = float(rospy.get_param("~front_sector_deg", 28.0))
        self.rear_sector_deg = float(rospy.get_param("~rear_sector_deg", 28.0))
        self.edge_lost_margin = float(rospy.get_param("~edge_lost_margin", 0.04))

        self.targets = self.load_targets()
        self.last_detection = None
        self.last_detection_time = 0.0
        self.last_detection_msg_time = 0.0
        self.last_target_visible = False
        self.last_center_error = 0.0
        self.distance_locked = False
        self.scan = None
        self.scan_time = 0.0
        self.last_state = ""
        self.search_started_time = time.time() if self.enabled else 0.0

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.status_pub = rospy.Publisher("/meter_visual_servo/status", String, queue_size=1, latch=True)
        rospy.Subscriber(self.detection_topic, String, self.on_detection, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/meter_visual_servo/request", String, self.on_request, queue_size=5)

        rospy.loginfo("meter_visual_servo ready enabled=%s target_file=%s targets=%s",
                      self.enabled, self.target_file, self.targets)
        self.publish_status("ready", "node ready")

    def load_targets(self):
        default = {
            "pressure_gauge": 0.0375,
            "water_meter": 0.1477,
            "default": 0.08,
        }
        if not os.path.exists(self.target_file):
            return default
        try:
            with open(self.target_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            targets = data.get("target_area_ratio") or data
            for key, value in default.items():
                targets.setdefault(key, value)
            return targets
        except Exception as exc:
            rospy.logwarn("failed to load target file %s: %s", self.target_file, exc)
            return default

    def on_request(self, msg):
        text = (msg.data or "").strip().lower()
        if text.startswith("{"):
            try:
                request = json.loads(msg.data)
                command = str(request.get("command", "start")).lower()
                target_class = str(request.get("target_class", "any")).lower()
                if target_class in ("water_meter", "pressure_gauge", "any"):
                    self.target_class = target_class
                text = command
            except Exception as exc:
                self.publish_status("bad_request", str(exc))
                return
        if text in ("start", "enable", "on"):
            self.enabled = True
            self.distance_locked = False
            self.last_detection = None
            self.last_detection_time = 0.0
            self.last_detection_msg_time = 0.0
            self.last_target_visible = False
            self.last_center_error = 0.0
            self.search_started_time = time.time()
            self.publish_status("enabled", "servo enabled")
        elif text in ("stop", "disable", "off"):
            self.enabled = False
            self.distance_locked = False
            self.search_started_time = 0.0
            self.stop_robot()
            self.publish_status("disabled", "servo disabled")
        elif text in ("reload", "reload_target"):
            self.targets = self.load_targets()
            self.distance_locked = False
            self.publish_status("reloaded", "target file reloaded")

    def on_detection(self, msg):
        try:
            now = time.time()
            self.last_detection_msg_time = now
            payload = json.loads(msg.data)
            detections = payload.get("detections") or []
            image_width = float(payload.get("image_width") or 0)
            image_height = float(payload.get("image_height") or 0)
            if not detections or image_width <= 0 or image_height <= 0:
                self.last_target_visible = False
                return

            valid = []
            for det in detections:
                cls = str(det.get("class_name", ""))
                if cls not in ("pressure_gauge", "water_meter"):
                    continue
                if self.target_class not in ("", "any", cls):
                    continue
                if float(det.get("score", 0.0)) < self.min_score:
                    continue
                x1, y1 = float(det["x1"]), float(det["y1"])
                x2, y2 = float(det["x2"]), float(det["y2"])
                w, h = max(0.0, x2 - x1), max(0.0, y2 - y1)
                if w <= 1 or h <= 1:
                    continue
                area_ratio = (w * h) / (image_width * image_height)
                cx = (x1 + x2) * 0.5
                center_error = (cx - image_width * 0.5) / (image_width * 0.5)
                edge_touch = (
                    x1 <= image_width * self.edge_lost_margin
                    or x2 >= image_width * (1.0 - self.edge_lost_margin)
                    or y1 <= image_height * self.edge_lost_margin
                    or y2 >= image_height * (1.0 - self.edge_lost_margin)
                )
                valid.append({
                    "class_name": cls,
                    "score": float(det.get("score", 0.0)),
                    "area_ratio": area_ratio,
                    "center_error": center_error,
                    "edge_touch": edge_touch,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                })
            if not valid:
                self.last_target_visible = False
                return
            group_x1 = min(d["x1"] for d in valid)
            group_y1 = min(d["y1"] for d in valid)
            group_x2 = max(d["x2"] for d in valid)
            group_y2 = max(d["y2"] for d in valid)
            margin_x = image_width * self.edge_lost_margin
            margin_y = image_height * self.edge_lost_margin
            group_left_edge = group_x1 <= margin_x
            group_right_edge = group_x2 >= image_width - margin_x
            group_edge_touch = (
                group_left_edge
                or group_right_edge
                or group_y1 <= margin_y
                or group_y2 >= image_height - margin_y
            )
            group_cx = (group_x1 + group_x2) * 0.5
            group_center_error = (group_cx - image_width * 0.5) / (image_width * 0.5)
            if group_left_edge and not group_right_edge:
                group_center_error = min(group_center_error, -self.center_deadband * 1.5)
            elif group_right_edge and not group_left_edge:
                group_center_error = max(group_center_error, self.center_deadband * 1.5)
            # Prefer the largest visible meter/gauge; use score only as a tie-breaker.
            best = sorted(valid, key=lambda d: (d["area_ratio"], d["score"]), reverse=True)[0]
            best["group_count"] = len(valid)
            best["group_edge_touch"] = group_edge_touch
            best["group_horizontal_edge_touch"] = group_left_edge or group_right_edge
            best["group_center_error"] = group_center_error
            self.last_detection = best
            self.last_detection_time = now
            self.last_target_visible = True
            self.last_center_error = best["group_center_error"]
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "bad detection message: %s", exc)

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = time.time()

    def sector_min(self, center_deg, half_width_deg):
        if self.scan is None:
            return None
        ranges = self.scan.ranges
        if not ranges:
            return None
        amin = self.scan.angle_min
        inc = self.scan.angle_increment
        best = None
        center = math.radians(center_deg)
        half = math.radians(half_width_deg)
        for i, r in enumerate(ranges):
            if not math.isfinite(r):
                continue
            angle = amin + i * inc
            delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(delta) <= half and self.scan.range_min <= r <= self.scan.range_max:
                best = r if best is None else min(best, r)
        return best

    def scan_fresh(self):
        return self.scan is not None and time.time() - self.scan_time <= self.scan_timeout

    def safe_linear(self, linear_x):
        if abs(linear_x) < 1e-5:
            return 0.0, "clear"
        if self.require_scan and not self.scan_fresh():
            return 0.0, "waiting_scan"
        if linear_x > 0:
            dist = self.sector_min(0.0, self.front_sector_deg)
            direction = "front"
        else:
            dist = self.sector_min(180.0, self.rear_sector_deg)
            direction = "rear"
        if dist is None:
            return (0.0, "no_%s_scan" % direction) if self.require_scan else (linear_x * 0.5, "no_scan_slow")
        dynamic_stop = self.obstacle_stop + abs(linear_x) * self.braking_time
        if dist < dynamic_stop:
            return 0.0, "%s_obstacle_stop_%.2f" % (direction, dist)
        if dist < self.obstacle_slow:
            return linear_x * 0.45, "%s_obstacle_slow_%.2f" % (direction, dist)
        if linear_x > 0:
            left = self.sector_min(70.0, 22.0)
            right = self.sector_min(-70.0, 22.0)
            side_values = [value for value in (left, right) if value is not None]
            if side_values and min(side_values) < self.side_stop:
                return 0.0, "side_obstacle_stop_%.2f" % min(side_values)
        return linear_x, "clear_%.2f" % dist

    def compute_cmd(self):
        now = time.time()
        if not self.enabled:
            return Twist(), "disabled", {}

        fresh_empty_frame = (
            self.last_detection is not None
            and not self.last_target_visible
            and now - self.last_detection_msg_time <= self.detection_timeout
        )
        stale_target = self.last_detection is None or now - self.last_detection_time > self.detection_timeout
        if fresh_empty_frame or stale_target:
            self.distance_locked = False
            cmd = Twist()
            target_age = now - self.last_detection_time if self.last_detection_time else 999.0
            search_age = now - self.search_started_time if self.search_started_time else 999.0
            if self.last_detection is None and search_age <= self.initial_search_timeout:
                cmd.angular.z = self.initial_search_wz
                return cmd, "searching_initial_target", {
                    "search_age": round(search_age, 3),
                    "search_timeout": round(self.initial_search_timeout, 3),
                    "target_class": self.target_class,
                }
            if abs(self.last_center_error) > 0.05 and target_age <= self.search_timeout:
                cmd.angular.z = self.search_wz if self.last_center_error < 0 else -self.search_wz
                return cmd, "searching_lost_target", {
                    "last_center_error": round(self.last_center_error, 4),
                    "target_age": round(target_age, 3),
                    "reason": "empty_frame" if fresh_empty_frame else "timeout",
                }
            return cmd, "no_target", {"target_age": round(target_age, 3)}

        det = self.last_detection
        target = float(self.targets.get(det["class_name"], self.targets.get("default", 0.08)))
        area = det["area_ratio"]
        area_error = (target - area) / max(target, 1e-6)

        group_edge_touch = bool(det.get("group_edge_touch", det["edge_touch"]))
        group_horizontal_edge_touch = bool(det.get("group_horizontal_edge_touch", det["edge_touch"]))
        group_center_error = float(det.get("group_center_error", det["center_error"]))

        if self.distance_locked:
            if abs(area_error) <= self.area_unlock_tolerance and not group_edge_touch:
                return Twist(), "target_size_hold", {
                    "class_name": det["class_name"],
                    "score": round(float(det.get("score", 0.0)), 3),
                    "area_ratio": round(area, 5),
                    "target_area_ratio": round(target, 5),
                    "area_error": round(area_error, 4),
                    "unlock_tolerance": round(self.area_unlock_tolerance, 4),
                    "group_count": int(det.get("group_count", 1)),
                }
            self.distance_locked = False

        if abs(area_error) <= self.area_tolerance and not group_edge_touch:
            self.distance_locked = True
            return Twist(), "target_size_reached", {
                "class_name": det["class_name"],
                "score": round(float(det.get("score", 0.0)), 3),
                "area_ratio": round(area, 5),
                "target_area_ratio": round(target, 5),
                "center_error": round(det["center_error"], 4),
                "group_count": int(det.get("group_count", 1)),
                "group_center_error": round(group_center_error, 4),
            }

        linear = clamp(self.linear_gain * area_error, -self.max_linear, self.max_linear)
        min_linear = self.min_linear_near if abs(area_error) <= self.near_area_error else self.min_linear
        if abs(linear) < min_linear:
            linear = min_linear if area_error > 0 else -min_linear
        linear, safety = self.safe_linear(linear)

        cmd = Twist()
        cmd.linear.x = linear
        if group_horizontal_edge_touch and abs(group_center_error) > self.center_deadband:
            cmd.angular.z = clamp(-self.center_kp * group_center_error, -self.max_angular, self.max_angular)
        state = "servo_forward" if linear > 0 else "servo_backward" if linear < 0 else "blocked"
        return cmd, state, {
            "class_name": det["class_name"],
            "score": round(float(det.get("score", 0.0)), 3),
            "target_class": self.target_class,
            "area_ratio": round(area, 5),
            "target_area_ratio": round(target, 5),
            "area_error": round(area_error, 4),
            "distance_locked": self.distance_locked,
            "min_linear": round(min_linear, 4),
            "center_error": round(det["center_error"], 4),
            "group_count": int(det.get("group_count", 1)),
            "group_center_error": round(group_center_error, 4),
            "group_edge_touch": group_edge_touch,
            "angular_z": round(cmd.angular.z, 4),
            "safety": safety,
        }

    def stop_robot(self):
        z = Twist()
        for _ in range(6):
            self.cmd_pub.publish(z)
            time.sleep(0.02)

    def publish_status(self, state, message, extra=None):
        payload = {
            "stamp": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
            "state": state,
            "message": message,
            "enabled": self.enabled,
        }
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))

    def spin(self):
        rate = rospy.Rate(self.cmd_rate)
        while not rospy.is_shutdown():
            cmd, state, extra = self.compute_cmd()
            self.cmd_pub.publish(cmd)
            if state != self.last_state:
                self.publish_status(state, state, extra)
                self.last_state = state
            rate.sleep()
        self.stop_robot()


def main():
    rospy.init_node("meter_visual_servo")
    MeterVisualServo().spin()


if __name__ == "__main__":
    main()
