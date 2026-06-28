#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Abnormal-meter triggered 1m pipe patrol with odom-tagged API analysis."""

import json
import math
import threading
import time

import requests
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class PipePatrolController:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/front/image/compressed")
        self.api_base = rospy.get_param("~api_base", "http://127.0.0.1:8000").rstrip("/")
        self.turn_angle_deg = float(rospy.get_param("~turn_angle_deg", 90.0))
        self.distance_m = float(rospy.get_param("~distance_m", 1.0))
        self.forward_speed = float(rospy.get_param("~forward_speed", 0.15))
        self.capture_interval = float(rospy.get_param("~capture_interval", 0.5))
        self.api_timeout = float(rospy.get_param("~api_timeout", 30.0))
        self.normal_min = float(rospy.get_param("~normal_min", 4.0))
        self.normal_max = float(rospy.get_param("~normal_max", 6.0))
        self.auto_on_meter_abnormal = bool(rospy.get_param("~auto_on_meter_abnormal", False))

        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_odom = None
        self.busy = False
        self.last_meter_stamp = 0.0

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.status_pub = rospy.Publisher("/pipe_patrol/status", String, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher("/pipe_patrol/result", String, queue_size=20, latch=True)
        self.command_response_pub = rospy.Publisher("/eggy/command/response", String, queue_size=20, latch=True)

        rospy.Subscriber(self.image_topic, CompressedImage, self.on_image, queue_size=1, buff_size=2**24)
        rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=20)
        rospy.Subscriber("/pipe_patrol/request", String, self.on_request, queue_size=5)
        rospy.Subscriber("/kimi_inspection/result", String, self.on_kimi_result, queue_size=10)

        self.publish_status("ready", "pipe patrol controller ready")
        rospy.loginfo("pipe_patrol_controller ready: image=%s auto_on_meter_abnormal=%s",
                      self.image_topic, self.auto_on_meter_abnormal)

    def on_image(self, msg):
        with self.lock:
            self.latest_image = msg

    def on_odom(self, msg):
        with self.lock:
            self.latest_odom = msg

    def on_request(self, msg):
        text = (msg.data or "").strip()
        if text:
            try:
                payload = json.loads(text)
                if str(payload.get("command", "start")).lower() in ("stop", "cancel"):
                    self.stop_robot()
                    self.publish_status("stopped", "stop requested")
                    return
            except Exception:
                pass
        self.start_patrol("manual")

    def on_kimi_result(self, msg):
        if not self.auto_on_meter_abnormal:
            return
        try:
            payload = json.loads(msg.data)
            stamp = float(payload.get("stamp", 0.0))
            if stamp <= self.last_meter_stamp:
                return
            self.last_meter_stamp = stamp
            api = payload.get("api") or {}
            result = api.get("result") or {}
            reading = result.get("reading")
            if not self.is_low_pressure_reading(reading):
                return
            self.start_patrol("meter_low_pressure", {"meter_result": payload})
        except Exception as exc:
            rospy.logwarn("failed to parse kimi result: %s", exc)

    def is_low_pressure_reading(self, reading):
        try:
            value = float(str(reading).strip())
        except Exception:
            return False
        return value < self.normal_min

    def start_patrol(self, trigger, extra=None):
        with self.lock:
            if self.busy:
                self.publish_status("busy", "patrol already running")
                return
            if self.latest_image is None or self.latest_odom is None:
                self.publish_status("not_ready", "missing camera frame or odom")
                return
            self.busy = True
        thread = threading.Thread(target=self.run_patrol, args=(trigger, extra or {}), daemon=True)
        thread.start()

    def pose_snapshot(self):
        with self.lock:
            odom = self.latest_odom
        p = odom.pose.pose.position
        yaw = yaw_from_quat(odom.pose.pose.orientation)
        return p.x, p.y, yaw

    def image_snapshot(self):
        with self.lock:
            image = self.latest_image
            odom = self.latest_odom
        p = odom.pose.pose.position
        return {
            "stamp": rospy.Time.now().to_sec(),
            "camera_stamp": image.header.stamp.to_sec(),
            "image": bytes(image.data),
            "odom_x": p.x,
            "odom_y": p.y,
        }

    def run_patrol(self, trigger, extra):
        started = rospy.Time.now().to_sec()
        captures = []
        try:
            self.publish_status("turning", "turning left %.1f deg" % self.turn_angle_deg)
            self.rotate_by_odom(math.radians(self.turn_angle_deg))

            self.publish_status("patrolling", "moving %.2fm and capturing every %.2fs" %
                                (self.distance_m, self.capture_interval))
            captures = self.forward_and_capture()
            self.stop_robot()
            self.publish_status("analyzing", "analyzing %d pipe frames" % len(captures))

            summary = self.analyze_captures(captures, trigger, started, extra)
            self.result_pub.publish(String(json.dumps(summary, ensure_ascii=False)))
            self.publish_command_response(summary)
            self.publish_status("idle", "pipe patrol complete")
        except Exception as exc:
            self.stop_robot()
            result = {
                "ok": False,
                "trigger": trigger,
                "stamp": rospy.Time.now().to_sec(),
                "elapsed_sec": round(rospy.Time.now().to_sec() - started, 3),
                "captures": len(captures),
                "error": str(exc),
            }
            self.result_pub.publish(String(json.dumps(result, ensure_ascii=False)))
            self.publish_command_response(result)
            self.publish_status("error", str(exc))
        finally:
            with self.lock:
                self.busy = False

    def rotate_by_odom(self, delta_yaw):
        _, _, start_yaw = self.pose_snapshot()
        target = start_yaw + delta_yaw
        rate = rospy.Rate(20)
        deadline = time.time() + 10.0
        while not rospy.is_shutdown() and time.time() < deadline:
            _, _, yaw = self.pose_snapshot()
            err = angle_diff(target, yaw)
            if abs(err) < math.radians(2.0):
                break
            cmd = Twist()
            cmd.angular.z = max(-0.6, min(0.6, 1.6 * err))
            if abs(cmd.angular.z) < 0.16:
                cmd.angular.z = 0.16 if err > 0 else -0.16
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.stop_robot()
        rospy.sleep(0.25)

    def forward_and_capture(self):
        start_x, start_y, _ = self.pose_snapshot()
        captures = []
        last_capture = 0.0
        rate = rospy.Rate(20)
        deadline = time.time() + max(8.0, self.distance_m / max(0.05, self.forward_speed) + 4.0)
        while not rospy.is_shutdown() and time.time() < deadline:
            x, y, _ = self.pose_snapshot()
            dist = math.hypot(x - start_x, y - start_y)
            now = time.time()
            if now - last_capture >= self.capture_interval:
                snap = self.image_snapshot()
                snap["distance_m"] = round(dist, 3)
                snap["index"] = len(captures)
                captures.append(snap)
                last_capture = now
            if dist >= self.distance_m:
                break
            cmd = Twist()
            cmd.linear.x = self.forward_speed
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.stop_robot()
        return captures

    def analyze_captures(self, captures, trigger, started, extra):
        frame_results = []
        abnormal = []
        for item in captures:
            files = {
                "image": (
                    "pipe_%03d.jpg" % item["index"],
                    item["image"],
                    "image/jpeg",
                )
            }
            frame = {
                "index": item["index"],
                "distance_m": item["distance_m"],
                "camera_stamp": item["camera_stamp"],
            }
            try:
                resp = requests.post(self.api_base + "/analyze_pipe", files=files, timeout=self.api_timeout)
                resp.raise_for_status()
                api = resp.json()
                frame["api"] = api
                result = api.get("result") or {}
                if result.get("status") == "abnormal" or result.get("has_abnormal") is True:
                    abnormal.append({
                        "index": item["index"],
                        "distance_m": item["distance_m"],
                        "defect_type": result.get("defect_type", "unknown"),
                        "severity": result.get("severity", "unknown"),
                        "confidence": result.get("confidence", 0.0),
                        "reason": result.get("reason", ""),
                        "suggestion": result.get("suggestion", ""),
                    })
            except Exception as exc:
                frame["api"] = {"ok": False, "error": str(exc)}
            frame_results.append(frame)

        return {
            "ok": True,
            "trigger": trigger,
            "stamp": rospy.Time.now().to_sec(),
            "elapsed_sec": round(rospy.Time.now().to_sec() - started, 3),
            "distance_m": self.distance_m,
            "capture_interval": self.capture_interval,
            "capture_count": len(captures),
            "abnormal_count": len(abnormal),
            "abnormal_locations": abnormal,
            "frames": frame_results,
            "extra": extra,
        }

    def stop_robot(self):
        z = Twist()
        for _ in range(8):
            self.cmd_pub.publish(z)
            time.sleep(0.03)

    def publish_status(self, state, message):
        payload = {
            "stamp": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
            "state": state,
            "message": message,
            "image_topic": self.image_topic,
        }
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))

    def publish_command_response(self, result):
        payload = {
            "request_id": "pipe_patrol",
            "command": "pipe_patrol",
            "target": "pipe",
            "success": bool(result.get("ok")),
            "message": "Pipe patrol complete" if result.get("ok") else "Pipe patrol failed",
            "details": result,
            "stamp": rospy.Time.now().to_sec(),
        }
        self.command_response_pub.publish(String(json.dumps(payload, ensure_ascii=False)))


def main():
    rospy.init_node("pipe_patrol_controller")
    PipePatrolController()
    rospy.spin()


if __name__ == "__main__":
    main()
