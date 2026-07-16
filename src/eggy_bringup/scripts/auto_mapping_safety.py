#!/usr/bin/env python3
"""Fail-closed directional safety filter for automatic mapping velocity."""

import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String

from eggy_bringup.auto_mapping_core import safe_mapping_twist


class AutoMappingSafety(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.active = False
        self.fault_latched = False
        self.emergency_stop = False
        self.base_stop = False
        self.last_scan = 0.0
        self.last_odom = 0.0
        self.last_raw = 0.0
        self.last_heartbeat = 0.0
        self.raw_twist = Twist()
        self.clearances = {}
        self.last_reason = "inactive"

        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.35))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.60))
        self.raw_timeout = float(rospy.get_param("~raw_timeout", 0.35))
        self.heartbeat_timeout = float(rospy.get_param("~heartbeat_timeout", 1.0))
        self.max_linear = float(rospy.get_param("~max_linear", 0.30))
        self.max_lateral = float(rospy.get_param("~max_lateral", 0.18))
        self.max_angular = float(rospy.get_param("~max_angular", 0.55))
        self.safety_config = {
            "base_clearance": float(rospy.get_param("~base_clearance", 0.18)),
            "latency_sec": float(rospy.get_param("~latency_sec", 0.15)),
            "deceleration": float(rospy.get_param("~deceleration", 0.50)),
            "margin": float(rospy.get_param("~margin", 0.05)),
            "slow_band": float(rospy.get_param("~slow_band", 0.25)),
            "rotation_clearance": float(rospy.get_param("~rotation_clearance", 0.30)),
        }

        self.output_pub = rospy.Publisher("/cmd_vel/mapping", Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/eggy/auto_mapping/safety_status", String, queue_size=1, latch=True)
        rospy.Subscriber("/cmd_vel/mapping_raw", Twist, self.raw_cb, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/eggy/auto_mapping/safety_active", Bool, self.active_cb, queue_size=1)
        rospy.Subscriber("/eggy/auto_mapping/safety_config", String, self.config_cb, queue_size=1)
        rospy.Subscriber("/eggy/auto_mapping/heartbeat", Empty, self.heartbeat_cb, queue_size=1)
        rospy.Subscriber("/eggy/emergency_stop", Bool, self.emergency_cb, queue_size=1)
        rospy.Subscriber("/base/flag_stop", Bool, self.base_stop_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)
        rospy.on_shutdown(self.shutdown)
        self.publish_status("inactive")

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def sector_min(scan, center_deg, half_width_deg):
        center = math.radians(center_deg)
        half_width = math.radians(half_width_deg)
        best = None
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value) or value < scan.range_min or value > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(delta) <= half_width:
                best = value if best is None else min(best, value)
        return best

    def scan_cb(self, msg):
        with self.lock:
            self.clearances = {
                "front": self.sector_min(msg, 0.0, 34.0),
                "rear": self.sector_min(msg, 180.0, 34.0),
                "left": self.sector_min(msg, 90.0, 34.0),
                "right": self.sector_min(msg, -90.0, 34.0),
                "rotation": self.sector_min(msg, 0.0, 180.0),
            }
            self.last_scan = time.time()

    def raw_cb(self, msg):
        with self.lock:
            self.raw_twist = msg
            self.last_raw = time.time()

    def odom_cb(self, _msg):
        with self.lock:
            self.last_odom = time.time()

    def heartbeat_cb(self, _msg):
        with self.lock:
            self.last_heartbeat = time.time()

    def config_cb(self, msg):
        try:
            data = json.loads(msg.data)
            max_linear = float(data["max_linear_speed"])
            if not math.isfinite(max_linear):
                return
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self.lock:
            self.max_linear = self.clamp(max_linear, 0.05, 0.30)

    def active_cb(self, msg):
        with self.lock:
            self.active = bool(msg.data)
            if not self.active:
                self.fault_latched = False
                self.raw_twist = Twist()
                self.last_reason = "inactive"
        if not msg.data:
            self.output_pub.publish(Twist())

    def emergency_cb(self, msg):
        with self.lock:
            self.emergency_stop = bool(msg.data)
            if self.emergency_stop:
                self.fault_latched = True

    def base_stop_cb(self, msg):
        with self.lock:
            self.base_stop = bool(msg.data)
            if self.base_stop:
                self.fault_latched = True

    def gate_reason(self, now):
        if not self.active:
            return "inactive"
        if self.fault_latched or self.emergency_stop:
            return "emergency_stop_latched"
        if self.base_stop:
            return "base_stop_latched"
        if now - self.last_heartbeat > self.heartbeat_timeout:
            return "heartbeat_stale"
        if now - self.last_scan > self.scan_timeout:
            return "scan_stale"
        if now - self.last_odom > self.odom_timeout:
            return "odom_stale"
        if now - self.last_raw > self.raw_timeout:
            return "raw_cmd_stale"
        return ""

    def tick(self, _event):
        now = time.time()
        with self.lock:
            active = self.active
            reason = self.gate_reason(now)
            raw = self.raw_twist
            clearances = dict(self.clearances)
        if not active:
            return
        output = Twist()
        if not reason:
            vx = self.clamp(raw.linear.x, -self.max_linear, self.max_linear)
            vy = self.clamp(raw.linear.y, -self.max_lateral, self.max_lateral)
            wz = self.clamp(raw.angular.z, -self.max_angular, self.max_angular)
            vx, vy, wz, reason = safe_mapping_twist(
                vx, vy, wz, clearances, self.safety_config)
            output.linear.x = vx
            output.linear.y = vy
            output.angular.z = wz
        self.output_pub.publish(output)
        if reason != self.last_reason or int(now * 2) != int((now - 0.05) * 2):
            self.last_reason = reason
            self.publish_status(reason, output)

    def publish_status(self, reason, output=None):
        now = time.time()
        output = output or Twist()
        with self.lock:
            payload = {
                "schema_version": 1,
                "stamp": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
                "active": self.active,
                "safe": self.active and reason in ("", "clear") or reason.endswith("_slow"),
                "reason": reason or "clear",
                "fault_latched": self.fault_latched,
                "limits": {"max_linear_speed": self.max_linear},
                "ages": {
                    "scan": None if not self.last_scan else round(now - self.last_scan, 3),
                    "odom": None if not self.last_odom else round(now - self.last_odom, 3),
                    "raw_cmd": None if not self.last_raw else round(now - self.last_raw, 3),
                    "heartbeat": None if not self.last_heartbeat else round(now - self.last_heartbeat, 3),
                },
                "clearances": self.clearances,
                "output": {
                    "vx": round(output.linear.x, 3),
                    "vy": round(output.linear.y, 3),
                    "wz": round(output.angular.z, 3),
                },
            }
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))

    def shutdown(self):
        self.output_pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("eggy_auto_mapping_safety")
    AutoMappingSafety()
    rospy.spin()
