#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Automatically read detected meter/gauge targets through the Kimi inspection API."""

import json
import threading
import time

import requests
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class AutoMeterKimiReader:
    def __init__(self):
        self.detection_topic = rospy.get_param("~detection_topic", "/meter/detection")
        self.image_topic = rospy.get_param("~image_topic", "/camera/front/image_source/compressed")
        self.api_base = rospy.get_param("~api_base", "http://127.0.0.1:8000").rstrip("/")
        self.api_timeout = float(rospy.get_param("~api_timeout", 60.0))
        self.min_score = float(rospy.get_param("~min_score", 0.45))
        self.stable_seconds = float(rospy.get_param("~stable_seconds", 0.8))
        self.cooldown_seconds = float(rospy.get_param("~cooldown_seconds", 8.0))
        self.center_tolerance_px = float(rospy.get_param("~center_tolerance_px", 35.0))
        self.area_tolerance = float(rospy.get_param("~area_tolerance", 0.20))
        self.stop_before_read = bool(rospy.get_param("~stop_before_read", True))
        self.settle_seconds = float(rospy.get_param("~settle_seconds", 0.7))

        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_image_stamp = None
        self.last_target = None
        self.stable_since = None
        self.last_read_time = 0.0
        self.busy = False
        self.armed = True
        self.current_state = "starting"
        self.current_message = "node starting"

        self.result_pub = rospy.Publisher(
            "/inspection_reading/result", String, queue_size=10, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/inspection_reading/status", String, queue_size=10, latch=True
        )
        self.kimi_result_pub = rospy.Publisher(
            "/kimi_inspection/result", String, queue_size=10, latch=True
        )
        self.command_response_pub = rospy.Publisher(
            "/eggy/command/response", String, queue_size=10, latch=True
        )
        self.cmd_pub = rospy.Publisher(
            rospy.get_param("~cmd_vel_topic", "/cmd_vel/mission"), Twist, queue_size=1)

        rospy.Subscriber(
            self.image_topic, CompressedImage, self.on_image, queue_size=1, buff_size=2**24
        )
        rospy.Subscriber(self.detection_topic, String, self.on_detection, queue_size=10)

        self.publish_status("ready", "auto meter reader started")
        threading.Thread(target=self.status_loop, daemon=True).start()
        rospy.loginfo(
            "inspection_reading_auto_node ready: detection=%s image=%s api=%s",
            self.detection_topic,
            self.image_topic,
            self.api_base,
        )

    def on_image(self, msg):
        with self.lock:
            self.latest_image = bytes(msg.data)
            self.latest_image_stamp = msg.header.stamp.to_sec()

    def on_detection(self, msg):
        target = self.best_meter_targets(msg.data)
        now = time.time()
        if target is None:
            with self.lock:
                self.last_target = None
                self.stable_since = None
                self.armed = True
            return

        should_start = False
        with self.lock:
            if self.latest_image is None:
                self.publish_status("waiting_image", "detection received but no source image yet")
                return
            if not self.armed:
                return
            if self.busy or now - self.last_read_time < self.cooldown_seconds:
                return

            if self.is_same_target(self.last_target, target):
                if self.stable_since is None:
                    self.stable_since = now
            else:
                self.last_target = target
                self.stable_since = now
                self.publish_status("target_seen", "meter target detected, waiting for stable box")

            if self.stable_since is not None and now - self.stable_since >= self.stable_seconds:
                self.busy = True
                self.armed = False
                should_start = True
                image_bytes = self.latest_image
                image_stamp = self.latest_image_stamp

        if should_start:
            thread = threading.Thread(
                target=self.run_reading, args=(target, image_bytes, image_stamp), daemon=True
            )
            thread.start()

    def status_loop(self):
        while not rospy.is_shutdown():
            self.publish_status(self.current_state, self.current_message)
            time.sleep(2.0)

    def best_meter_targets(self, data):
        detections = self.parse_detections(data)
        by_class = {}
        for det in detections:
            name = str(det.get("class_name") or det.get("name") or "").strip()
            score = float(det.get("score") or det.get("confidence") or 0.0)
            if name in ("water_meter", "pressure_gauge") and score >= self.min_score:
                bbox = self.read_bbox(det)
                if bbox:
                    det = dict(det)
                    det["bbox"] = bbox
                    det["score"] = score
                    previous = by_class.get(name)
                    if previous is None or score > previous.get("score", 0.0):
                        by_class[name] = det
        if not by_class:
            return None
        detections = [by_class[k] for k in ("water_meter", "pressure_gauge") if k in by_class]
        primary = max(detections, key=lambda d: d.get("score", 0.0))
        return {
            "classes": list(by_class.keys()),
            "primary": primary,
            "detections": detections,
        }

    def parse_detections(self, data):
        try:
            payload = json.loads(data)
        except Exception:
            return []
        if isinstance(payload, list):
            return [x for x in payload if isinstance(x, dict)]
        if isinstance(payload, dict):
            for key in ("detections", "objects", "boxes", "result"):
                value = payload.get(key)
                if isinstance(value, list):
                    return [x for x in value if isinstance(x, dict)]
            return [payload]
        return []

    def read_bbox(self, det):
        try:
            if isinstance(det.get("bbox"), dict):
                box = det["bbox"]
                return [float(box[k]) for k in ("x1", "y1", "x2", "y2")]
            if isinstance(det.get("bbox"), (list, tuple)) and len(det["bbox"]) >= 4:
                return [float(x) for x in det["bbox"][:4]]
            return [float(det[k]) for k in ("x1", "y1", "x2", "y2")]
        except Exception:
            return None

    def is_same_target(self, previous, current):
        if not previous:
            return False
        p = previous["primary"]["bbox"]
        c = current["primary"]["bbox"]
        pcx, pcy = (p[0] + p[2]) * 0.5, (p[1] + p[3]) * 0.5
        ccx, ccy = (c[0] + c[2]) * 0.5, (c[1] + c[3]) * 0.5
        p_area = max(1.0, (p[2] - p[0]) * (p[3] - p[1]))
        c_area = max(1.0, (c[2] - c[0]) * (c[3] - c[1]))
        center_delta = ((pcx - ccx) ** 2 + (pcy - ccy) ** 2) ** 0.5
        area_delta = abs(c_area - p_area) / p_area
        return center_delta <= self.center_tolerance_px and area_delta <= self.area_tolerance

    def run_reading(self, target, image_bytes, image_stamp):
        started = time.time()
        try:
            self.publish_status("settling", "meter target stable, stopping before API reading")
            if self.stop_before_read:
                self.publish_stop(self.settle_seconds)

            self.publish_status("running", "posting full camera image to Kimi")

            files = {"image": ("water_meter_full.jpg", image_bytes, "image/jpeg")}
            response = requests.post(
                self.api_base + "/analyze_meter", files=files, timeout=self.api_timeout
            )
            response.raise_for_status()
            api_result = response.json()

            result = {
                "ok": True,
                "type": "inspection_reading",
                "target": "meter_reading",
                "detected_targets": target.get("classes", []),
                "stamp": rospy.Time.now().to_sec(),
                "camera_stamp": image_stamp,
                "elapsed_sec": round(time.time() - started, 3),
                "source": {
                    "detection_topic": self.detection_topic,
                    "image_topic": self.image_topic,
                    "api_endpoint": "/analyze_meter",
                },
                "detection": target,
                "image_mode": "full_frame",
                "api": api_result,
                "readings": self.extract_api_field(api_result, "readings") or {},
                "analysis": self.extract_api_field(api_result, "analysis") or {},
                "water_meter": self.extract_reading(api_result, "water_meter"),
                "pressure_gauge": self.extract_reading(api_result, "pressure_gauge"),
                "reading": self.legacy_reading(api_result),
                "status": self.legacy_status(api_result),
                "confidence": self.legacy_confidence(api_result),
                "reason": self.extract_api_field(api_result, "summary") or self.legacy_reason(api_result),
            }
            self.publish_result(result)
            self.publish_status("idle", "reading complete")
        except Exception as exc:
            result = {
                "ok": False,
                "type": "inspection_reading",
                "target": "meter_reading",
                "stamp": rospy.Time.now().to_sec(),
                "elapsed_sec": round(time.time() - started, 3),
                "error": str(exc),
                "detection": target,
            }
            self.publish_result(result)
            self.publish_status("error", str(exc))
        finally:
            with self.lock:
                self.busy = False
                self.last_read_time = time.time()
                self.stable_since = None
                self.last_target = None

    def extract_api_field(self, api_result, key):
        if not isinstance(api_result, dict):
            return None
        result = api_result.get("result")
        if isinstance(result, dict) and key in result:
            return result.get(key)
        return api_result.get(key)

    def extract_reading(self, api_result, target):
        readings = self.extract_api_field(api_result, "readings")
        if isinstance(readings, dict) and isinstance(readings.get(target), dict):
            return readings.get(target)
        return None

    def legacy_reading(self, api_result):
        water = self.extract_reading(api_result, "water_meter")
        if isinstance(water, dict):
            return water.get("reading") or water.get("best_effort_reading")
        return self.extract_api_field(api_result, "reading")

    def legacy_status(self, api_result):
        water = self.extract_reading(api_result, "water_meter")
        pressure = self.extract_reading(api_result, "pressure_gauge")
        for item in (water, pressure):
            if isinstance(item, dict) and item.get("present"):
                return item.get("status")
        return self.extract_api_field(api_result, "status")

    def legacy_confidence(self, api_result):
        values = []
        for target in ("water_meter", "pressure_gauge"):
            item = self.extract_reading(api_result, target)
            if isinstance(item, dict) and item.get("present"):
                try:
                    values.append(float(item.get("confidence")))
                except Exception:
                    pass
        if values:
            return min(values)
        return self.extract_api_field(api_result, "confidence")

    def legacy_reason(self, api_result):
        reasons = []
        for target in ("water_meter", "pressure_gauge"):
            item = self.extract_reading(api_result, target)
            if isinstance(item, dict) and item.get("reason"):
                reasons.append("%s: %s" % (target, item.get("reason")))
        if reasons:
            return "; ".join(reasons)
        return self.extract_api_field(api_result, "reason")

    def publish_stop(self, seconds):
        end = time.time() + max(0.0, seconds)
        msg = Twist()
        while not rospy.is_shutdown() and time.time() < end:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_pub.publish(msg)

    def publish_result(self, result):
        text = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(String(text))
        self.kimi_result_pub.publish(String(text))
        command = {
            "request_id": "inspection_reading_auto",
            "command": "inspection_reading",
            "target": "meter_reading",
            "success": bool(result.get("ok")),
            "message": "meter reading complete" if result.get("ok") else "meter reading failed",
            "details": result,
            "stamp": rospy.Time.now().to_sec(),
        }
        self.command_response_pub.publish(String(json.dumps(command, ensure_ascii=False)))

    def publish_status(self, state, message):
        self.current_state = state
        self.current_message = message
        payload = {
            "stamp": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
            "state": state,
            "message": message,
            "detection_topic": self.detection_topic,
            "image_topic": self.image_topic,
            "api_base": self.api_base,
        }
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))


def main():
    rospy.init_node("inspection_reading_auto_node")
    AutoMeterKimiReader()
    rospy.spin()


if __name__ == "__main__":
    main()
