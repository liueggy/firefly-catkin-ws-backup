#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bridge ROS compressed camera frames to the Kimi inspection HTTP API."""

import json
import threading
import time

import cv2
import numpy as np
import requests
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class KimiInspectionBridge:
    def __init__(self):
        self.image_topic = rospy.get_param(
            "~image_topic", "/camera/front/image/compressed"
        )
        self.api_base = rospy.get_param("~api_base", "http://127.0.0.1:8000").rstrip("/")
        self.default_task = rospy.get_param("~default_task", "meter")
        self.timeout = float(rospy.get_param("~timeout", 30.0))
        self.auto_interval = float(rospy.get_param("~auto_interval", 0.0))
        self.max_image_age = float(rospy.get_param("~max_image_age", 1.5))

        self.lock = threading.Lock()
        self.latest_msg = None
        self.busy = False

        self.result_pub = rospy.Publisher(
            "/kimi_inspection/result", String, queue_size=10, latch=True
        )
        self.command_response_pub = rospy.Publisher(
            "/eggy/command/response", String, queue_size=10, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/kimi_inspection/status", String, queue_size=1, latch=True
        )

        rospy.Subscriber(
            self.image_topic, CompressedImage, self.on_image, queue_size=1, buff_size=2**24
        )
        rospy.Subscriber(
            "/kimi_inspection/request", String, self.on_request, queue_size=5
        )

        if self.auto_interval > 0:
            rospy.Timer(rospy.Duration(self.auto_interval), self.on_timer)

        self.publish_status("ready", "bridge started")
        rospy.loginfo(
            "kimi_inspection_bridge ready: image=%s api=%s request=/kimi_inspection/request result=/kimi_inspection/result",
            self.image_topic,
            self.api_base,
        )

    def on_image(self, msg):
        with self.lock:
            self.latest_msg = msg

    def on_request(self, msg):
        payload = self.parse_request(msg.data)
        self.start_analysis(payload, "request")

    def on_timer(self, _event):
        self.start_analysis({"task": self.default_task}, "timer")

    def parse_request(self, data):
        text = (data or "").strip()
        if not text:
            return {"task": self.default_task}
        try:
            payload = json.loads(text)
            if isinstance(payload, dict):
                payload["task"] = str(payload.get("task") or self.default_task).strip()
                return payload
        except Exception:
            pass
        return {"task": text}

    def normalize_task(self, task):
        task = (task or self.default_task).strip().lower()
        if task in ("meter", "water_meter", "analyze_meter"):
            return "meter", "/analyze_meter"
        if task in ("pipe", "pipe_inspection", "analyze_pipe"):
            return "pipe", "/analyze_pipe"
        raise ValueError("unknown task: %s" % task)

    def start_analysis(self, request_payload, trigger):
        with self.lock:
            if self.busy:
                self.publish_status("busy", "analysis already running")
                return
            msg = self.latest_msg
            if msg is None:
                self.publish_status("no_image", "no camera frame received yet")
                return
            stamp = msg.header.stamp.to_sec()
            now = rospy.Time.now().to_sec()
            if stamp > 0.0 and now > 0.0 and now - stamp > self.max_image_age:
                self.publish_status("stale_image", "latest camera frame is too old")
                return
            self.busy = True

        thread = threading.Thread(
            target=self.run_analysis, args=(request_payload, trigger, msg), daemon=True
        )
        thread.start()

    def run_analysis(self, request_payload, trigger, msg):
        started = time.time()
        task = request_payload.get("task", self.default_task)
        request_id = request_payload.get("request_id")
        try:
            task_name, endpoint = self.normalize_task(task)
            url = self.api_base + endpoint
            self.publish_status("running", "posting image to %s" % endpoint)

            image_bytes, roi_meta = self.prepare_image(msg, request_payload.get("roi"))
            files = {
                "image": (
                    "inspection_roi.jpg" if roi_meta.get("applied") else "camera.jpg",
                    image_bytes,
                    "image/jpeg",
                )
            }
            response = requests.post(url, files=files, timeout=self.timeout)
            response.raise_for_status()
            api_result = response.json()

            result = {
                "ok": True,
                "request_id": request_id,
                "task": task_name,
                "trigger": trigger,
                "stamp": rospy.Time.now().to_sec(),
                "camera_stamp": msg.header.stamp.to_sec(),
                "image": roi_meta,
                "elapsed_sec": round(time.time() - started, 3),
                "api": api_result,
                "request": request_payload,
            }
            self.result_pub.publish(String(json.dumps(result, ensure_ascii=False)))
            self.publish_command_response(task_name, True, "Kimi inspection complete", result)
            self.publish_status("idle", "analysis complete")
        except Exception as exc:
            result = {
                "ok": False,
                "request_id": request_id,
                "task": task,
                "trigger": trigger,
                "stamp": rospy.Time.now().to_sec(),
                "elapsed_sec": round(time.time() - started, 3),
                "error": str(exc),
                "request": request_payload,
            }
            self.result_pub.publish(String(json.dumps(result, ensure_ascii=False)))
            self.publish_command_response(task, False, "Kimi inspection failed", result)
            self.publish_status("error", str(exc))
        finally:
            with self.lock:
                self.busy = False

    @staticmethod
    def prepare_image(msg, roi):
        """Crop the detector ROI with padding before uploading to Kimi."""
        fallback = {"applied": False, "reason": "no_valid_roi"}
        if not isinstance(roi, dict):
            return bytes(msg.data), fallback
        try:
            encoded = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                return bytes(msg.data), {"applied": False, "reason": "decode_failed"}
            height, width = image.shape[:2]
            x1 = max(0, min(width - 1, int(round(float(roi["x1"])))))
            y1 = max(0, min(height - 1, int(round(float(roi["y1"])))))
            x2 = max(x1 + 1, min(width, int(round(float(roi["x2"])))))
            y2 = max(y1 + 1, min(height, int(round(float(roi["y2"])))))
            if x2 <= x1 or y2 <= y1:
                return bytes(msg.data), fallback
            cropped = image[y1:y2, x1:x2]
            ok, output = cv2.imencode(
                ".jpg", cropped, [int(cv2.IMWRITE_JPEG_QUALITY), 92]
            )
            if not ok:
                return bytes(msg.data), {"applied": False, "reason": "encode_failed"}
            return bytes(output), {
                "applied": True,
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "width": x2 - x1,
                "height": y2 - y1,
                "source_width": width,
                "source_height": height,
            }
        except (KeyError, TypeError, ValueError, cv2.error) as exc:
            return bytes(msg.data), {"applied": False, "reason": str(exc)}

    def publish_command_response(self, task, success, message, result):
        payload = {
            "request_id": "kimi_inspection",
            "command": "kimi_inspection",
            "target": task,
            "success": bool(success),
            "message": message,
            "details": result,
            "stamp": rospy.Time.now().to_sec(),
        }
        self.command_response_pub.publish(String(json.dumps(payload, ensure_ascii=False)))

    def publish_status(self, state, message):
        payload = {
            "stamp": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
            "state": state,
            "message": message,
            "image_topic": self.image_topic,
            "api_base": self.api_base,
        }
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))


def main():
    rospy.init_node("kimi_inspection_bridge")
    KimiInspectionBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
