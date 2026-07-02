#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bridge ROS compressed camera frames to the Kimi inspection HTTP API."""

import json
import threading
import time

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

            files = {
                "image": (
                    "camera.jpg",
                    bytes(msg.data),
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
