#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run an inspection route and call target alignment at every waypoint."""

import json
import math
import threading
import time
import uuid

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
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


class InspectionRouteRunner:
    def __init__(self):
        self.route_file = rospy.get_param("~route_file", "")
        self.default_frame = rospy.get_param("~default_frame", "map")
        self.move_base_timeout = float(rospy.get_param("~move_base_timeout", 120.0))
        self.align_timeout = float(rospy.get_param("~align_timeout", 60.0))
        self.align_target = rospy.get_param("~align_target", "any")

        self.lock = threading.Lock()
        self.busy = False
        self.cancel_requested = False
        self.align_results = {}

        self.status_pub = rospy.Publisher("/inspection_route/status", String, queue_size=10, latch=True)
        self.result_pub = rospy.Publisher("/inspection_route/result", String, queue_size=10, latch=True)
        self.align_request_pub = rospy.Publisher("/inspection_align/request", String, queue_size=10)

        rospy.Subscriber("/inspection_route/request", String, self.on_request, queue_size=5)
        rospy.Subscriber("/inspection_align/result", String, self.on_align_result, queue_size=20)

        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.publish_status("ready", "inspection route runner ready", {})
        rospy.loginfo("inspection_route_runner ready")

    def on_align_result(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        rid = data.get("request_id")
        if rid:
            with self.lock:
                self.align_results[str(rid)] = data

    def on_request(self, msg):
        payload = self.parse_payload(msg.data)
        command = str(payload.get("command", "start")).lower()
        if command in ("stop", "cancel"):
            self.cancel_requested = True
            self.client.cancel_all_goals()
            self.align_request_pub.publish(String(json.dumps({"command": "cancel"}, ensure_ascii=False)))
            self.publish_status("cancelled", "route cancel requested", {})
            return
        with self.lock:
            if self.busy:
                self.publish_status("busy", "route already running", {})
                return
            self.busy = True
            self.cancel_requested = False
        threading.Thread(target=self.run_route, args=(payload,), daemon=True).start()

    def parse_payload(self, text):
        text = (text or "").strip()
        if not text:
            return {"route": self.load_route_file()}
        try:
            data = json.loads(text)
            if isinstance(data, dict):
                if "route" not in data and "waypoints" not in data:
                    data["route"] = self.load_route_file()
                return data
        except Exception:
            pass
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
            item.setdefault("target", payload.get("target", self.align_target))
            item["x"] = float(item["x"])
            item["y"] = float(item["y"])
            item["yaw"] = float(item.get("yaw", 0.0))
            out.append(item)
        return out

    def run_route(self, payload):
        started = time.time()
        waypoints = []
        results = []
        ok = False
        try:
            waypoints = self.normalize_waypoints(payload)
            if not waypoints:
                raise RuntimeError("empty route; publish JSON with route:[{id,x,y,yaw,target}] or set ~route_file")
            self.publish_status("waiting_move_base", "waiting for move_base", {})
            if not self.client.wait_for_server(rospy.Duration(10.0)):
                raise RuntimeError("move_base action server not available")
            for index, wp in enumerate(waypoints):
                if self.cancel_requested or rospy.is_shutdown():
                    raise RuntimeError("cancelled")
                self.publish_status("navigating", "going to waypoint %s" % wp["id"], {"index": index, "waypoint": wp})
                nav = self.navigate_to(wp)
                if not nav.get("ok"):
                    results.append({"waypoint": wp, "navigation": nav, "alignment": None})
                    if bool(payload.get("stop_on_nav_fail", True)):
                        raise RuntimeError("navigation failed at %s: %s" % (wp["id"], nav.get("state_text")))
                    continue
                self.publish_status("aligning", "aligning at waypoint %s" % wp["id"], {"index": index, "waypoint": wp})
                align = self.call_aligner(wp)
                results.append({"waypoint": wp, "navigation": nav, "alignment": align})
                if not align.get("ok") and bool(payload.get("stop_on_align_fail", False)):
                    raise RuntimeError("alignment failed at %s: %s" % (wp["id"], align.get("state")))
            ok = True
            state = "complete"
            message = "inspection route complete"
        except Exception as exc:
            state = "error"
            message = str(exc)
        finally:
            result = {
                "ok": ok,
                "state": state,
                "message": message,
                "stamp": rospy.Time.now().to_sec(),
                "elapsed_sec": round(time.time() - started, 3),
                "waypoint_count": len(waypoints),
                "results": results,
                "handoff_for_teammate": {
                    "wait_for_topic": "/inspection_align/ready_for_reading",
                    "per_waypoint_alignment_result": "/inspection_align/result",
                    "image_topic": "/camera/front/image/compressed",
                    "detection_topic": "/meter/detection",
                },
            }
            self.result_pub.publish(String(json.dumps(result, ensure_ascii=False)))
            self.publish_status(state, message, result)
            with self.lock:
                self.busy = False

    def navigate_to(self, wp):
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = wp.get("frame_id", self.default_frame)
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["x"]
        goal.target_pose.pose.position.y = wp["y"]
        goal.target_pose.pose.orientation = quat_from_yaw(wp.get("yaw", 0.0))
        self.client.send_goal(goal)
        done = self.client.wait_for_result(rospy.Duration(float(wp.get("nav_timeout", self.move_base_timeout))))
        if not done:
            self.client.cancel_goal()
            return {"ok": False, "state": -1, "state_text": "TIMEOUT"}
        state = self.client.get_state()
        return {"ok": state == GoalStatus.SUCCEEDED, "state": int(state), "state_text": GOAL_STATUS_TEXT.get(state, str(state))}

    def call_aligner(self, wp):
        request_id = str(uuid.uuid4())
        req = {
            "request_id": request_id,
            "waypoint_id": wp["id"],
            "target": wp.get("target", self.align_target),
            "search_timeout": float(wp.get("search_timeout", 20.0)),
            "align_timeout": float(wp.get("align_timeout", 25.0)),
        }
        if "desired_bbox_height_px" in wp:
            req["desired_bbox_height_px"] = float(wp["desired_bbox_height_px"])
        with self.lock:
            self.align_results.pop(request_id, None)
        # Publish several times so the request is not lost if subscriber connects slowly.
        for _ in range(3):
            self.align_request_pub.publish(String(json.dumps(req, ensure_ascii=False)))
            rospy.sleep(0.1)
        deadline = time.time() + self.align_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "request_id": request_id}
            with self.lock:
                result = self.align_results.get(request_id)
            if result:
                return result
            rospy.sleep(0.1)
        return {"ok": False, "state": "align_timeout", "request_id": request_id}

    def publish_status(self, state, message, extra):
        payload = {
            "stamp": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
            "state": state,
            "message": message,
            "extra": extra,
        }
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))


def main():
    rospy.init_node("inspection_route_runner")
    InspectionRouteRunner()
    rospy.spin()


if __name__ == "__main__":
    main()
