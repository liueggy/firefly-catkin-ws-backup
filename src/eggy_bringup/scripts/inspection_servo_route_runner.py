#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Route runner that calls meter_visual_servo.py at each waypoint.

This node is intentionally narrow: it does not change move_base, Qt's existing
single-goal bridge, or meter_visual_servo.py. It only sequences them.
"""

import json
import math
import os
import signal
import subprocess
import threading
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
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


def now_sec():
    return rospy.Time.now().to_sec() if not rospy.is_shutdown() else time.time()


class InspectionServoRouteRunner:
    def __init__(self):
        self.default_frame = rospy.get_param("~default_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.route_file = rospy.get_param("~route_file", "")
        self.move_base_timeout = float(rospy.get_param("~move_base_timeout", 120.0))
        self.current_pose_timeout = float(rospy.get_param("~current_pose_timeout", 3.0))
        self.servo_timeout = float(rospy.get_param("~servo_timeout", 60.0))
        self.servo_stable_sec = float(rospy.get_param("~servo_stable_sec", 0.6))
        self.servo_startup_wait = float(rospy.get_param("~servo_startup_wait", 0.35))
        self.servo_shutdown_timeout = float(rospy.get_param("~servo_shutdown_timeout", 3.0))
        self.stop_on_nav_fail = bool(rospy.get_param("~stop_on_nav_fail", True))
        self.stop_on_servo_fail = bool(rospy.get_param("~stop_on_servo_fail", True))
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        self.lock = threading.Lock()
        self.busy = False
        self.cancel_requested = False
        self.latest_servo_status = {}
        self.servo_proc = None

        self.status_pub = rospy.Publisher("/inspection_servo_route/status", String, queue_size=10, latch=True)
        self.result_pub = rospy.Publisher("/inspection_servo_route/result", String, queue_size=10, latch=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.servo_request_pub = rospy.Publisher("/meter_visual_servo/request", String, queue_size=5)

        rospy.Subscriber("/inspection_servo_route/request", String, self.on_request, queue_size=5)
        rospy.Subscriber("/meter_visual_servo/status", String, self.on_servo_status, queue_size=20)

        self.tf_listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.publish_status("ready", "inspection servo route runner ready", {})
        rospy.loginfo("inspection_servo_route_runner ready dry_run=%s", self.dry_run)

    def on_servo_status(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.latest_servo_status = data

    def on_request(self, msg):
        payload = self.parse_payload(msg.data)
        command = str(payload.get("command", "start")).lower()
        if command in ("stop", "cancel"):
            self.cancel_requested = True
            self.client.cancel_all_goals()
            self.stop_servo_node()
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
                point = {"waypoint": wp, "navigation": nav, "servo": None}
                if not nav.get("ok"):
                    results.append(point)
                    if self.stop_on_nav_fail:
                        raise RuntimeError("navigation failed at %s: %s" % (wp["id"], nav.get("state_text")))
                    continue
                self.stop_robot()
                self.publish_status("servo_starting", "starting visual servo at %s" % wp["id"], {
                    "index": index,
                    "waypoint": wp,
                })
                servo = self.run_visual_servo(wp)
                point["servo"] = servo
                results.append(point)
                self.stop_servo_node()
                self.stop_robot()
                if not servo.get("ok") and self.stop_on_servo_fail:
                    raise RuntimeError("visual servo failed at %s: %s" % (wp["id"], servo.get("state")))
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
            self.stop_servo_node()
            self.stop_robot()
            result = {
                "ok": ok,
                "state": state,
                "message": message,
                "stamp": now_sec(),
                "elapsed_sec": round(time.time() - started, 3),
                "waypoint_count": len(waypoints),
                "home_navigation": home_nav,
                "results": results,
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
        self.client.send_goal(goal)
        done = self.client.wait_for_result(rospy.Duration(float(wp.get("nav_timeout", self.move_base_timeout))))
        if not done:
            self.client.cancel_goal()
            return {"ok": False, "state": -1, "state_text": "TIMEOUT"}
        state = self.client.get_state()
        return {"ok": state == GoalStatus.SUCCEEDED, "state": int(state), "state_text": GOAL_STATUS_TEXT.get(state, str(state))}

    def run_visual_servo(self, wp):
        if self.dry_run:
            rospy.sleep(0.5)
            return {"ok": True, "state": "DRY_RUN", "status": {}}
        self.start_servo_node()
        deadline = time.time() + float(wp.get("servo_timeout", self.servo_timeout))
        hold_started = None
        last_status = {}
        good_states = set(["target_size_reached", "target_size_hold"])
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_requested:
                return {"ok": False, "state": "cancelled", "status": last_status}
            with self.lock:
                last_status = dict(self.latest_servo_status)
            state = str(last_status.get("state", ""))
            if state in good_states:
                if hold_started is None:
                    hold_started = time.time()
                if time.time() - hold_started >= float(wp.get("servo_stable_sec", self.servo_stable_sec)):
                    return {
                        "ok": True,
                        "state": state,
                        "stable_sec": round(time.time() - hold_started, 3),
                        "status": last_status,
                    }
            else:
                hold_started = None
            rospy.sleep(0.1)
        return {"ok": False, "state": "servo_timeout", "status": last_status}

    def start_servo_node(self):
        self.stop_servo_node()
        with self.lock:
            self.latest_servo_status = {}
        env = os.environ.copy()
        self.servo_proc = subprocess.Popen(
            ["rosrun", "eggy_bringup", "meter_visual_servo.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            env=env,
        )
        rospy.sleep(self.servo_startup_wait)
        self.servo_request_pub.publish(String("start"))

    def stop_servo_node(self):
        self.servo_request_pub.publish(String("stop"))
        if self.servo_proc is not None:
            proc = self.servo_proc
            self.servo_proc = None
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    deadline = time.time() + self.servo_shutdown_timeout
                    while proc.poll() is None and time.time() < deadline:
                        time.sleep(0.05)
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception as exc:
                rospy.logwarn("failed to stop meter_visual_servo process: %s", exc)

    def stop_robot(self):
        z = Twist()
        for _ in range(8):
            self.cmd_pub.publish(z)
            time.sleep(0.025)

    def publish_status(self, state, message, extra):
        payload = {
            "stamp": now_sec(),
            "state": state,
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
