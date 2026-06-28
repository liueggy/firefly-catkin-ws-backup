#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Navigation status monitor for Eggy robot.

Publishes a compact JSON status string on /eggy_nav/status so the upper computer
can quickly judge current pose freshness, goal progress and move_base state.
"""
import json
import math
import threading

import rospy
import tf
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

lock = threading.RLock()
last = {}
latest_goal = None
move_base_state = "UNKNOWN"
move_base_state_code = -1

STATUS_TEXT = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST",
}


def now_sec():
    return rospy.Time.now().to_sec()


def stamp(name):
    with lock:
        last[name] = now_sec()


def age(name):
    with lock:
        t = last.get(name)
    if not t:
        return None
    return max(0.0, now_sec() - t)


def q_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def odom_cb(msg):
    stamp("odom")


def scan_cb(msg):
    stamp("scan")


def map_cb(msg):
    stamp("map")


def global_plan_cb(msg):
    stamp("global_plan")


def local_plan_cb(msg):
    stamp("local_plan")


def goal_cb(msg):
    global latest_goal
    if not msg.header.frame_id:
        msg.header.frame_id = "map"
    with lock:
        latest_goal = msg
    stamp("goal")


def status_cb(msg):
    global move_base_state, move_base_state_code
    stamp("move_base_status")
    if msg.status_list:
        st = msg.status_list[-1].status
        with lock:
            move_base_state_code = int(st)
            move_base_state = STATUS_TEXT.get(int(st), str(st))


def lookup_pose(listener):
    # Prefer map pose. Use non-blocking TF query; never let monitor loop hang.
    for frame in ("map", "odom"):
        try:
            if not listener.canTransform(frame, "base_link", rospy.Time(0)):
                continue
            trans, rot = listener.lookupTransform(frame, "base_link", rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return {
                "frame": frame,
                "x": round(float(trans[0]), 4),
                "y": round(float(trans[1]), 4),
                "yaw": round(float(yaw), 4),
                "tf_ok": True,
            }
        except Exception:
            continue
    return {"frame": None, "x": None, "y": None, "yaw": None, "tf_ok": False}


def transform_goal(listener, goal, target_frame):
    if goal is None or not target_frame:
        return None
    try:
        g = goal
        g.header.stamp = rospy.Time(0)
        if g.header.frame_id != target_frame:
            if not listener.canTransform(target_frame, g.header.frame_id, rospy.Time(0)):
                return None
            g = listener.transformPose(target_frame, g)
        return g
    except Exception:
        return goal if goal.header.frame_id == target_frame else None


def main():
    rospy.init_node("eggy_nav_monitor")
    listener = tf.TransformListener()
    pub = rospy.Publisher("/eggy_nav/status", String, queue_size=1, latch=True)

    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=5)
    rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=3)
    rospy.Subscriber("/map", OccupancyGrid, map_cb, queue_size=1)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, global_plan_cb, queue_size=1)
    rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, global_plan_cb, queue_size=1)
    rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, local_plan_cb, queue_size=1)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_cb, queue_size=1)
    rospy.Subscriber("/nav_goal", PoseStamped, goal_cb, queue_size=1)
    rospy.Subscriber("/goal_pose", PoseStamped, goal_cb, queue_size=1)
    rospy.Subscriber("/move_base/status", GoalStatusArray, status_cb, queue_size=5)

    rate = rospy.Rate(float(rospy.get_param("~rate", 5.0)))
    while not rospy.is_shutdown():
        pose = lookup_pose(listener)
        with lock:
            goal = latest_goal
            state = move_base_state
            state_code = move_base_state_code

        goal_info = None
        dist = None
        if goal is not None:
            tg = transform_goal(listener, goal, pose.get("frame"))
            gx = gy = gyaw = None
            gframe = goal.header.frame_id or "map"
            if tg is not None:
                gx = float(tg.pose.position.x)
                gy = float(tg.pose.position.y)
                gyaw = q_to_yaw(tg.pose.orientation)
                gframe = tg.header.frame_id
                if pose.get("x") is not None:
                    dist = math.hypot(gx - float(pose["x"]), gy - float(pose["y"]))
            goal_info = {
                "frame": gframe,
                "x": None if gx is None else round(gx, 4),
                "y": None if gy is None else round(gy, 4),
                "yaw": None if gyaw is None else round(gyaw, 4),
                "age_sec": None if age("goal") is None else round(age("goal"), 3),
            }

        payload = {
            "stamp": round(now_sec(), 3),
            "pose": pose,
            "ages": {
                "odom": None if age("odom") is None else round(age("odom"), 3),
                "scan": None if age("scan") is None else round(age("scan"), 3),
                "map": None if age("map") is None else round(age("map"), 3),
                "global_plan": None if age("global_plan") is None else round(age("global_plan"), 3),
                "local_plan": None if age("local_plan") is None else round(age("local_plan"), 3),
                "move_base_status": None if age("move_base_status") is None else round(age("move_base_status"), 3),
            },
            "health": {
                "odom_ok": age("odom") is not None and age("odom") < 0.5,
                "scan_ok": age("scan") is not None and age("scan") < 0.5,
                "map_ok": age("map") is not None and age("map") < 5.0,
                "tf_ok": bool(pose.get("tf_ok")),
            },
            "move_base": {
                "state": state,
                "state_code": state_code,
                "active": state in ("PENDING", "ACTIVE", "PREEMPTING", "RECALLING"),
            },
            "goal": goal_info,
            "distance_to_goal": None if dist is None else round(dist, 4),
        }
        pub.publish(String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))
        rate.sleep()


if __name__ == "__main__":
    main()
