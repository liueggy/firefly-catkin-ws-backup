#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simple point navigator: rotate toward goal, then drive straight with heading correction."""
import math
import os
import sys
import threading
import time

for p in ("/opt/ros/noetic/lib/python3/dist-packages", "/root/catkin_ws/devel/lib/python3/dist-packages"):
    if p not in sys.path:
        sys.path.insert(0, p)

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

state_lock = threading.RLock()
odom = None
scan = None
goal = None
cancelled = False
nav_state = "idle"

MAX_VX = float(os.environ.get("EGGY_SIMPLE_NAV_MAX_VX", "0.25"))
MAX_WZ = float(os.environ.get("EGGY_SIMPLE_NAV_MAX_WZ", "0.60"))
GOAL_TOL = float(os.environ.get("EGGY_SIMPLE_NAV_GOAL_TOL", "0.12"))
YAW_TOL = float(os.environ.get("EGGY_SIMPLE_NAV_YAW_TOL", "0.087"))  # 5 deg
FRONT_STOP = float(os.environ.get("EGGY_SIMPLE_NAV_FRONT_STOP", "0.35"))
KP_YAW = float(os.environ.get("EGGY_SIMPLE_NAV_KP_YAW", "1.2"))
KP_DIST = float(os.environ.get("EGGY_SIMPLE_NAV_KP_DIST", "0.45"))
KP_HEADING = float(os.environ.get("EGGY_SIMPLE_NAV_KP_HEADING", "0.8"))


def norm_ang(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_from_q(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def front_distance():
    with state_lock:
        s = scan
    if not s:
        return None
    vals = []
    for i, r in enumerate(s.ranges):
        if not math.isfinite(r) or r < max(s.range_min, 0.15):
            continue
        ang = s.angle_min + i * s.angle_increment
        if abs(math.degrees(ang)) <= 15:
            vals.append(r)
    return min(vals) if vals else None


def set_state(pub, st, extra=""):
    global nav_state
    nav_state = st
    msg = st if not extra else f"{st}: {extra}"
    pub.publish(String(data=msg))
    rospy.loginfo("simple_nav %s", msg)


def odom_cb(msg):
    global odom
    with state_lock:
        odom = msg


def scan_cb(msg):
    global scan
    with state_lock:
        scan = msg


def goal_cb(msg):
    global goal, cancelled
    with state_lock:
        goal = msg
        cancelled = False
    rospy.loginfo("simple_nav goal %.3f %.3f", msg.pose.position.x, msg.pose.position.y)


def cancel_cb(msg):
    global cancelled
    with state_lock:
        cancelled = True


def stop(pub):
    z = Twist()
    for _ in range(6):
        pub.publish(z)
        rospy.sleep(0.05)


def main():
    global goal, cancelled
    rospy.init_node("eggy_simple_navigator")
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    state_pub = rospy.Publisher("/simple_nav/status", String, queue_size=10, latch=True)
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=5)
    rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=3)
    rospy.Subscriber("/simple_goal", PoseStamped, goal_cb, queue_size=1)
    rospy.Subscriber("/simple_nav/cancel", String, cancel_cb, queue_size=1)

    rate = rospy.Rate(20)
    set_state(state_pub, "idle")
    last_status_pub = time.time()
    while not rospy.is_shutdown():
        if time.time() - last_status_pub > 1.0:
            state_pub.publish(String(data=nav_state))
            last_status_pub = time.time()
        with state_lock:
            g = goal
            o = odom
            is_cancelled = cancelled
        if not g or not o:
            rate.sleep(); continue
        if is_cancelled:
            stop(cmd_pub)
            with state_lock:
                goal = None
                cancelled = False
            set_state(state_pub, "cancelled")
            rate.sleep(); continue

        x = o.pose.pose.position.x
        y = o.pose.pose.position.y
        yaw = yaw_from_q(o.pose.pose.orientation)
        gx = g.pose.position.x
        gy = g.pose.position.y
        dx, dy = gx-x, gy-y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = norm_ang(target_yaw - yaw)
        fd = front_distance()

        tw = Twist()
        if dist <= GOAL_TOL:
            stop(cmd_pub)
            with state_lock:
                goal = None
            set_state(state_pub, "arrived", f"dist={dist:.2f}")
            rate.sleep(); continue

        if fd is not None and fd < FRONT_STOP:
            stop(cmd_pub)
            set_state(state_pub, "blocked", f"front={fd:.2f} dist={dist:.2f}")
            rate.sleep(); continue

        if abs(yaw_err) > YAW_TOL:
            tw.angular.z = clamp(KP_YAW * yaw_err, -MAX_WZ, MAX_WZ)
            set_state(state_pub, "rotating", f"yaw_err={math.degrees(yaw_err):.1f} dist={dist:.2f}")
        else:
            tw.linear.x = clamp(KP_DIST * dist, 0.05, MAX_VX)
            tw.angular.z = clamp(KP_HEADING * yaw_err, -0.25, 0.25)
            set_state(state_pub, "driving", f"dist={dist:.2f} yaw_err={math.degrees(yaw_err):.1f}")
        cmd_pub.publish(tw)
        rate.sleep()


if __name__ == "__main__":
    main()
