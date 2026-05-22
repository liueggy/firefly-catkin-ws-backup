#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Smooth omni point navigator for Eggy.

Receives /simple_goal (PoseStamped) in map/odom, uses TF to compute robot pose in
that frame, then drives holonomically toward the goal with acceleration limiting.
This avoids rotate-then-drive pauses and makes map-click navigation feel direct.
"""
import math
import os
import sys
import threading
import time

for p in ("/opt/ros/noetic/lib/python3/dist-packages", "/root/catkin_ws/devel/lib/python3/dist-packages"):
    if p not in sys.path:
        sys.path.insert(0, p)

import rospy
import tf
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
last_twist = Twist()

MAX_VX = float(os.environ.get("EGGY_SIMPLE_NAV_MAX_VX", "0.65"))
MAX_VY = float(os.environ.get("EGGY_SIMPLE_NAV_MAX_VY", "0.55"))
MAX_WZ = float(os.environ.get("EGGY_SIMPLE_NAV_MAX_WZ", "0.90"))
MAX_TRANS = float(os.environ.get("EGGY_SIMPLE_NAV_MAX_TRANS", "0.75"))
ACC_XY = float(os.environ.get("EGGY_SIMPLE_NAV_ACC_XY", "0.90"))
ACC_WZ = float(os.environ.get("EGGY_SIMPLE_NAV_ACC_WZ", "1.60"))
GOAL_TOL = float(os.environ.get("EGGY_SIMPLE_NAV_GOAL_TOL", "0.12"))
SLOW_RADIUS = float(os.environ.get("EGGY_SIMPLE_NAV_SLOW_RADIUS", "0.65"))
FRONT_STOP = float(os.environ.get("EGGY_SIMPLE_NAV_FRONT_STOP", "0.24"))
KP_DIST = float(os.environ.get("EGGY_SIMPLE_NAV_KP_DIST", "1.15"))
KP_YAW = float(os.environ.get("EGGY_SIMPLE_NAV_KP_YAW", "1.20"))
KEEP_FACE_TARGET = os.environ.get("EGGY_SIMPLE_NAV_FACE_TARGET", "1") not in ("0", "false", "False")


def norm_ang(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_from_q(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def yaw_from_quat_tuple(q):
    return tf.transformations.euler_from_quaternion(q)[2]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ramp(prev, target, max_delta):
    return prev + clamp(target - prev, -max_delta, max_delta)


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
        if abs(math.degrees(ang)) <= 18:
            vals.append(r)
    return min(vals) if vals else None


def set_state(pub, st, extra=""):
    global nav_state
    nav_state = st
    msg = st if not extra else f"{st}: {extra}"
    pub.publish(String(data=msg))


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
    if not msg.header.frame_id:
        msg.header.frame_id = "map"
    with state_lock:
        goal = msg
        cancelled = False
    rospy.loginfo("simple_nav goal frame=%s %.3f %.3f", msg.header.frame_id, msg.pose.position.x, msg.pose.position.y)


def cancel_cb(msg):
    global cancelled
    with state_lock:
        cancelled = True


def publish_stop(pub):
    global last_twist
    z = Twist()
    last_twist = z
    for _ in range(4):
        pub.publish(z)
        rospy.sleep(0.04)


def pose_in_frame(listener, frame):
    try:
        listener.waitForTransform(frame, "base_link", rospy.Time(0), rospy.Duration(0.15))
        trans, rot = listener.lookupTransform(frame, "base_link", rospy.Time(0))
        return trans[0], trans[1], yaw_from_quat_tuple(rot)
    except Exception:
        with state_lock:
            o = odom
        if not o:
            raise
        return o.pose.pose.position.x, o.pose.pose.position.y, yaw_from_q(o.pose.pose.orientation)


def main():
    global goal, cancelled, last_twist
    rospy.init_node("eggy_simple_navigator")
    listener = tf.TransformListener()
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    state_pub = rospy.Publisher("/simple_nav/status", String, queue_size=10, latch=True)
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=5)
    rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=3)
    rospy.Subscriber("/simple_goal", PoseStamped, goal_cb, queue_size=1)
    rospy.Subscriber("/simple_nav/cancel", String, cancel_cb, queue_size=1)

    rate_hz = 30.0
    rate = rospy.Rate(rate_hz)
    set_state(state_pub, "idle")
    last_status_pub = time.time()
    while not rospy.is_shutdown():
        if time.time() - last_status_pub > 0.5:
            state_pub.publish(String(data=nav_state))
            last_status_pub = time.time()
        with state_lock:
            g = goal
            is_cancelled = cancelled
        if not g:
            rate.sleep(); continue
        if is_cancelled:
            publish_stop(cmd_pub)
            with state_lock:
                goal = None
                cancelled = False
            set_state(state_pub, "cancelled")
            rate.sleep(); continue

        frame = g.header.frame_id or "map"
        try:
            x, y, yaw = pose_in_frame(listener, frame)
        except Exception as e:
            set_state(state_pub, "waiting_tf", str(e)[:60])
            rate.sleep(); continue

        gx, gy = g.pose.position.x, g.pose.position.y
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist <= GOAL_TOL:
            publish_stop(cmd_pub)
            with state_lock:
                goal = None
            set_state(state_pub, "arrived", f"dist={dist:.2f}")
            rate.sleep(); continue

        fd = front_distance()
        # Only hard-stop for very close front obstacles while commanded forward.
        if fd is not None and fd < FRONT_STOP:
            publish_stop(cmd_pub)
            set_state(state_pub, "blocked", f"front={fd:.2f} dist={dist:.2f}")
            rate.sleep(); continue

        # World/map vector -> base_link velocity for holonomic chassis.
        ux, uy = dx / max(dist, 1e-6), dy / max(dist, 1e-6)
        speed = min(MAX_TRANS, KP_DIST * dist)
        if dist < SLOW_RADIUS:
            speed *= max(0.22, dist / SLOW_RADIUS)
        vx_world, vy_world = speed * ux, speed * uy
        c, s = math.cos(yaw), math.sin(yaw)
        target_vx = c * vx_world + s * vy_world
        target_vy = -s * vx_world + c * vy_world
        target_vx = clamp(target_vx, -MAX_VX, MAX_VX)
        target_vy = clamp(target_vy, -MAX_VY, MAX_VY)

        if KEEP_FACE_TARGET:
            yaw_err = norm_ang(math.atan2(dy, dx) - yaw)
        else:
            goal_yaw = yaw_from_q(g.pose.orientation)
            yaw_err = norm_ang(goal_yaw - yaw)
        target_wz = clamp(KP_YAW * yaw_err, -MAX_WZ, MAX_WZ)

        max_dv = ACC_XY / rate_hz
        max_dw = ACC_WZ / rate_hz
        tw = Twist()
        tw.linear.x = ramp(last_twist.linear.x, target_vx, max_dv)
        tw.linear.y = ramp(last_twist.linear.y, target_vy, max_dv)
        tw.angular.z = ramp(last_twist.angular.z, target_wz, max_dw)
        last_twist = tw
        cmd_pub.publish(tw)
        set_state(state_pub, "driving", f"dist={dist:.2f} vx={tw.linear.x:.2f} vy={tw.linear.y:.2f}")
        rate.sleep()


if __name__ == "__main__":
    main()
