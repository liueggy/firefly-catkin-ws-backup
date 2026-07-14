#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Car-like point navigator for Eggy.

Receives /simple_goal (PoseStamped) in map/odom, uses TF to compute robot pose in
that frame, then drives in a car-like fashion: rotate to face the target, drive
forward only (no lateral drift), and optionally align to a goal yaw at the end.
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
YAW_TOL = float(os.environ.get("EGGY_SIMPLE_NAV_YAW_TOL", "0.08"))
SLOW_RADIUS = float(os.environ.get("EGGY_SIMPLE_NAV_SLOW_RADIUS", "0.65"))
FRONT_STOP = float(os.environ.get("EGGY_SIMPLE_NAV_FRONT_STOP", "0.24"))
KP_DIST = float(os.environ.get("EGGY_SIMPLE_NAV_KP_DIST", "1.15"))
KP_YAW = float(os.environ.get("EGGY_SIMPLE_NAV_KP_YAW", "1.20"))
KEEP_FACE_TARGET = os.environ.get("EGGY_SIMPLE_NAV_FACE_TARGET", "0") not in ("0", "false", "False")
# CAR_LIKE=1 (default): face target direction, no sideways drift, car-style motion.
# CAR_LIKE=0: original holonomic (mecanum) omni-directional driving.
CAR_LIKE = os.environ.get("EGGY_SIMPLE_NAV_CAR_LIKE", "1") not in ("0", "false", "False")
# When CAR_LIKE=1 and the goal pose has a non-zero yaw, the robot will spend
# up to this many extra seconds aligning yaw after arriving at the position.
YAW_ALIGN_TIMEOUT = float(os.environ.get("EGGY_SIMPLE_NAV_YAW_ALIGN_TIMEOUT", "5.0"))
DRIVE_HEADING_TOL = math.radians(float(
    os.environ.get("EGGY_SIMPLE_NAV_DRIVE_HEADING_TOL_DEG", "45.0")))


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
        # Odom coordinates are only a valid fallback for an odom-frame goal.
        # Treating them as map coordinates after a map TF failure can send the
        # robot toward an unrelated physical position.
        odom_frame = (o.header.frame_id if o else "").lstrip("/")
        if not o or frame.lstrip("/") != odom_frame:
            raise
        return o.pose.pose.position.x, o.pose.pose.position.y, yaw_from_q(o.pose.pose.orientation)


def main():
    global goal, cancelled, last_twist
    rospy.init_node("eggy_simple_navigator")
    listener = tf.TransformListener()
    cmd_pub = rospy.Publisher(rospy.get_param("~cmd_vel_topic", "/cmd_vel/navigation"), Twist, queue_size=1)
    state_pub = rospy.Publisher("/simple_nav/status", String, queue_size=10, latch=True)
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=5)
    rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=3)
    rospy.Subscriber("/simple_goal", PoseStamped, goal_cb, queue_size=1)
    rospy.Subscriber("/goal_pose", PoseStamped, goal_cb, queue_size=1)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_cb, queue_size=1)
    rospy.Subscriber("/simple_nav/cancel", String, cancel_cb, queue_size=1)

    rate_hz = 30.0
    rate = rospy.Rate(rate_hz)
    set_state(state_pub, "idle")
    last_status_pub = time.time()
    yaw_align_start = None  # timestamp when yaw-align phase began
    while not rospy.is_shutdown():
        if time.time() - last_status_pub > 0.5:
            state_pub.publish(String(data=nav_state))
            last_status_pub = time.time()
        with state_lock:
            g = goal
            is_cancelled = cancelled
        if not g:
            yaw_align_start = None
            rate.sleep(); continue
        if is_cancelled:
            publish_stop(cmd_pub)
            with state_lock:
                goal = None
                cancelled = False
            yaw_align_start = None
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

        # ---- Compute target velocities ----
        if CAR_LIKE:
            # During approach, always steer toward the target position. Goal yaw
            # is relevant only after reaching the positional tolerance.
            target_heading = math.atan2(dy, dx)
            heading_err = norm_ang(target_heading - yaw)

            # Forward speed: scale by distance, and by cosine of heading error.
            # When misaligned, the robot slows down and turns in place.
            speed = min(MAX_TRANS, KP_DIST * dist)
            if dist < SLOW_RADIUS:
                speed *= max(0.22, dist / SLOW_RADIUS)
            alignment = max(0.0, math.cos(heading_err))
            target_vx = 0.0 if abs(heading_err) > DRIVE_HEADING_TOL else speed * alignment
            target_vy = 0.0  # no lateral drift
            target_wz = clamp(KP_YAW * heading_err, -MAX_WZ, MAX_WZ)

            # ---- Arrival logic: position + optional yaw alignment ----
            if dist <= GOAL_TOL:
                if KEEP_FACE_TARGET:
                    # No specific goal yaw to achieve; just stop.
                    publish_stop(cmd_pub)
                    with state_lock:
                        goal = None
                    yaw_align_start = None
                    set_state(state_pub, "arrived", f"dist={dist:.2f}")
                    rate.sleep(); continue

                # Check yaw alignment.
                goal_yaw = yaw_from_q(g.pose.orientation)
                yaw_err = abs(norm_ang(goal_yaw - yaw))
                if yaw_err <= YAW_TOL:
                    publish_stop(cmd_pub)
                    with state_lock:
                        goal = None
                    yaw_align_start = None
                    set_state(state_pub, "arrived", f"dist={dist:.2f} yaw_err={math.degrees(yaw_err):.1f}deg")
                    rate.sleep(); continue

                # Yaw alignment phase: rotate in place.
                if yaw_align_start is None:
                    yaw_align_start = time.time()
                elif time.time() - yaw_align_start > YAW_ALIGN_TIMEOUT:
                    publish_stop(cmd_pub)
                    with state_lock:
                        goal = None
                    yaw_align_start = None
                    set_state(state_pub, "failed_yaw_timeout",
                              f"dist={dist:.2f} yaw_err={math.degrees(yaw_err):.1f}deg")
                    rate.sleep(); continue

                # Rotate in place toward goal yaw.
                tw = Twist()
                tw.linear.x = 0.0
                tw.linear.y = 0.0
                tw.angular.z = clamp(KP_YAW * norm_ang(goal_yaw - yaw), -MAX_WZ, MAX_WZ)
                last_twist = tw
                cmd_pub.publish(tw)
                set_state(state_pub, "aligning_yaw",
                          f"yaw_err={math.degrees(yaw_err):.1f}deg")
                rate.sleep(); continue

            yaw_align_start = None  # reset align timer when still approaching
        else:
            # Original holonomic (mecanum) omni-directional driving.
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

            if dist <= GOAL_TOL:
                publish_stop(cmd_pub)
                with state_lock:
                    goal = None
                set_state(state_pub, "arrived", f"dist={dist:.2f}")
                rate.sleep(); continue

        # A close obstacle blocks forward translation, but not an in-place turn.
        # This keeps the robot stopped while still allowing it to turn away.
        fd = front_distance()
        if fd is not None and fd < FRONT_STOP and target_vx > 0.01:
            publish_stop(cmd_pub)
            set_state(state_pub, "blocked", f"front={fd:.2f} dist={dist:.2f}")
            rate.sleep(); continue

        # ---- Apply acceleration limits and publish ----
        max_dv = ACC_XY / rate_hz
        max_dw = ACC_WZ / rate_hz
        tw = Twist()
        tw.linear.x = ramp(last_twist.linear.x, target_vx, max_dv)
        tw.linear.y = ramp(last_twist.linear.y, target_vy, max_dv)
        tw.angular.z = ramp(last_twist.angular.z, target_wz, max_dw)
        last_twist = tw
        cmd_pub.publish(tw)
        car_tag = "_car" if CAR_LIKE else ""
        set_state(state_pub, "driving"+car_tag,
                  f"dist={dist:.2f} vx={tw.linear.x:.2f} vy={tw.linear.y:.2f}")
        rate.sleep()


if __name__ == "__main__":
    main()
