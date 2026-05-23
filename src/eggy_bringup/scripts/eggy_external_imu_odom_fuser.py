#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fuse wheel odometry translation with external IMU gyro-integrated yaw.

Manual-compliant external IMU usage:
  - Official Euler 0x26: roll/pitch/yaw are float radians.
  - Official Quaternion 0x16: Q0=w, Q1/Q2/Q3=x/y/z (handled in driver).

Yaw policy for navigation:
  - Initialize base yaw from official Euler yaw + pi mounting compensation.
  - Then integrate external raw gyro.z from /external_imu/imu/data_raw.
  - Do NOT continuously overwrite yaw with Euler/quaternion because measured
    fused yaw can drift after rotation under magnetic/fusion influence.

Mounting:
  Initial offset is configurable by ~imu_to_base_yaw_offset_deg.
  2026-05-23 forward calibration measured +X motion vs yaw diff about -150 deg,
  initial +30 deg was insufficient because wheel pose xy and IMU yaw were mixed.
  Forward calibration showed yaw should be about -122.5 deg offset for current mounting.
  Fuser integrates wheel odom *delta translation* in local wheel frame using external IMU yaw,
  instead of copying wheel_odom.position directly. This keeps /odom x/y/yaw self-consistent.
  A pure yaw mounting rotation does not invert gyro.z.
"""
import math
import threading
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped

lock = threading.RLock()
latest_wheel = None
latest_raw = None
latest_euler_yaw = None
latest_euler_time = None
base_yaw = None
last_raw_stamp = None
last_raw_seq = None
imu_to_base_yaw_offset = math.radians(-122.5)
fused_x = 0.0
fused_y = 0.0
last_wheel_x = None
last_wheel_y = None
last_wheel_yaw = None


def norm_ang(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def imu_yaw_to_base_yaw(imu_yaw):
    return norm_ang(imu_yaw + imu_to_base_yaw_offset)


def yaw_from_quat(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def wheel_cb(msg):
    global latest_wheel, fused_x, fused_y, last_wheel_x, last_wheel_y, last_wheel_yaw
    with lock:
        latest_wheel = msg
        pos = msg.pose.pose.position
        wyaw = yaw_from_quat(msg.pose.pose.orientation)
        if last_wheel_x is None:
            last_wheel_x, last_wheel_y, last_wheel_yaw = pos.x, pos.y, wyaw
            return
        dx = pos.x - last_wheel_x
        dy = pos.y - last_wheel_y
        # Convert wheel odom frame delta to robot-local delta using previous wheel yaw.
        cy = math.cos(last_wheel_yaw)
        sy = math.sin(last_wheel_yaw)
        local_dx = cy * dx + sy * dy
        local_dy = -sy * dx + cy * dy
        if base_yaw is not None:
            by = base_yaw
            cb = math.cos(by)
            sb = math.sin(by)
            fused_x += cb * local_dx - sb * local_dy
            fused_y += sb * local_dx + cb * local_dy
        last_wheel_x, last_wheel_y, last_wheel_yaw = pos.x, pos.y, wyaw


def euler_cb(msg):
    global latest_euler_yaw, latest_euler_time
    with lock:
        latest_euler_yaw = float(msg.data)
        latest_euler_time = rospy.Time.now().to_sec()


def raw_cb(msg):
    global latest_raw, base_yaw, last_raw_stamp, last_raw_seq
    with lock:
        latest_raw = msg
        now = rospy.Time.now().to_sec()
        if base_yaw is None:
            if latest_euler_yaw is not None and latest_euler_time is not None and now - latest_euler_time < 2.0:
                base_yaw = imu_yaw_to_base_yaw(latest_euler_yaw)
                rospy.loginfo('external IMU yaw initialized from official euler: imu=%.3f base=%.3f', latest_euler_yaw, base_yaw)
            else:
                return
            last_raw_stamp = msg.header.stamp.to_sec() if msg.header.stamp else now
            last_raw_seq = msg.header.seq
            return
        t = msg.header.stamp.to_sec() if msg.header.stamp else now
        if last_raw_stamp is not None and msg.header.seq != last_raw_seq:
            dt = t - last_raw_stamp
            if 0.0 < dt < 0.2:
                base_yaw = norm_ang(base_yaw + msg.angular_velocity.z * dt)
        last_raw_stamp = t
        last_raw_seq = msg.header.seq


def main():
    global imu_to_base_yaw_offset
    rospy.init_node('eggy_external_imu_odom_fuser')
    imu_to_base_yaw_offset = math.radians(float(rospy.get_param('~imu_to_base_yaw_offset_deg', -122.5)))
    odom_frame = rospy.get_param('~odom_frame_id', 'odom')
    base_frame = rospy.get_param('~base_frame_id', 'base_link')
    wheel_topic = rospy.get_param('~wheel_odom_topic', '/wheel_odom')
    raw_topic = rospy.get_param('~external_imu_raw_topic', '/external_imu/imu/data_raw')
    euler_topic = rospy.get_param('~external_imu_euler_topic', '/external_imu/imu/euler')
    publish_tf = rospy.get_param('~publish_tf', True)
    rate_hz = float(rospy.get_param('~rate', 50.0))

    pub = rospy.Publisher('/odom', Odometry, queue_size=20)
    br = tf2_ros.TransformBroadcaster() if publish_tf else None
    rospy.Subscriber(wheel_topic, Odometry, wheel_cb, queue_size=50)
    rospy.Subscriber(euler_topic, Float32, euler_cb, queue_size=100)
    rospy.Subscriber(raw_topic, Imu, raw_cb, queue_size=200)

    rospy.loginfo('external IMU fuser active: wheel=%s raw=%s euler_init=%s -> /odom', wheel_topic, raw_topic, euler_topic)
    rospy.loginfo('external imu yaw: euler rad init + offset %.1f deg, then gyro.z integration', math.degrees(imu_to_base_yaw_offset))

    rate = rospy.Rate(rate_hz)
    warned = False
    while not rospy.is_shutdown():
        with lock:
            w = latest_wheel
            raw = latest_raw
            yaw = base_yaw
        if w is None or yaw is None:
            if not warned:
                rospy.logwarn('waiting for wheel odom and external IMU yaw init')
                warned = True
            rate.sleep(); continue

        now_t = rospy.Time.now()
        q = quat_from_yaw(yaw)
        out = Odometry()
        out.header.stamp = now_t
        out.header.frame_id = odom_frame
        out.child_frame_id = base_frame
        out.pose.pose.position.x = fused_x
        out.pose.pose.position.y = fused_y
        out.pose.pose.position.z = w.pose.pose.position.z
        out.pose.pose.orientation.x = q[0]
        out.pose.pose.orientation.y = q[1]
        out.pose.pose.orientation.z = q[2]
        out.pose.pose.orientation.w = q[3]
        out.pose.covariance = w.pose.covariance
        out.twist.twist.linear = w.twist.twist.linear
        if raw is not None:
            out.twist.twist.angular.z = raw.angular_velocity.z
        else:
            out.twist.twist.angular = w.twist.twist.angular
        out.twist.covariance = w.twist.covariance
        pub.publish(out)

        if br:
            tr = TransformStamped()
            tr.header.stamp = out.header.stamp
            tr.header.frame_id = odom_frame
            tr.child_frame_id = base_frame
            tr.transform.translation.x = out.pose.pose.position.x
            tr.transform.translation.y = out.pose.pose.position.y
            tr.transform.translation.z = out.pose.pose.position.z
            tr.transform.rotation = out.pose.pose.orientation
            br.sendTransform(tr)
        rate.sleep()


if __name__ == '__main__':
    main()
