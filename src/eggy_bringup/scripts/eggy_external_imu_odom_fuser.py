#!/usr/bin/env python3
# -*- coding: utf-8
"""Fuse wheel_odom position with STM32 onboard MPU6050 gyro yaw.

Replaces external YB IMU with onboard MPU6050 for yaw tracking.

Yaw strategy:
  1. On startup, calibrate gyro.z bias over ~50 samples (~2.5s).
  2. Initialize base_yaw from wheel_odom quaternion.
  3. Continuously integrate gyro.z (bias-corrected) for yaw.
  4. Publish /odom = wheel_odom x/y + integrated gyro yaw.
  5. Publish TF odom->base_link.

Notes:
  - STM32 firmware zeros gyro.z when Flag_Stop==1 (first ~10s after boot).
  - STM32 already does zero-drift compensation, but we add a second-stage bias
    calibration for extra precision.
  - gyro.z sign: positive = counterclockwise (left turn).
"""
import math
import threading
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

lock = threading.RLock()

latest_wheel = None
base_yaw = None
gyro_bias = None
bias_sum = 0.0
bias_count = 0
BIAS_SAMPLES = 50
last_stamp = None


def norm_ang(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def wheel_cb(msg):
    global latest_wheel
    with lock:
        latest_wheel = msg


def imu_cb(msg):
    global base_yaw, gyro_bias, bias_sum, bias_count, last_stamp
    gz = msg.angular_velocity.z
    now = msg.header.stamp.to_sec() if msg.header.stamp.secs > 0 else rospy.Time.now().to_sec()

    with lock:
        # Phase 1: calibrate gyro bias
        if gyro_bias is None:
            bias_sum += gz
            bias_count += 1
            if bias_count >= BIAS_SAMPLES:
                gyro_bias = bias_sum / bias_count
                # Initialize yaw from latest wheel_odom
                if latest_wheel is not None:
                    q = latest_wheel.pose.pose.orientation
                    base_yaw = yaw_from_quat(q)
                else:
                    base_yaw = 0.0
                last_stamp = now
                rospy.loginfo('MPU6050 fuser ready: gyro_bias=%.6f rad/s, init_yaw=%.2f deg',
                              gyro_bias, math.degrees(base_yaw))
            return

        # Phase 2: integrate gyro.z
        dt = now - last_stamp
        last_stamp = now
        if dt <= 0 or dt > 0.2:
            return
        base_yaw = norm_ang(base_yaw + (gz - gyro_bias) * dt)


def main():
    global base_yaw
    rospy.init_node('eggy_external_imu_odom_fuser')

    odom_frame = rospy.get_param('~odom_frame_id', 'odom')
    base_frame = rospy.get_param('~base_frame_id', 'base_link')
    wheel_topic = rospy.get_param('~wheel_odom_topic', '/wheel_odom')
    imu_topic = rospy.get_param('~imu_topic', '/stm32/imu/data_raw')
    publish_tf = rospy.get_param('~publish_tf', True)
    rate_hz = float(rospy.get_param('~rate', 50.0))

    pub = rospy.Publisher('/odom', Odometry, queue_size=20)
    br = tf2_ros.TransformBroadcaster() if publish_tf else None

    rospy.Subscriber(wheel_topic, Odometry, wheel_cb, queue_size=50)
    rospy.Subscriber(imu_topic, Imu, imu_cb, queue_size=200)

    rospy.loginfo('MPU6050 odom fuser: wheel=%s imu=%s rate=%.0fHz tf=%s',
                  wheel_topic, imu_topic, rate_hz, publish_tf)

    rate = rospy.Rate(rate_hz)
    warned = False

    while not rospy.is_shutdown():
        with lock:
            w = latest_wheel
            yaw = base_yaw

        if w is None or yaw is None:
            if not warned:
                rospy.logwarn('waiting for wheel odom and MPU6050 gyro calib...')
                warned = True
            rate.sleep()
            continue

        now_t = rospy.Time.now()

        out = Odometry()
        out.header.stamp = now_t
        out.header.frame_id = odom_frame
        out.child_frame_id = base_frame
        # Position from wheel_odom (x, y are reliable)
        out.pose.pose.position = w.pose.pose.position
        # Orientation from integrated gyro yaw
        qx, qy, qz, qw = quat_from_yaw(yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = w.pose.covariance
        # Twist from wheel_odom
        out.twist.twist.linear = w.twist.twist.linear
        out.twist.twist.angular.z = w.twist.twist.angular.z
        out.twist.covariance = w.twist.covariance
        pub.publish(out)

        if br:
            tr = TransformStamped()
            tr.header.stamp = now_t
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
