#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""STM32 mecanum base driver for ROS Noetic.

Protocol summary:
- RX from STM32: 24 bytes, 0x7B ... checksum ... 0x7D.
- TX to STM32: 11 bytes, cmd_vel as X/Y mm/s and Z rad/s*1000.
"""

import math
import struct
import threading
import time

import rospy
import serial
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, UInt8

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
STATUS_FRAME_SIZE = 24
CONTROL_FRAME_SIZE = 11


def clamp(value, low, high):
    return max(low, min(high, value))


def int16_be(data, offset):
    return struct.unpack('>h', data[offset:offset + 2])[0]


def xor_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum & 0xFF


class Stm32BaseDriver:
    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = int(rospy.get_param('~baudrate', 115200))
        self.control_rate = float(rospy.get_param('~control_rate', 20.0))
        self.status_timeout = float(rospy.get_param('~status_timeout', 1.0))
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.imu_frame_id = rospy.get_param('~imu_frame_id', 'imu_link')
        self.publish_tf = bool(rospy.get_param('~publish_tf', True))
        self.acc_lsb_per_g = float(rospy.get_param('~acc_lsb_per_g', 16384.0))
        self.standard_gravity = float(rospy.get_param('~standard_gravity', 9.80665))
        self.gyro_lsb_per_deg_s = float(rospy.get_param('~gyro_lsb_per_deg_s', 16.4))
        self.max_linear_x = float(rospy.get_param('~max_linear_x', 0.5))
        self.max_linear_y = float(rospy.get_param('~max_linear_y', 0.5))
        self.max_angular_z = float(rospy.get_param('~max_angular_z', 1.0))
        self.cmd_vel_timeout = float(rospy.get_param('~cmd_vel_timeout', 0.5))

        self.serial = serial.Serial(self.port, self.baudrate, timeout=0.02)
        self.serial_lock = threading.Lock()
        self.read_buffer = bytearray()
        self.latest_cmd = Twist()
        self.latest_cmd_time = rospy.Time(0)
        self.last_status_time = rospy.Time(0)
        self.last_odom_time = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_voltage = 0.0
        self.last_flag_stop = 0
        self.valid_frames = 0
        self.bad_frames = 0
        self.running = True

        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=20)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=20)
        self.voltage_pub = rospy.Publisher('battery/voltage', Float32, queue_size=10)
        self.flag_stop_pub = rospy.Publisher('base/flag_stop', UInt8, queue_size=10)
        self.diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=10)
        self.cmd_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        self.reader_thread = threading.Thread(target=self.read_loop)
        self.reader_thread.daemon = True
        self.reader_thread.start()

        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_timer_cb)
        self.diagnostic_timer = rospy.Timer(rospy.Duration(1.0), self.diagnostic_timer_cb)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('STM32 base driver started: port=%s baudrate=%d', self.port, self.baudrate)

    def cmd_vel_callback(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_time = rospy.Time.now()

    def build_control_frame(self, vx, vy, wz):
        x = int(round(clamp(vx, -self.max_linear_x, self.max_linear_x) * 1000.0))
        y = int(round(clamp(vy, -self.max_linear_y, self.max_linear_y) * 1000.0))
        z = int(round(clamp(wz, -self.max_angular_z, self.max_angular_z) * 1000.0))
        payload = bytearray([FRAME_HEADER, 0x00, 0x00])
        payload.extend(struct.pack('>h', x))
        payload.extend(struct.pack('>h', y))
        payload.extend(struct.pack('>h', z))
        payload.append(xor_checksum(payload))
        payload.append(FRAME_TAIL)
        return bytes(payload)

    def control_timer_cb(self, _event):
        now = rospy.Time.now()
        if (now - self.latest_cmd_time).to_sec() > self.cmd_vel_timeout:
            vx = vy = wz = 0.0
        else:
            vx = self.latest_cmd.linear.x
            vy = self.latest_cmd.linear.y
            wz = self.latest_cmd.angular.z
        frame = self.build_control_frame(vx, vy, wz)
        try:
            with self.serial_lock:
                self.serial.write(frame)
        except serial.SerialException as exc:
            rospy.logerr_throttle(2.0, 'Failed to write serial command: %s', exc)

    def read_loop(self):
        while self.running and not rospy.is_shutdown():
            try:
                chunk = self.serial.read(256)
                if chunk:
                    self.read_buffer.extend(chunk)
                    self.extract_frames()
            except serial.SerialException as exc:
                rospy.logerr_throttle(2.0, 'Serial read error: %s', exc)
                time.sleep(0.1)

    def extract_frames(self):
        while True:
            try:
                start = self.read_buffer.index(FRAME_HEADER)
            except ValueError:
                self.read_buffer.clear()
                return
            if start:
                del self.read_buffer[:start]
            if len(self.read_buffer) < STATUS_FRAME_SIZE:
                return
            frame = bytes(self.read_buffer[:STATUS_FRAME_SIZE])
            del self.read_buffer[:STATUS_FRAME_SIZE]
            if frame[-1] != FRAME_TAIL:
                self.bad_frames += 1
                continue
            if xor_checksum(frame[:22]) != frame[22]:
                self.bad_frames += 1
                continue
            self.valid_frames += 1
            self.handle_status_frame(frame)

    def handle_status_frame(self, frame):
        stamp = rospy.Time.now()
        self.last_status_time = stamp
        flag_stop = frame[1]
        vx = int16_be(frame, 2) / 1000.0
        vy = int16_be(frame, 4) / 1000.0
        wz = int16_be(frame, 6) / 1000.0
        acc_x_raw = int16_be(frame, 8)
        acc_y_raw = int16_be(frame, 10)
        acc_z_raw = int16_be(frame, 12)
        gyro_x_raw = int16_be(frame, 14)
        gyro_y_raw = int16_be(frame, 16)
        gyro_z_raw = int16_be(frame, 18)
        voltage = int16_be(frame, 20) / 1000.0
        self.last_voltage = voltage
        self.last_flag_stop = flag_stop

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.imu_frame_id
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.linear_acceleration.x = acc_x_raw / self.acc_lsb_per_g * self.standard_gravity
        imu_msg.linear_acceleration.y = acc_y_raw / self.acc_lsb_per_g * self.standard_gravity
        imu_msg.linear_acceleration.z = acc_z_raw / self.acc_lsb_per_g * self.standard_gravity
        gyro_scale = math.pi / 180.0 / self.gyro_lsb_per_deg_s
        imu_msg.angular_velocity.x = gyro_x_raw * gyro_scale
        imu_msg.angular_velocity.y = gyro_y_raw * gyro_scale
        imu_msg.angular_velocity.z = gyro_z_raw * gyro_scale
        self.imu_pub.publish(imu_msg)

        self.publish_odometry(stamp, vx, vy, wz)
        self.voltage_pub.publish(Float32(voltage))
        self.flag_stop_pub.publish(UInt8(flag_stop))

    def publish_odometry(self, stamp, vx, vy, wz):
        if self.last_odom_time is None:
            dt = 0.0
        else:
            dt = (stamp - self.last_odom_time).to_sec()
        self.last_odom_time = stamp
        if 0.0 < dt < 1.0:
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            self.x += (vx * cos_yaw - vy * sin_yaw) * dt
            self.y += (vx * sin_yaw + vy * cos_yaw) * dt
            self.yaw += wz * dt
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

    def diagnostic_timer_cb(self, _event):
        now = rospy.Time.now()
        age = (now - self.last_status_time).to_sec() if self.last_status_time.to_sec() > 0 else 999.0
        status = DiagnosticStatus()
        status.name = 'eggy_base_driver/stm32_serial'
        status.hardware_id = self.port
        if age > self.status_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = 'No recent STM32 status frame'
        elif self.last_flag_stop != 0:
            status.level = DiagnosticStatus.WARN
            status.message = 'STM32 reports Flag_Stop != 0'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        status.values = [
            KeyValue('port', self.port),
            KeyValue('voltage_v', f'{self.last_voltage:.3f}'),
            KeyValue('flag_stop', str(self.last_flag_stop)),
            KeyValue('valid_frames', str(self.valid_frames)),
            KeyValue('bad_frames', str(self.bad_frames)),
            KeyValue('last_status_age_s', f'{age:.3f}'),
        ]
        array = DiagnosticArray()
        array.header.stamp = now
        array.status.append(status)
        self.diag_pub.publish(array)

    def shutdown(self):
        self.running = False
        try:
            stop = self.build_control_frame(0.0, 0.0, 0.0)
            with self.serial_lock:
                for _ in range(5):
                    self.serial.write(stop)
                    time.sleep(0.02)
                self.serial.close()
        except Exception:
            pass


if __name__ == '__main__':
    rospy.init_node('stm32_base_driver')
    driver = Stm32BaseDriver()
    rospy.spin()
