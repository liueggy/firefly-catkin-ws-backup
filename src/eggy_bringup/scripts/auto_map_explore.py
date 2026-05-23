#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import os
import subprocess
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan


class AutoMapper:
    def __init__(self):
        rospy.init_node('auto_mapper', anonymous=False)
        self.duration_sec = float(rospy.get_param('~duration_sec', 60.0))
        self.map_path = rospy.get_param('~map_path', '/root/catkin_ws/maps/auto_map')
        self.linear_speed = float(rospy.get_param('~linear_speed', 0.08))
        self.turn_speed = float(rospy.get_param('~turn_speed', 0.25))
        self.save_map = bool(rospy.get_param('~save_map', True))
        self.start_gmapping = bool(rospy.get_param('~start_gmapping', True))
        self.gmapping_startup_delay = float(rospy.get_param('~gmapping_startup_delay', 3.0))
        self.imu_samples = int(rospy.get_param('~imu_calibration_samples', 30))

        self.imu_yaw = 0.0
        self.imu_last_time = None
        self.imu_bias = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.lock = threading.Lock()

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/external_imu/imu/data_raw', Imu, self.imu_cb, queue_size=1)
        rospy.on_shutdown(self.stop)

        self.wait_for_inputs()
        self.calibrate_imu()

    def wait_for_inputs(self):
        scan_timeout = float(rospy.get_param('~scan_timeout', 10.0))
        odom_timeout = float(rospy.get_param('~odom_timeout', 10.0))
        imu_timeout = float(rospy.get_param('~imu_timeout', 10.0))
        rospy.loginfo('Waiting for /scan, /odom and /external_imu/imu/data_raw...')
        rospy.wait_for_message('/scan', LaserScan, timeout=scan_timeout)
        rospy.wait_for_message('/odom', Odometry, timeout=odom_timeout)
        rospy.wait_for_message('/external_imu/imu/data_raw', Imu, timeout=imu_timeout)

    def calibrate_imu(self):
        samples = []
        rospy.loginfo('Calibrating IMU gyro bias with %d samples...', self.imu_samples)
        for _ in range(max(1, self.imu_samples)):
            msg = rospy.wait_for_message('/external_imu/imu/data_raw', Imu, timeout=2.0)
            samples.append(msg.angular_velocity.z)
        self.imu_bias = sum(samples) / len(samples)
        rospy.loginfo('IMU gyro bias: %.6f rad/s', self.imu_bias)

    def odom_cb(self, msg):
        with self.lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y

    def imu_cb(self, msg):
        with self.lock:
            now = rospy.Time.now().to_sec()
            if self.imu_last_time is not None:
                dt = now - self.imu_last_time
                if 0 < dt < 0.1:
                    self.imu_yaw += (msg.angular_velocity.z - self.imu_bias) * dt
            self.imu_last_time = now

    def stop(self):
        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)
            rospy.sleep(0.02)

    def forward(self, distance):
        with self.lock:
            sx, sy = self.odom_x, self.odom_y
            self.imu_yaw = 0.0
            self.imu_last_time = rospy.Time.now().to_sec()

        rate = rospy.Rate(20)
        heading_err = 0.0
        dist = 0.0
        while not rospy.is_shutdown():
            with self.lock:
                dx = self.odom_x - sx
                dy = self.odom_y - sy
                dist = math.sqrt(dx * dx + dy * dy)
                heading_err = self.imu_yaw
            if dist >= max(0.0, distance - 0.02):
                break
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            cmd.angular.z = max(-0.25, min(0.25, -2.0 * heading_err))
            self.pub.publish(cmd)
            rate.sleep()
        self.stop()
        rospy.loginfo('Forward %.2fm done, drift %.1f deg', dist, math.degrees(heading_err))

    def rotate(self, angle_deg):
        target_rad = math.radians(abs(angle_deg))
        direction = 1.0 if angle_deg > 0 else -1.0
        stop_threshold = math.radians(3.2)

        with self.lock:
            self.imu_yaw = 0.0
            self.imu_last_time = rospy.Time.now().to_sec()

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            with self.lock:
                current = abs(self.imu_yaw)
            remaining = target_rad - current
            if remaining <= stop_threshold:
                break
            if remaining < math.radians(15):
                wz = self.turn_speed * 0.4
            elif remaining < math.radians(30):
                wz = self.turn_speed * 0.6
            else:
                wz = self.turn_speed
            cmd = Twist()
            cmd.angular.z = direction * wz
            self.pub.publish(cmd)
            rate.sleep()

        self.stop()
        with self.lock:
            final = math.degrees(self.imu_yaw)
        rospy.loginfo('Rotate %.1f deg done, actual %.1f deg', angle_deg, final)

    def explore(self):
        rospy.loginfo('Starting low-memory exploration for %.0fs...', self.duration_sec)
        start_time = time.time()
        step = 0
        patterns = [
            ('forward', 0.7), ('rotate', 90), ('forward', 0.5), ('rotate', -90),
            ('forward', 0.8), ('rotate', 90), ('forward', 0.45), ('rotate', 90),
            ('forward', 0.6), ('rotate', -90), ('forward', 0.7), ('rotate', -90),
            ('forward', 0.5), ('rotate', 90), ('forward', 0.6), ('rotate', 180),
        ]

        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= self.duration_sec:
                rospy.loginfo('Time limit reached: %.0fs', elapsed)
                break
            action, value = patterns[step % len(patterns)]
            remaining_time = self.duration_sec - elapsed
            rospy.loginfo('[%.0fs/%.0fs] %s %.2f', elapsed, self.duration_sec, action, value)
            if action == 'forward':
                max_dist = min(value, remaining_time * self.linear_speed * 0.8)
                if max_dist <= 0.1:
                    break
                self.forward(max_dist)
            else:
                self.rotate(value)
            step += 1
            time.sleep(0.5)
        self.stop()

    def gmapping_command(self):
        scan_topic = rospy.get_param('~scan_topic', '/scan')
        params = [
            ('base_frame', 'base_frame', 'base_link'),
            ('odom_frame', 'odom_frame', 'odom'),
            ('map_update_interval', 'map_update_interval', 3.0),
            ('maxUrange', 'max_urange', 8.0),
            ('sigma', 'sigma', 0.05),
            ('kernelSize', 'kernel_size', 1),
            ('lstep', 'lstep', 0.05),
            ('astep', 'astep', 0.05),
            ('iterations', 'iterations', 5),
            ('lsigma', 'lsigma', 0.075),
            ('ogain', 'ogain', 3.0),
            ('lskip', 'lskip', 1),
            ('minimumScore', 'minimum_score', 50),
            ('linearUpdate', 'linear_update', 0.30),
            ('angularUpdate', 'angular_update', 0.30),
            ('temporalUpdate', 'temporal_update', 3.0),
            ('resampleThreshold', 'resample_threshold', 0.5),
            ('particles', 'particles', 20),
            ('xmin', 'xmin', -8.0),
            ('ymin', 'ymin', -8.0),
            ('xmax', 'xmax', 8.0),
            ('ymax', 'ymax', 8.0),
            ('delta', 'delta', 0.05),
            ('llsamplerange', 'llsamplerange', 0.01),
            ('llsamplestep', 'llsamplestep', 0.01),
            ('lasamplerange', 'lasamplerange', 0.005),
            ('lasamplestep', 'lasamplestep', 0.005),
            ('throttle_scans', 'throttle_scans', 1),
        ]
        cmd = ['rosrun', 'gmapping', 'slam_gmapping', 'scan:=' + scan_topic]
        for gmapping_name, private_name, default in params:
            value = rospy.get_param('~gmapping_' + private_name, default)
            cmd.append('_{}:={}'.format(gmapping_name, value))
        return cmd

    def save_current_map(self):
        if not self.save_map:
            return
        os.makedirs(os.path.dirname(self.map_path), exist_ok=True)
        try:
            rospy.wait_for_message('/map', OccupancyGrid, timeout=10.0)
        except Exception as exc:
            rospy.logwarn('No /map before saving: %s', exc)
        rospy.loginfo('Saving map to %s.yaml / .pgm', self.map_path)
        result = subprocess.run(
            ['rosrun', 'map_server', 'map_saver', '-f', self.map_path],
            capture_output=True, text=True, timeout=20)
        if result.returncode == 0:
            rospy.loginfo('Map saved to %s.yaml / %s.pgm', self.map_path, self.map_path)
        else:
            rospy.logwarn('Map save failed: %s', result.stderr.strip())


def main():
    mapper = AutoMapper()
    gmapping_proc = None
    try:
        if mapper.start_gmapping:
            cmd = mapper.gmapping_command()
            rospy.loginfo('Starting gmapping: %s', ' '.join(map(str, cmd)))
            quiet = bool(rospy.get_param('~gmapping_quiet', False))
            gmapping_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL if quiet else None,
                stderr=subprocess.STDOUT if quiet else None)
            rospy.sleep(mapper.gmapping_startup_delay)
        mapper.explore()
        mapper.save_current_map()
    finally:
        mapper.stop()
        if gmapping_proc:
            gmapping_proc.terminate()
            try:
                gmapping_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                gmapping_proc.kill()
            rospy.loginfo('gmapping stopped')


if __name__ == '__main__':
    main()
