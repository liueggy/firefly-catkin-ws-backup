#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动建图流程：启动 gmapping + 自动探索 + 保存地图
运行约 1 分钟后自动停止并保存地图
"""

import rospy
import math
import time
import subprocess
import signal
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import threading

class AutoMapper:
    def __init__(self):
        rospy.init_node('auto_mapper', anonymous=True)
        
        # IMU 航向
        self.imu_yaw = 0.0
        self.imu_last_time = None
        self.imu_bias = 0.0
        self.lock = threading.Lock()
        
        # Odom
        self.odom_x = 0.0
        self.odom_y = 0.0
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/external_imu/imu/data_raw', Imu, self.imu_cb)
        
        # IMU 零偏校准
        rospy.loginfo("Calibrating IMU...")
        samples = []
        for i in range(50):
            msg = rospy.wait_for_message('/external_imu/imu/data_raw', Imu, timeout=2)
            samples.append(msg.angular_velocity.z)
        self.imu_bias = sum(samples) / len(samples)
        rospy.loginfo(f"IMU bias: {self.imu_bias:.6f} rad/s")
    
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
        self.pub.publish(Twist())
    
    def forward(self, distance, speed=0.1):
        """前进指定距离，IMU 航向纠偏"""
        with self.lock:
            sx, sy = self.odom_x, self.odom_y
            self.imu_yaw = 0.0
            self.imu_last_time = rospy.Time.now().to_sec()
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            with self.lock:
                dx = self.odom_x - sx
                dy = self.odom_y - sy
                dist = math.sqrt(dx*dx + dy*dy)
                heading_err = self.imu_yaw
            if dist >= distance - 0.02:
                break
            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = -2.0 * heading_err
            cmd.angular.z = max(-0.3, min(0.3, cmd.angular.z))
            self.pub.publish(cmd)
            rate.sleep()
        self.stop()
        rospy.loginfo(f"  Forward {dist:.2f}m done, drift {math.degrees(heading_err):.1f}°")
    
    def rotate(self, angle_deg, max_wz=0.30):
        """旋转指定角度，IMU 闭环"""
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
            
            # 分段减速
            if remaining < math.radians(15):
                wz = max_wz * 0.4
            elif remaining < math.radians(30):
                wz = max_wz * 0.6
            else:
                wz = max_wz
            
            cmd = Twist()
            cmd.angular.z = direction * wz
            self.pub.publish(cmd)
            rate.sleep()
        
        self.stop()
        with self.lock:
            final = math.degrees(self.imu_yaw)
        rospy.loginfo(f"  Rotate {angle_deg}° done, actual {final:.1f}°")
    
    def explore(self, duration_sec=60):
        """自动探索：前进-旋转循环"""
        rospy.loginfo(f"Starting exploration for {duration_sec}s...")
        start_time = time.time()
        step = 0
        
        # 探索模式：前进 → 左转 → 前进 → 右转 → ...
        patterns = [
            ('forward', 0.8),
            ('rotate', 90),
            ('forward', 0.6),
            ('rotate', -90),
            ('forward', 1.0),
            ('rotate', 90),
            ('forward', 0.5),
            ('rotate', 90),
            ('forward', 0.7),
            ('rotate', -90),
            ('forward', 0.9),
            ('rotate', -90),
            ('forward', 0.6),
            ('rotate', 90),
            ('forward', 0.8),
            ('rotate', 180),
        ]
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration_sec:
                rospy.loginfo(f"Time's up ({elapsed:.0f}s), stopping exploration")
                break
            
            pattern = patterns[step % len(patterns)]
            action = pattern[0]
            value = pattern[1]
            
            remaining_time = duration_sec - elapsed
            rospy.loginfo(f"[{elapsed:.0f}s/{duration_sec}s] Step {step+1}: {action} {value}")
            
            if action == 'forward':
                # 限制前进距离，确保不超时
                max_dist = min(value, remaining_time * 0.08)
                if max_dist > 0.1:
                    self.forward(max_dist, speed=0.1)
                else:
                    break
            elif action == 'rotate':
                self.rotate(value, max_wz=0.30)
            
            step += 1
            time.sleep(0.5)  # 步骤间短暂停顿
        
        self.stop()
        rospy.loginfo("Exploration completed")


def main():
    mapper = AutoMapper()
    gmapping_proc = None
    
    try:
        # 启动 gmapping
        rospy.loginfo("Starting gmapping...")
        gmapping_proc = subprocess.Popen(
            ['rosrun', 'gmapping', 'slam_gmapping',
             'scan:=/scan',
             '_base_frame:=base_link',
             '_odom_frame:=odom',
             '_map_update_interval:=2.0',
             '_maxUrange:=8.0',
             '_sigma:=0.05',
             '_kernelSize:=1',
             '_lstep:=0.05',
             '_astep:=0.05',
             '_iterations:=5',
             '_lsigma:=0.075',
             '_ogain:=3.0',
             '_lskip:=0',
             '_minimumScore:=50',
             '_linearUpdate:=0.2',
             '_angularUpdate:=0.2',
             '_temporalUpdate:=2.0',
             '_resampleThreshold:=0.5',
             '_particles:=30',
             '_xmin:=-10.0', '_ymin:=-10.0',
             '_xmax:=10.0', '_ymax:=10.0',
             '_delta:=0.05',
             '_llsamplerange:=0.01',
             '_llsamplestep:=0.01',
             '_lasamplerange:=0.005',
             '_lasamplestep:=0.005'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        rospy.loginfo("gmapping started, waiting 3s for initialization...")
        time.sleep(3)
        
        # 自动探索 60 秒
        mapper.explore(duration_sec=60)
        
        # 保存地图
        rospy.loginfo("Saving map...")
        map_path = '/root/catkin_ws/maps/auto_map'
        subprocess.run(['mkdir', '-p', '/root/catkin_ws/maps'], check=True)
        result = subprocess.run(
            ['rosrun', 'map_server', 'map_saver', '-f', map_path],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            rospy.loginfo(f"Map saved to {map_path}.pgm / {map_path}.yaml")
        else:
            rospy.logwarn(f"Map save failed: {result.stderr}")
        
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        # 停车
        mapper.stop()
        # 关闭 gmapping
        if gmapping_proc:
            gmapping_proc.terminate()
            gmapping_proc.wait(timeout=5)
            rospy.loginfo("gmapping stopped")
        rospy.loginfo("Auto mapping completed!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
