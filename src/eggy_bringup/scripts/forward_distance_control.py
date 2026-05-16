#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前进指定距离闭环控制节点（IMU 航向纠偏版）
- 订阅 /odom 获取前进距离
- 订阅 /external_imu/imu/data_raw 的 gyro.z 实时积分航向
- 发布 /cmd_vel 控制底盘
- 以 IMU 为准持续纠正航向偏移
"""

import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import threading

class ForwardDistanceController:
    def __init__(self):
        rospy.init_node('forward_distance_controller', anonymous=True)
        
        # 参数
        self.target_distance = rospy.get_param('~target_distance', 1.0)
        self.linear_speed = rospy.get_param('~linear_speed', 0.1)
        self.heading_kp = rospy.get_param('~heading_kp', 2.0)
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.02)
        
        # IMU 航向积分状态
        self.imu_yaw_accumulated = 0.0  # 从启动前进开始积分的航向变化 (rad)
        self.imu_last_time = None
        self.imu_gyro_z = 0.0
        self.imu_bias_z = 0.0  # gyro.z 零偏
        self.imu_calibrated = False
        self.calibration_samples = []
        
        # Odom 距离状态
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        
        self.is_running = False
        self.lock = threading.Lock()
        
        # 发布器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 订阅器
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/external_imu/imu/data_raw', Imu, self.imu_callback)
        
        rospy.loginfo("ForwardDistanceController (IMU heading) initialized")
        rospy.loginfo(f"Target: {self.target_distance}m, Speed: {self.linear_speed}m/s, Heading Kp: {self.heading_kp}")
    
    def odom_callback(self, msg):
        with self.lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
    
    def imu_callback(self, msg):
        with self.lock:
            now = rospy.Time.now().to_sec()
            gz = msg.angular_velocity.z
            
            if not self.imu_calibrated:
                # 收集零偏样本
                self.calibration_samples.append(gz)
                if len(self.calibration_samples) >= 50:
                    self.imu_bias_z = sum(self.calibration_samples) / len(self.calibration_samples)
                    self.imu_calibrated = True
                    rospy.loginfo(f"IMU gyro.z bias calibrated: {self.imu_bias_z:.6f} rad/s ({len(self.calibration_samples)} samples)")
                return
            
            # 积分航向（仅在前进运行时）
            if self.is_running and self.imu_last_time is not None:
                dt = now - self.imu_last_time
                if 0 < dt < 0.1:  # 防止异常 dt
                    corrected_gz = gz - self.imu_bias_z
                    self.imu_yaw_accumulated += corrected_gz * dt
            
            self.imu_last_time = now
            self.imu_gyro_z = gz
    
    def get_distance_traveled(self):
        with self.lock:
            dx = self.odom_x - self.start_x
            dy = self.odom_y - self.start_y
            return math.sqrt(dx*dx + dy*dy)
    
    def get_imu_heading_error(self):
        """获取 IMU 积分的航向偏移量 (rad)，目标是保持 0"""
        with self.lock:
            return self.imu_yaw_accumulated
    
    def start_forward(self, distance=None, speed=None):
        if distance is not None:
            self.target_distance = distance
        if speed is not None:
            self.linear_speed = speed
        
        # 等待 IMU 校准完成
        if not self.imu_calibrated:
            rospy.loginfo("Waiting for IMU calibration...")
            while not self.imu_calibrated and not rospy.is_shutdown():
                rospy.sleep(0.1)
        
        with self.lock:
            if self.is_running:
                rospy.logwarn("Already running")
                return
            self.is_running = True
            self.imu_yaw_accumulated = 0.0  # 重置航向积分
            self.imu_last_time = rospy.Time.now().to_sec()
            self.start_x = self.odom_x
            self.start_y = self.odom_y
        
        rospy.loginfo(f"Forward: target={self.target_distance}m, speed={self.linear_speed}m/s")
        rospy.loginfo(f"Start: ({self.start_x:.3f}, {self.start_y:.3f})")
        
        rate = rospy.Rate(20)
        while self.is_running and not rospy.is_shutdown():
            distance_traveled = self.get_distance_traveled()
            heading_error = self.get_imu_heading_error()  # rad, 正=左偏, 负=右偏
            
            # 到达目标
            if distance_traveled >= self.target_distance - self.distance_threshold:
                with self.lock:
                    final_yaw = math.degrees(self.imu_yaw_accumulated)
                rospy.loginfo(f"Target reached: {distance_traveled:.3f}m, heading drift: {final_yaw:.2f}°")
                self.stop()
                break
            
            # 控制量
            linear_x = self.linear_speed
            
            # 航向纠偏：heading_error > 0 表示左偏，需要右转（angular.z < 0）
            angular_z = -self.heading_kp * heading_error
            angular_z = max(-0.3, min(0.3, angular_z))
            
            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            self.cmd_vel_pub.publish(cmd)
            
            rate.sleep()
        
        rospy.loginfo("Forward motion completed")
    
    def stop(self):
        with self.lock:
            self.is_running = False
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Stop")
    
    def run_interactive(self):
        rospy.loginfo("Commands:")
        rospy.loginfo("  'f [distance] [speed]' - forward")
        rospy.loginfo("  's' - stop")
        rospy.loginfo("  'q' - quit")
        
        while not rospy.is_shutdown():
            try:
                cmd_input = input(">> ").strip().lower()
                if cmd_input.startswith('f'):
                    parts = cmd_input.split()
                    d = float(parts[1]) if len(parts) > 1 else None
                    s = float(parts[2]) if len(parts) > 2 else None
                    self.start_forward(d, s)
                elif cmd_input == 's':
                    self.stop()
                elif cmd_input == 'q':
                    break
                else:
                    rospy.logwarn(f"Unknown: {cmd_input}")
            except ValueError as e:
                rospy.logwarn(f"Invalid: {e}")
            except KeyboardInterrupt:
                break
        self.stop()

if __name__ == '__main__':
    try:
        controller = ForwardDistanceController()
        controller.run_interactive()
    except rospy.ROSInterruptException:
        pass
