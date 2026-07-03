#!/usr/bin/env python3
import json
import math
import os
import subprocess
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class FastAutoMapper:
    def __init__(self):
        rospy.init_node("auto_mapper", anonymous=False)

        self.duration_sec = self.param_float("~duration_sec", 180.0, 5.0, 3600.0)
        self.linear_speed = self.param_float("~linear_speed", 0.16, 0.03, 0.35)
        self.turn_speed = self.param_float("~turn_speed", 0.45, 0.10, 0.90)
        self.backup_speed = self.param_float("~backup_speed", 0.08, 0.03, 0.18)
        self.min_front_clearance = self.param_float("~min_front_clearance", 0.42, 0.20, 1.20)
        self.slow_front_clearance = self.param_float("~slow_front_clearance", 1.05, 0.30, 2.50)
        self.predict_front_clearance = self.param_float("~predict_front_clearance", 1.45, 0.50, 3.50)
        self.side_clearance = self.param_float("~side_clearance", 0.32, 0.15, 1.00)
        self.corridor_side_clearance = self.param_float(
            "~corridor_side_clearance", 0.24, 0.12, 0.80
        )
        self.stop_clearance = self.param_float("~stop_clearance", 0.26, 0.12, 0.80)
        self.stuck_timeout = self.param_float("~stuck_timeout", 2.5, 0.5, 10.0)
        self.scan_timeout = self.param_float("~scan_timeout", 1.0, 0.2, 5.0)
        self.command_rate_limit = self.param_float("~command_rate_limit", 0.10, 0.02, 0.40)
        self.angular_rate_limit = self.param_float("~angular_rate_limit", 0.18, 0.04, 0.60)
        self.gap_heading_range = self.param_float("~gap_heading_range", 95.0, 35.0, 135.0)
        self.gap_heading_step = self.param_float("~gap_heading_step", 10.0, 3.0, 20.0)
        self.gap_width = self.param_float("~gap_width", 34.0, 16.0, 70.0)
        self.heading_gain = self.param_float("~heading_gain", 1.15, 0.30, 2.50)
        self.heading_hold_sec = self.param_float("~heading_hold_sec", 0.8, 0.1, 3.0)
        self.forward_bias = self.param_float("~forward_bias", 0.18, 0.0, 0.80)
        self.end_spin_enabled = bool(rospy.get_param("~end_spin_enabled", True))
        self.end_spin_rotations = self.param_float("~end_spin_rotations", 2.0, 0.0, 5.0)
        self.end_spin_speed = self.param_float("~end_spin_speed", 0.45, 0.15, 0.80)
        self.save_map = rospy.get_param("~save_map", True)
        self.map_path = rospy.get_param("~map_path", "/root/catkin_ws/maps/auto_explore_map")
        self.start_gmapping = rospy.get_param("~start_gmapping", False)
        self.gmapping_startup_delay = self.param_float("~gmapping_startup_delay", 3.0, 0.0, 20.0)
        self.status_rate_hz = self.param_float("~status_rate_hz", 2.0, 0.2, 10.0)
        self.completion_enabled = bool(rospy.get_param("~completion_enabled", True))
        self.completion_min_elapsed = self.param_float(
            "~completion_min_elapsed", 60.0, 10.0, 1800.0
        )
        self.map_stable_sec = self.param_float("~map_stable_sec", 30.0, 5.0, 300.0)
        self.frontier_stable_sec = self.param_float(
            "~frontier_stable_sec", 15.0, 3.0, 120.0
        )
        self.map_growth_cells = int(rospy.get_param("~map_growth_cells", 30))
        self.frontier_min_cells = int(rospy.get_param("~frontier_min_cells", 18))
        self.min_known_cells = int(rospy.get_param("~min_known_cells", 800))

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.status_pub = rospy.Publisher("/auto_explore/status", String, queue_size=10, latch=True)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber("/auto_explore/stop", Bool, self.stop_cb, queue_size=1)
        rospy.Subscriber("/auto_explore/stop_text", String, self.stop_text_cb, queue_size=1)

        self.scan = None
        self.scan_stamp = 0.0
        self.stop_requested = False
        self.last_progress_time = time.time()
        self.last_front = None
        self.start_time = time.time()
        self.state = "init"
        self.state_until = 0.0
        self.turn_direction = 1.0
        self.known_cells = 0
        self.frontier_cells = 0
        self.max_known_cells = 0
        self.last_map_growth_time = time.time()
        self.low_frontier_since = None
        self.map_stamp = 0.0
        self.stop_reason = ""
        self.last_cmd_vx = 0.0
        self.last_cmd_wz = 0.0
        self.turn_hold_until = 0.0
        self.heading_hold_until = 0.0
        self.target_heading_deg = 0.0
        self.target_clearance = 0.0
        rospy.on_shutdown(self.stop_motion)

    @staticmethod
    def param_float(name, default, lower, upper):
        value = float(rospy.get_param(name, default))
        return max(lower, min(upper, value))

    def stop_cb(self, msg):
        if msg.data:
            self.stop_requested = True
            self.publish_status("manual_stop")

    def stop_text_cb(self, msg):
        if msg.data.strip().lower() in ("1", "true", "stop", "halt", "cancel"):
            self.stop_requested = True
            self.publish_status("manual_stop")

    def scan_cb(self, msg):
        self.scan = msg
        self.scan_stamp = time.time()

    def map_cb(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        data = msg.data
        if width <= 2 or height <= 2 or len(data) != width * height:
            return
        known = sum(1 for value in data if value >= 0)
        frontiers = 0
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                idx = row + x
                if data[idx] != 0:
                    continue
                if (
                    data[idx - 1] < 0
                    or data[idx + 1] < 0
                    or data[idx - width] < 0
                    or data[idx + width] < 0
                ):
                    frontiers += 1
        now = time.time()
        if known >= self.max_known_cells + max(1, self.map_growth_cells):
            self.max_known_cells = known
            self.last_map_growth_time = now
        else:
            self.max_known_cells = max(self.max_known_cells, known)
        if frontiers < self.frontier_min_cells:
            if self.low_frontier_since is None:
                self.low_frontier_since = now
        else:
            self.low_frontier_since = None
        self.known_cells = known
        self.frontier_cells = frontiers
        self.map_stamp = now

    def publish_status(self, state, **extra):
        payload = {
            "state": state,
            "elapsed_sec": round(time.time() - self.start_time, 1),
            "known_cells": self.known_cells,
            "frontier_cells": self.frontier_cells,
            "stop_reason": self.stop_reason,
            "last_cmd_vx": round(self.last_cmd_vx, 3),
            "last_cmd_wz": round(self.last_cmd_wz, 3),
            "target_heading_deg": round(self.target_heading_deg, 1),
            "target_clearance": round(self.target_clearance, 2),
        }
        payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def completion_reason(self, elapsed):
        if not self.completion_enabled or elapsed < self.completion_min_elapsed:
            return ""
        if self.known_cells < self.min_known_cells or self.map_stamp <= 0:
            return ""
        map_stable = time.time() - self.last_map_growth_time >= self.map_stable_sec
        frontier_stable = (
            self.low_frontier_since is not None
            and time.time() - self.low_frontier_since >= self.frontier_stable_sec
        )
        if map_stable and frontier_stable:
            return "map_complete"
        return ""

    def stop_motion(self):
        self.last_cmd_vx = 0.0
        self.last_cmd_wz = 0.0
        cmd = Twist()
        for _ in range(6):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.03)

    def end_spin(self):
        if not self.end_spin_enabled or self.end_spin_rotations <= 0.0:
            return
        self.publish_status("end_spin", rotations=self.end_spin_rotations)
        duration = (2.0 * math.pi * self.end_spin_rotations) / self.end_spin_speed
        deadline = time.time() + duration
        rate = rospy.Rate(20)
        while time.time() < deadline and not rospy.is_shutdown():
            self.command(0.0, self.end_spin_speed, smooth=False)
            rate.sleep()
        self.stop_motion()

    def sector_min(self, center_deg, width_deg, max_range=8.0):
        if self.scan is None:
            return 0.0
        angle_min = self.scan.angle_min
        angle_inc = self.scan.angle_increment
        if angle_inc == 0.0:
            return 0.0
        center = math.radians(center_deg)
        half = math.radians(width_deg) * 0.5
        low = center - half
        high = center + half
        best = max_range
        found = False
        for idx, value in enumerate(self.scan.ranges):
            if not math.isfinite(value):
                continue
            if value < self.scan.range_min or value > min(self.scan.range_max, max_range):
                continue
            angle = angle_min + idx * angle_inc
            if low <= angle <= high:
                best = min(best, value)
                found = True
        return best if found else max_range

    def sector_mean(self, center_deg, width_deg, max_range=5.0):
        if self.scan is None:
            return 0.0
        angle_min = self.scan.angle_min
        angle_inc = self.scan.angle_increment
        if angle_inc == 0.0:
            return 0.0
        center = math.radians(center_deg)
        half = math.radians(width_deg) * 0.5
        low = center - half
        high = center + half
        values = []
        for idx, value in enumerate(self.scan.ranges):
            if not math.isfinite(value):
                continue
            if value < self.scan.range_min or value > min(self.scan.range_max, max_range):
                continue
            angle = angle_min + idx * angle_inc
            if low <= angle <= high:
                values.append(value)
        if not values:
            return max_range
        values.sort()
        cut = max(1, int(len(values) * 0.35))
        return sum(values[:cut]) / float(cut)

    def choose_turn_direction(self):
        left = self.sector_mean(45, 90) + 0.45 * self.sector_mean(90, 60)
        right = self.sector_mean(-45, 90) + 0.45 * self.sector_mean(-90, 60)
        return 1.0 if left >= right else -1.0

    def choose_open_heading(self):
        best_heading = 0.0
        best_clearance = 0.0
        best_score = -999.0
        heading = -self.gap_heading_range
        while heading <= self.gap_heading_range + 0.001:
            near_clearance = self.sector_min(heading, self.gap_width, max_range=4.0)
            mean_clearance = self.sector_mean(heading, self.gap_width * 1.35, max_range=4.0)
            if near_clearance < self.stop_clearance:
                heading += self.gap_heading_step
                continue

            forward_score = 1.0 - min(abs(heading), self.gap_heading_range) / self.gap_heading_range
            continuity = 1.0 - min(abs(heading - self.target_heading_deg), self.gap_heading_range) / self.gap_heading_range
            score = (
                1.35 * mean_clearance
                + 0.75 * near_clearance
                + self.forward_bias * forward_score
                + 0.20 * continuity
                - 0.004 * abs(heading)
            )
            if score > best_score:
                best_score = score
                best_heading = heading
                best_clearance = min(mean_clearance, near_clearance)
            heading += self.gap_heading_step

        if best_score < -900.0:
            best_heading = 45.0 * self.choose_turn_direction()
            best_clearance = 0.0
        return best_heading, best_clearance

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def approach(current, target, max_step):
        if target > current:
            return min(target, current + max_step)
        return max(target, current - max_step)

    def command(self, vx=0.0, wz=0.0, smooth=True):
        vx = self.clamp(vx, -self.backup_speed, self.linear_speed)
        wz = self.clamp(wz, -self.turn_speed, self.turn_speed)
        if smooth:
            vx = self.approach(self.last_cmd_vx, vx, self.command_rate_limit)
            wz = self.approach(self.last_cmd_wz, wz, self.angular_rate_limit)
        self.last_cmd_vx = vx
        self.last_cmd_wz = wz
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def start_gmapping_process(self):
        if not self.start_gmapping:
            return None
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        cmd = [
            "rosrun", "gmapping", "slam_gmapping", "scan:=" + scan_topic,
            "_base_frame:=base_link", "_odom_frame:=odom",
            "_map_update_interval:={}".format(rospy.get_param("~gmapping_map_update_interval", 2.0)),
            "_linearUpdate:={}".format(rospy.get_param("~gmapping_linear_update", 0.20)),
            "_angularUpdate:={}".format(rospy.get_param("~gmapping_angular_update", 0.20)),
            "_temporalUpdate:={}".format(rospy.get_param("~gmapping_temporal_update", 2.0)),
            "_particles:={}".format(rospy.get_param("~gmapping_particles", 25)),
            "_xmin:={}".format(rospy.get_param("~gmapping_xmin", -12.0)),
            "_ymin:={}".format(rospy.get_param("~gmapping_ymin", -12.0)),
            "_xmax:={}".format(rospy.get_param("~gmapping_xmax", 12.0)),
            "_ymax:={}".format(rospy.get_param("~gmapping_ymax", 12.0)),
            "_delta:={}".format(rospy.get_param("~gmapping_delta", 0.05)),
        ]
        self.publish_status("starting_gmapping")
        proc = subprocess.Popen(cmd)
        rospy.sleep(self.gmapping_startup_delay)
        return proc

    def save_current_map(self):
        if not self.save_map:
            return
        directory = os.path.dirname(self.map_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        try:
            rospy.wait_for_message("/map", OccupancyGrid, timeout=10.0)
        except Exception as exc:
            rospy.logwarn("No /map before saving: %s", exc)
        self.publish_status("saving_map", path=self.map_path)
        result = subprocess.run(
            ["rosrun", "map_server", "map_saver", "-f", self.map_path],
            capture_output=True, text=True, timeout=25)
        if result.returncode == 0:
            self.publish_status("map_saved", path=self.map_path)
        else:
            self.publish_status("map_save_failed", error=result.stderr.strip().replace(" ", "_"))

    def wait_ready(self):
        self.publish_status("waiting_scan")
        rospy.wait_for_message("/scan", LaserScan, timeout=10.0)
        deadline = time.time() + 2.0
        while self.scan is None and time.time() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.05)

    def wait_map_ready(self):
        self.publish_status("waiting_map")
        try:
            rospy.wait_for_message("/map", OccupancyGrid, timeout=15.0)
        except Exception as exc:
            rospy.logwarn("No /map before exploration: %s", exc)

    def step(self):
        now = time.time()
        if self.scan_stamp <= 0.0 or now - self.scan_stamp > self.scan_timeout:
            self.state = "scan_timeout"
            self.stop_reason = "scan_timeout"
            self.stop_requested = True
            self.stop_motion()
            return

        front = self.sector_min(0, 34)
        front_wide = self.sector_min(0, 76)
        front_predict = self.sector_mean(0, 118, max_range=4.0)
        left_front = self.sector_mean(38, 76, max_range=4.0)
        right_front = self.sector_mean(-38, 76, max_range=4.0)
        left_side = self.sector_min(82, 52, max_range=3.0)
        right_side = self.sector_min(-82, 52, max_range=3.0)

        if self.last_front is None or abs(front - self.last_front) > 0.08:
            self.last_progress_time = now
            self.last_front = front

        if front_wide <= self.stop_clearance:
            self.state = "backup"
            self.state_until = now + 0.8
            self.turn_direction = -self.choose_turn_direction()

        if now < self.state_until:
            if self.state == "backup":
                self.command(-self.backup_speed, self.turn_direction * self.turn_speed * 0.45)
            elif self.state == "turn":
                self.command(0.0, self.turn_direction * self.turn_speed * 0.85)
            return

        if now >= self.heading_hold_until:
            self.target_heading_deg, self.target_clearance = self.choose_open_heading()
            self.heading_hold_until = now + self.heading_hold_sec

        hard_blocked = front < self.min_front_clearance or front_wide <= self.stop_clearance * 1.25
        if hard_blocked:
            self.state = "turn"
            self.turn_direction = 1.0 if self.target_heading_deg >= 0.0 else -1.0
            self.state_until = now + 0.45
            self.command(0.0, self.turn_direction * self.turn_speed * 0.80)
            return

        if now - self.last_progress_time > self.stuck_timeout:
            self.state = "turn"
            self.turn_direction = self.choose_turn_direction()
            self.state_until = now + 0.9
            self.last_progress_time = now
            self.command(0.0, self.turn_direction * self.turn_speed * 0.75)
            return

        risk = 0.0
        if front_predict < self.predict_front_clearance:
            risk = max(risk, (self.predict_front_clearance - front_predict) / self.predict_front_clearance)
        if front < self.slow_front_clearance:
            risk = max(risk, (self.slow_front_clearance - front) / self.slow_front_clearance)
        if self.target_clearance < self.slow_front_clearance:
            risk = max(risk, (self.slow_front_clearance - self.target_clearance) / self.slow_front_clearance)
        risk = self.clamp(risk, 0.0, 1.0)

        if risk > 0.10 and now >= self.turn_hold_until:
            self.turn_direction = 1.0 if self.target_heading_deg >= 0.0 else -1.0
            self.turn_hold_until = now + 1.3

        clearance_balance = (left_front - right_front) / max(left_front + right_front, 0.2)
        side_balance = 0.0
        if left_side < self.side_clearance:
            side_balance -= (self.side_clearance - left_side) / self.side_clearance
        if right_side < self.side_clearance:
            side_balance += (self.side_clearance - right_side) / self.side_clearance
        if left_side < self.corridor_side_clearance and right_side < self.corridor_side_clearance:
            risk = max(risk, 0.65)
        heading_steer = self.clamp(math.radians(self.target_heading_deg) * self.heading_gain, -1.0, 1.0)
        steer = 0.72 * heading_steer + 0.18 * clearance_balance + 0.22 * side_balance
        if risk > 0.10:
            steer += self.turn_direction * (0.12 + 0.36 * risk)

        speed_scale = 1.0 - 0.72 * risk
        if abs(self.target_heading_deg) > 55.0:
            speed_scale *= 0.55
        elif abs(self.target_heading_deg) > 35.0:
            speed_scale *= 0.75
        if abs(steer) > 0.45:
            speed_scale *= 0.78
        vx = self.linear_speed * self.clamp(speed_scale, 0.24, 1.0)
        wz = self.turn_speed * self.clamp(steer, -1.0, 1.0)
        self.state = "gap_seek" if abs(self.target_heading_deg) > 12.0 else (
            "curve_avoid" if risk > 0.10 or abs(steer) > 0.18 else "cruise"
        )
        self.command(vx, wz)

    def run(self):
        self.start_time = time.time()
        gmapping_proc = None
        try:
            self.wait_ready()
            gmapping_proc = self.start_gmapping_process()
            self.wait_map_ready()
            rate = rospy.Rate(20)
            last_status = 0.0
            while not rospy.is_shutdown() and not self.stop_requested:
                elapsed = time.time() - self.start_time
                if elapsed >= self.duration_sec:
                    self.stop_reason = "time_limit"
                    self.publish_status("time_limit")
                    break
                completion = self.completion_reason(elapsed)
                if completion:
                    self.stop_reason = completion
                    self.publish_status(completion)
                    break
                self.step()
                if time.time() - last_status >= 1.0 / self.status_rate_hz:
                    self.publish_status(self.state)
                    last_status = time.time()
                rate.sleep()
        finally:
            if self.stop_requested and not self.stop_reason:
                self.stop_reason = "manual_stop"
            self.stop_motion()
            self.save_current_map()
            if self.stop_reason in ("time_limit", "map_complete"):
                self.end_spin()
            if gmapping_proc:
                gmapping_proc.terminate()
                try:
                    gmapping_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    gmapping_proc.kill()
            self.publish_status("stopped")


if __name__ == "__main__":
    FastAutoMapper().run()
