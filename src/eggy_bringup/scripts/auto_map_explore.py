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
        self.stop_clearance = self.param_float("~stop_clearance", 0.26, 0.12, 0.80)
        self.stuck_timeout = self.param_float("~stuck_timeout", 2.5, 0.5, 10.0)
        self.scan_timeout = self.param_float("~scan_timeout", 1.0, 0.2, 5.0)
        self.command_rate_limit = self.param_float("~command_rate_limit", 0.10, 0.02, 0.40)
        self.angular_rate_limit = self.param_float("~angular_rate_limit", 0.18, 0.04, 0.60)
        self.obstacle_trigger_clearance = self.param_float(
            "~obstacle_trigger_clearance", 0.78, 0.35, 2.00
        )
        self.free_path_clearance = self.param_float("~free_path_clearance", 0.68, 0.30, 2.50)
        self.free_path_max_range = self.param_float("~free_path_max_range", 3.5, 1.0, 8.0)
        self.free_path_angle_range = self.param_float("~free_path_angle_range", 125.0, 45.0, 170.0)
        self.min_gap_span_deg = self.param_float("~min_gap_span_deg", 18.0, 5.0, 60.0)
        self.turn_to_gap_tolerance = self.param_float("~turn_to_gap_tolerance", 10.0, 3.0, 25.0)
        self.turn_in_place_heading = self.param_float("~turn_in_place_heading", 28.0, 10.0, 70.0)
        self.heading_gain = self.param_float("~heading_gain", 1.20, 0.30, 2.50)
        self.cruise_centering_gain = self.param_float("~cruise_centering_gain", 0.18, 0.0, 0.80)
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
        self.target_heading_deg = 0.0
        self.target_clearance = 0.0
        self.target_gap_span_deg = 0.0
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
            "target_gap_span_deg": round(self.target_gap_span_deg, 1),
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

    def scan_points(self, max_range):
        if self.scan is None or self.scan.angle_increment == 0.0:
            return []
        points = []
        limit = min(self.scan.range_max, max_range)
        for idx, value in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + idx * self.scan.angle_increment
            angle_deg = math.degrees(angle)
            if abs(angle_deg) > self.free_path_angle_range:
                continue
            if math.isfinite(value):
                if value < self.scan.range_min:
                    distance = 0.0
                else:
                    distance = min(value, limit)
            else:
                distance = limit
            points.append((angle_deg, distance))
        return points

    def choose_longest_free_path(self):
        points = self.scan_points(self.free_path_max_range)
        best = None
        current = []
        for point in points:
            if point[1] >= self.free_path_clearance:
                current.append(point)
            elif current:
                best = self.better_gap(best, current)
                current = []
        if current:
            best = self.better_gap(best, current)

        if not best:
            return 45.0 * self.choose_turn_direction(), 0.0, 0.0
        first_angle = best[0][0]
        last_angle = best[-1][0]
        span = max(0.0, last_angle - first_angle)
        center = (first_angle + last_angle) * 0.5
        mean_clearance = sum(point[1] for point in best) / float(len(best))
        return center, mean_clearance, span

    def better_gap(self, best, candidate):
        if not candidate:
            return best
        if best is None:
            return candidate
        cand_span = candidate[-1][0] - candidate[0][0]
        best_span = best[-1][0] - best[0][0]
        cand_mean = sum(point[1] for point in candidate) / float(len(candidate))
        best_mean = sum(point[1] for point in best) / float(len(best))
        cand_center = abs((candidate[0][0] + candidate[-1][0]) * 0.5)
        best_center = abs((best[0][0] + best[-1][0]) * 0.5)
        cand_score = 3.0 * cand_span + 14.0 * cand_mean - 0.20 * cand_center
        best_score = 3.0 * best_span + 14.0 * best_mean - 0.20 * best_center
        return candidate if cand_score > best_score else best

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

        if now - self.last_progress_time > self.stuck_timeout:
            self.state = "turn"
            self.turn_direction = self.choose_turn_direction()
            self.state_until = now + 0.9
            self.last_progress_time = now
            self.command(0.0, self.turn_direction * self.turn_speed * 0.75)
            return

        front_blocked = (
            front < self.min_front_clearance
            or front_wide <= self.obstacle_trigger_clearance
            or front_predict <= self.obstacle_trigger_clearance
        )
        if not front_blocked:
            self.target_heading_deg = 0.0
            self.target_clearance = front_predict
            self.target_gap_span_deg = 0.0
            clearance_balance = (left_front - right_front) / max(left_front + right_front, 0.2)
            side_balance = 0.0
            if left_side < self.side_clearance:
                side_balance -= (self.side_clearance - left_side) / self.side_clearance
            if right_side < self.side_clearance:
                side_balance += (self.side_clearance - right_side) / self.side_clearance
            steer = self.clamp(
                self.cruise_centering_gain * clearance_balance + 0.16 * side_balance,
                -0.35,
                0.35,
            )
            speed_scale = 1.0
            if front_predict < self.predict_front_clearance:
                speed_scale = self.clamp(
                    0.55 + 0.45 * front_predict / self.predict_front_clearance,
                    0.45,
                    1.0,
                )
            self.state = "cruise"
            self.command(self.linear_speed * speed_scale, self.turn_speed * steer)
            return

        self.target_heading_deg, self.target_clearance, self.target_gap_span_deg = (
            self.choose_longest_free_path()
        )
        if self.target_gap_span_deg < self.min_gap_span_deg:
            self.state = "backup"
            self.state_until = now + 0.7
            self.turn_direction = self.choose_turn_direction()
            self.command(-self.backup_speed, self.turn_direction * self.turn_speed * 0.45)
            return

        self.turn_direction = 1.0 if self.target_heading_deg >= 0.0 else -1.0
        heading_abs = abs(self.target_heading_deg)
        heading_steer = self.clamp(
            math.radians(self.target_heading_deg) * self.heading_gain,
            -1.0,
            1.0,
        )
        if heading_abs > self.turn_in_place_heading:
            self.state = "turn_to_longest_gap"
            self.command(0.0, self.turn_speed * heading_steer)
            return

        speed_scale = self.clamp(
            self.target_clearance / max(self.free_path_max_range, 0.1),
            0.35,
            0.75,
        )
        if heading_abs > self.turn_to_gap_tolerance:
            speed_scale *= 0.55
        self.state = "enter_longest_gap"
        self.command(self.linear_speed * speed_scale, self.turn_speed * heading_steer)

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
