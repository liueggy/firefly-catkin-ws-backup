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
        self.slow_front_clearance = self.param_float("~slow_front_clearance", 0.75, 0.30, 2.00)
        self.side_clearance = self.param_float("~side_clearance", 0.32, 0.15, 1.00)
        self.stop_clearance = self.param_float("~stop_clearance", 0.26, 0.12, 0.80)
        self.stuck_timeout = self.param_float("~stuck_timeout", 2.5, 0.5, 10.0)
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
        cmd = Twist()
        for _ in range(6):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.03)

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

    def choose_turn_direction(self):
        left = self.sector_min(65, 80)
        right = self.sector_min(-65, 80)
        return 1.0 if left >= right else -1.0

    def command(self, vx=0.0, wz=0.0):
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

    def step(self):
        now = time.time()
        front = self.sector_min(0, 34)
        front_wide = self.sector_min(0, 70)
        left = self.sector_min(65, 80)
        right = self.sector_min(-65, 80)

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
                self.command(0.0, self.turn_direction * self.turn_speed)
            return

        if front < self.min_front_clearance or left < self.side_clearance or right < self.side_clearance:
            self.state = "turn"
            self.turn_direction = self.choose_turn_direction()
            self.state_until = now + 0.8
            self.command(0.0, self.turn_direction * self.turn_speed)
            return

        if now - self.last_progress_time > self.stuck_timeout:
            self.state = "turn"
            self.turn_direction = self.choose_turn_direction()
            self.state_until = now + 1.2
            self.last_progress_time = now
            self.command(0.0, self.turn_direction * self.turn_speed)
            return

        self.state = "forward"
        if front < self.slow_front_clearance:
            vx = self.linear_speed * 0.55
        else:
            vx = self.linear_speed
        balance = max(-1.0, min(1.0, (left - right) / max(left + right, 0.1)))
        self.command(vx, 0.35 * balance)

    def run(self):
        self.start_time = time.time()
        gmapping_proc = None
        try:
            self.wait_ready()
            gmapping_proc = self.start_gmapping_process()
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
            if gmapping_proc:
                gmapping_proc.terminate()
                try:
                    gmapping_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    gmapping_proc.kill()
            self.publish_status("stopped")


if __name__ == "__main__":
    FastAutoMapper().run()
