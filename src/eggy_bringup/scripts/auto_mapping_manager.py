#!/usr/bin/env python3
"""Frontier-driven automatic mapping manager for Eggy ROS1."""

import json
import math
import os
import shutil
import subprocess
import tempfile
import threading
import time
import uuid
from datetime import datetime

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool, Empty, Float32, String, UInt8

from eggy_bringup.auto_mapping_core import (
    completion_decision,
    extract_frontier_clusters,
    rank_frontiers,
    validate_mapping_request,
)


TERMINAL_STATES = {"completed", "cancelled", "aborted", "rejected"}
ACTIVE_STATES = {
    "preflight", "planning", "navigating", "observing", "returning",
    "final_scan", "saving",
}


class AutoMappingManager(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.state = "idle"
        self.message = "automatic mapping ready"
        self.request_id = ""
        self.options = {}
        self.started_at = 0.0
        self.state_started_at = time.time()
        self.stop_reason = ""
        self.result_sent = False
        self.result_details = {}

        self.latest_map = None
        self.map_stamp = 0.0
        self.scan_stamp = 0.0
        self.odom_stamp = 0.0
        self.battery_stamp = 0.0
        self.battery_voltage = None
        self.known_cells = 0
        self.last_known_cells = 0
        self.last_map_growth_time = time.time()
        self.emergency_stop = False
        self.base_stop = False
        self.safety_status = {}
        self.control_status = {}

        self.home_pose = None
        self.current_goal = None
        self.current_goal_started = 0.0
        self.visited_points = []
        self.failed_points = []
        self.frontier_count = 0
        self.reachable_frontier_count = 0
        self.no_frontier_cycles = 0
        self.recovery_count = 0
        self.completed_reason = ""
        self.observe_until = 0.0
        self.final_scan_deadline = 0.0
        self.last_plan_time = 0.0
        self.last_status_time = 0.0

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.map_timeout = float(rospy.get_param("~map_timeout", 3.0))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.8))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.8))
        self.preflight_timeout = float(rospy.get_param("~preflight_timeout", 12.0))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.observe_sec = float(rospy.get_param("~observe_sec", 1.5))
        self.planning_period = float(rospy.get_param("~planning_period", 1.0))
        self.min_known_cells = int(rospy.get_param("~min_known_cells", 800))
        self.min_elapsed_sec = float(rospy.get_param("~min_elapsed_sec", 60.0))
        self.map_stable_sec = float(rospy.get_param("~map_stable_sec", 30.0))
        self.required_no_frontier_cycles = int(rospy.get_param("~required_no_frontier_cycles", 3))
        self.max_candidate_plans = int(rospy.get_param("~max_candidate_plans", 6))
        self.min_goal_distance = float(rospy.get_param("~min_goal_distance", 0.45))
        self.final_scan_enabled = bool(rospy.get_param("~final_scan_enabled", True))
        self.final_scan_speed = float(rospy.get_param("~final_scan_speed", 0.28))
        self.map_library_root = rospy.get_param(
            "~map_library_root", "/root/catkin_ws/maps/library")
        self.require_battery = bool(rospy.get_param("~require_battery", True))
        self.min_start_voltage = float(rospy.get_param("~min_start_voltage", 11.0))
        self.critical_voltage = float(rospy.get_param("~critical_voltage", 10.5))

        self.status_pub = rospy.Publisher(
            "/eggy/auto_mapping/status", String, queue_size=1, latch=True)
        self.legacy_status_pub = rospy.Publisher(
            "/auto_explore/status", String, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher(
            "/eggy/auto_mapping/result", String, queue_size=1, latch=True)
        self.frontier_pub = rospy.Publisher(
            "/eggy/auto_mapping/frontiers", PoseArray, queue_size=1)
        self.goal_pub = rospy.Publisher(
            "/eggy/auto_mapping/current_goal", PoseStamped, queue_size=1, latch=True)
        self.safety_active_pub = rospy.Publisher(
            "/eggy/auto_mapping/safety_active", Bool, queue_size=1, latch=True)
        self.safety_config_pub = rospy.Publisher(
            "/eggy/auto_mapping/safety_config", String, queue_size=1, latch=True)
        self.heartbeat_pub = rospy.Publisher(
            "/eggy/auto_mapping/heartbeat", Empty, queue_size=1)
        self.raw_cmd_pub = rospy.Publisher("/cmd_vel/mapping_raw", Twist, queue_size=1)

        rospy.Subscriber(
            "/eggy/auto_mapping/request", String, self.request_cb, queue_size=10)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        # Only the arrival timestamp is needed here. AnyMsg avoids decoding the
        # full ranges/intensities arrays in a second Python process.
        rospy.Subscriber("/scan", rospy.AnyMsg, self.scan_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/battery/voltage", Float32, self.battery_cb, queue_size=1)
        rospy.Subscriber("/eggy/emergency_stop", Bool, self.emergency_cb, queue_size=1)
        rospy.Subscriber("/base/flag_stop", UInt8, self.base_stop_cb, queue_size=1)
        rospy.Subscriber(
            "/eggy/auto_mapping/safety_status", String, self.safety_status_cb, queue_size=1)
        rospy.Subscriber(
            "/eggy/cmd_vel/control", String, self.control_status_cb, queue_size=1)

        self.tf_listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.tick)
        rospy.on_shutdown(self.shutdown)
        self.safety_active_pub.publish(Bool(False))
        self.publish_status(force=True)

    @staticmethod
    def yaw_quaternion(yaw):
        return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

    def robot_pose(self):
        try:
            trans, rot = self.tf_listener.lookupTransform(
                self.map_frame, self.base_frame, rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return (float(trans[0]), float(trans[1]), float(yaw))
        except Exception:
            return None

    def map_cb(self, msg):
        now = time.time()
        with self.lock:
            active = self.state in ACTIVE_STATES
            self.latest_map = msg
            self.map_stamp = now
        if not active:
            return
        known = sum(1 for value in msg.data if value >= 0)
        with self.lock:
            if known >= self.last_known_cells + 30:
                self.last_map_growth_time = now
            self.last_known_cells = max(self.last_known_cells, known)
            self.known_cells = known

    def scan_cb(self, _msg):
        with self.lock:
            self.scan_stamp = time.time()

    def odom_cb(self, _msg):
        with self.lock:
            self.odom_stamp = time.time()

    def battery_cb(self, msg):
        with self.lock:
            self.battery_voltage = float(msg.data)
            self.battery_stamp = time.time()

    def emergency_cb(self, msg):
        if msg.data:
            self.abort("emergency_stop", "急停已经锁定")
        with self.lock:
            self.emergency_stop = bool(msg.data)

    def base_stop_cb(self, msg):
        if msg.data:
            self.abort("base_stop", "底盘停车信号已触发")
        with self.lock:
            self.base_stop = bool(msg.data)

    def safety_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.safety_status = data if isinstance(data, dict) else {}

    def control_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.control_status = data if isinstance(data, dict) else {}
            should_pause = self.state in ACTIVE_STATES and data.get("active_source") == "manual"
        if should_pause:
            self.pause("manual_takeover", "检测到人工接管，自动建图已暂停")

    def request_cb(self, msg):
        try:
            payload = validate_mapping_request(json.loads(msg.data))
            if not payload["request_id"]:
                raise ValueError("request_id is required")
        except Exception as exc:
            self.publish_transient("bad_request", str(exc))
            return
        command = payload["command"]
        if command == "start":
            self.start(payload)
        elif command == "pause":
            if self.matches_request(payload):
                self.pause("requested", "自动建图已暂停")
        elif command == "resume":
            if self.matches_request(payload):
                self.resume()
        elif command == "cancel":
            if self.matches_request(payload):
                self.cancel("requested")
        else:
            self.publish_status(force=True)

    def matches_request(self, payload):
        with self.lock:
            matches = payload["request_id"] == self.request_id
        if not matches:
            self.publish_transient("request_id_mismatch", "request_id does not own active mapping")
        return matches

    def start(self, payload):
        with self.lock:
            if self.state in ACTIVE_STATES or self.state == "paused":
                self.publish_transient("busy", "automatic mapping already active")
                return
            self.request_id = payload["request_id"]
            self.options = payload["options"]
            self.started_at = time.time()
            self.state_started_at = self.started_at
            self.state = "preflight"
            self.message = "正在执行自动建图启动检查"
            self.stop_reason = ""
            self.result_sent = False
            self.result_details = {}
            self.home_pose = None
            self.current_goal = None
            self.visited_points = []
            self.failed_points = []
            self.frontier_count = 0
            self.reachable_frontier_count = 0
            self.no_frontier_cycles = 0
            self.recovery_count = 0
            self.completed_reason = ""
            self.last_plan_time = 0.0
            self.last_map_growth_time = time.time()
        self.safety_active_pub.publish(Bool(False))
        self.safety_config_pub.publish(String(json.dumps({
            "schema_version": 1,
            "max_linear_speed": self.options["max_linear_speed"],
        })))
        self.publish_status(force=True)

    def pause(self, reason, message):
        with self.lock:
            if self.state not in ACTIVE_STATES:
                return
            self.state = "paused"
            self.state_started_at = time.time()
            self.stop_reason = reason
            self.message = message
        self.move_base.cancel_all_goals()
        self.stop_raw_motion()
        self.safety_active_pub.publish(Bool(False))
        self.publish_status(force=True)

    def resume(self):
        with self.lock:
            if self.state != "paused" or self.emergency_stop or self.base_stop:
                self.publish_transient("resume_rejected", "当前安全状态不允许继续")
                return
            self.state = "preflight"
            self.state_started_at = time.time()
            self.stop_reason = ""
            self.message = "正在重新检查建图条件"
        self.publish_status(force=True)

    def cancel(self, reason):
        with self.lock:
            if self.state not in ACTIVE_STATES and self.state != "paused":
                return
            save_draft = bool(self.options.get("save_draft_on_abort", True))
            self.stop_reason = reason
        self.move_base.cancel_all_goals()
        self.stop_raw_motion()
        self.safety_active_pub.publish(Bool(False))
        if save_draft and self.latest_map is not None:
            self.finish_with_save("cancelled", "用户结束自动建图", "draft")
        else:
            self.finish("cancelled", "用户结束自动建图", {})

    def abort(self, reason, message):
        with self.lock:
            if self.state not in ACTIVE_STATES and self.state != "paused":
                return
            self.stop_reason = reason
        self.move_base.cancel_all_goals()
        self.stop_raw_motion()
        self.safety_active_pub.publish(Bool(False))
        self.finish("aborted", message, {"reason": reason})

    def preflight(self):
        now = time.time()
        with self.lock:
            failures = []
            if self.emergency_stop:
                failures.append("emergency_stop")
            if self.base_stop:
                failures.append("base_stop")
            if now - self.scan_stamp > self.scan_timeout:
                failures.append("scan_stale")
            if now - self.odom_stamp > self.odom_timeout:
                failures.append("odom_stale")
            if self.latest_map is None or now - self.map_stamp > self.map_timeout:
                failures.append("map_stale")
            if self.require_battery:
                if self.battery_voltage is None or now - self.battery_stamp > 3.0:
                    failures.append("battery_stale")
                elif self.battery_voltage < self.min_start_voltage:
                    failures.append("battery_low")
        if self.robot_pose() is None:
            failures.append("tf_unavailable")
        if not self.move_base.wait_for_server(rospy.Duration(0.02)):
            failures.append("move_base_unavailable")
        try:
            rospy.wait_for_service("/move_base/make_plan", timeout=0.05)
        except Exception:
            failures.append("make_plan_unavailable")
        return sorted(set(failures))

    def active_health_failure(self):
        now = time.time()
        with self.lock:
            if self.emergency_stop:
                return "emergency_stop"
            if self.base_stop:
                return "base_stop"
            if now - self.scan_stamp > self.scan_timeout:
                return "scan_stale"
            if now - self.odom_stamp > self.odom_timeout:
                return "odom_stale"
            if now - self.map_stamp > self.map_timeout:
                return "map_stale"
            if self.battery_voltage is not None and self.battery_voltage < self.critical_voltage:
                return "battery_critical"
        return ""

    def plan_path(self, robot_pose, candidate):
        start = self.pose_stamped(robot_pose[0], robot_pose[1], robot_pose[2])
        yaw = math.atan2(candidate["y"] - robot_pose[1], candidate["x"] - robot_pose[0])
        goal = self.pose_stamped(candidate["x"], candidate["y"], yaw)
        try:
            response = self.make_plan(start=start, goal=goal, tolerance=0.25)
        except Exception:
            return None
        poses = list(response.plan.poses)
        if len(poses) < 2:
            return None
        length = 0.0
        for previous, current in zip(poses, poses[1:]):
            length += math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y)
        return max(length, self.min_goal_distance)

    def select_frontier(self):
        with self.lock:
            msg = self.latest_map
            min_cluster = int(self.options.get("min_frontier_cells", 8))
            visited = list(self.visited_points)
            failed = list(self.failed_points)
        pose = self.robot_pose()
        if msg is None or pose is None:
            return None
        info = msg.info
        clusters = extract_frontier_clusters(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            min_cluster_cells=min_cluster,
        )
        clusters = [item for item in clusters if math.hypot(
            item["x"] - pose[0], item["y"] - pose[1]) >= self.min_goal_distance]
        ranked = rank_frontiers(clusters, pose[:2], visited, failed)
        self.publish_frontiers(ranked)
        reachable = []
        for candidate in ranked[:max(1, self.max_candidate_plans)]:
            path_length = self.plan_path(pose, candidate)
            if path_length is None:
                continue
            item = dict(candidate)
            item["path_length"] = path_length
            reachable.append(item)
        reachable = rank_frontiers(reachable, pose[:2], visited, failed)
        with self.lock:
            self.frontier_count = len(ranked)
            self.reachable_frontier_count = len(reachable)
        return reachable[0] if reachable else None

    def send_frontier_goal(self, candidate, returning=False):
        pose = self.robot_pose()
        if pose is None:
            self.abort("tf_unavailable", "机器人位姿不可用")
            return
        yaw = candidate.get("yaw", math.atan2(candidate["y"] - pose[1], candidate["x"] - pose[0]))
        goal_pose = self.pose_stamped(candidate["x"], candidate["y"], yaw)
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose
        self.move_base.send_goal(goal)
        with self.lock:
            self.current_goal = dict(candidate, yaw=yaw)
            self.current_goal_started = time.time()
            self.state = "returning" if returning else "navigating"
            self.state_started_at = time.time()
            self.message = "正在返回起点" if returning else "正在前往新的探索区域"
        self.goal_pub.publish(goal_pose)
        self.publish_status(force=True)

    def pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = self.yaw_quaternion(float(yaw))
        return pose

    def publish_frontiers(self, candidates):
        array = PoseArray()
        array.header.frame_id = self.map_frame
        array.header.stamp = rospy.Time.now()
        for item in candidates[:40]:
            pose = Pose()
            pose.position.x = item["x"]
            pose.position.y = item["y"]
            pose.orientation.w = 1.0
            array.poses.append(pose)
        self.frontier_pub.publish(array)

    def begin_completion(self, reason):
        with self.lock:
            self.completed_reason = reason
            return_home = bool(self.options.get("return_home", True))
            home = self.home_pose
        if return_home and home is not None:
            self.send_frontier_goal({"x": home[0], "y": home[1], "yaw": home[2]}, returning=True)
            return
        self.begin_final_scan()

    def begin_final_scan(self):
        self.move_base.cancel_all_goals()
        if self.final_scan_enabled and self.final_scan_speed > 0.05:
            with self.lock:
                self.state = "final_scan"
                self.state_started_at = time.time()
                self.final_scan_deadline = time.time() + 2.0 * math.pi / self.final_scan_speed
                self.message = "正在进行最终一圈慢速扫描"
        else:
            self.finish_with_save("completed", "自动建图完成", "qualified")

    def finish_with_save(self, terminal_state, message, quality):
        with self.lock:
            self.state = "saving"
            self.message = "正在保存地图"
        self.publish_status(force=True)
        saved = self.save_map(quality)
        if saved.get("ok"):
            self.finish(terminal_state, message, saved)
        else:
            self.finish("aborted", "地图保存失败", saved)

    def save_map(self, quality):
        os.makedirs(self.map_library_root, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_id = "auto_%s_%s" % (timestamp, self.request_id.replace("-", "")[:8])
        staging = tempfile.mkdtemp(prefix=".auto-staging-", dir=self.map_library_root)
        try:
            prefix = os.path.join(staging, "map")
            process = subprocess.run(
                ["rosrun", "map_server", "map_saver", "-f", prefix],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30)
            yaml_path = prefix + ".yaml"
            pgm_path = prefix + ".pgm"
            if process.returncode != 0 or not os.path.exists(yaml_path) or not os.path.exists(pgm_path):
                raise RuntimeError((process.stdout or "") + (process.stderr or ""))
            metadata = {
                "schema_version": 1,
                "map_id": map_id,
                "source": "automatic_mapping",
                "quality": quality,
                "request_id": self.request_id,
                "created_at": timestamp,
                "known_cells": self.known_cells,
                "visited_frontiers": len(self.visited_points),
                "failed_frontiers": len(self.failed_points),
                "completion_reason": self.completed_reason or self.stop_reason,
            }
            with open(os.path.join(staging, "metadata.json"), "w", encoding="utf-8") as stream:
                json.dump(metadata, stream, ensure_ascii=False, indent=2)
            destination = os.path.join(self.map_library_root, map_id)
            os.replace(staging, destination)
            staging = ""
            return {"ok": True, "map_id": map_id,
                    "map_yaml": os.path.join(destination, "map.yaml"), "quality": quality}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        finally:
            if staging and os.path.isdir(staging):
                shutil.rmtree(staging, ignore_errors=True)

    def finish(self, terminal_state, message, details):
        self.move_base.cancel_all_goals()
        self.stop_raw_motion()
        self.safety_active_pub.publish(Bool(False))
        with self.lock:
            self.state = terminal_state
            self.state_started_at = time.time()
            self.message = message
            self.result_details = dict(details) if isinstance(details, dict) else {}
        self.publish_status(force=True)
        self.publish_result(details)

    def publish_result(self, details):
        with self.lock:
            if self.result_sent:
                return
            self.result_sent = True
            payload = self.status_payload()
            payload["ok"] = self.state == "completed"
            payload["details"] = details
        self.result_pub.publish(String(json.dumps(payload, ensure_ascii=False)))

    def tick(self, _event):
        now = time.time()
        with self.lock:
            state = self.state
        if state in ACTIVE_STATES:
            self.heartbeat_pub.publish(Empty())
        if state in ("idle", "paused") or state in TERMINAL_STATES:
            self.publish_status()
            return

        if state == "preflight":
            failures = self.preflight()
            if not failures:
                pose = self.robot_pose()
                with self.lock:
                    if self.home_pose is None:
                        self.home_pose = pose
                    self.state = "planning"
                    self.state_started_at = now
                    self.message = "启动检查通过，正在选择探索区域"
                self.safety_active_pub.publish(Bool(True))
            elif now - self.state_started_at > self.preflight_timeout:
                self.abort("preflight_failed", "启动检查未通过：" + ",".join(failures))
                return
            else:
                with self.lock:
                    self.message = "等待启动条件：" + ",".join(failures)
        else:
            failure = self.active_health_failure()
            if failure:
                self.abort(failure, "自动建图安全停止：" + failure)
                return

        if self.options and now - self.started_at >= self.options.get("max_duration_sec", 900.0):
            self.completed_reason = "time_limit"
            self.begin_completion("time_limit")
            return

        if state == "planning" and now - self.last_plan_time >= self.planning_period:
            self.last_plan_time = now
            candidate = self.select_frontier()
            if candidate is None:
                with self.lock:
                    self.no_frontier_cycles += 1
                    stable = now - self.last_map_growth_time
                reason = completion_decision(
                    now - self.started_at, self.known_cells, self.no_frontier_cycles, stable,
                    self.min_elapsed_sec, self.min_known_cells,
                    self.required_no_frontier_cycles, self.map_stable_sec)
                if reason:
                    self.begin_completion(reason)
                    return
                with self.lock:
                    self.message = "暂未找到可达前沿，正在重新评估"
            else:
                with self.lock:
                    self.no_frontier_cycles = 0
                self.send_frontier_goal(candidate)

        elif state in ("navigating", "returning"):
            action_state = self.move_base.get_state()
            if action_state == GoalStatus.SUCCEEDED:
                with self.lock:
                    goal = self.current_goal
                    if state == "navigating" and goal:
                        self.visited_points.append((goal["x"], goal["y"]))
                    self.current_goal = None
                if state == "returning":
                    self.begin_final_scan()
                else:
                    with self.lock:
                        self.state = "observing"
                        self.observe_until = now + self.observe_sec
                        self.message = "已到达探索点，等待地图稳定"
            elif action_state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST):
                with self.lock:
                    goal = self.current_goal
                    if goal:
                        self.failed_points.append((goal["x"], goal["y"]))
                    self.recovery_count += 1
                    self.current_goal = None
                    self.state = "planning"
                    self.message = "目标不可达，已重新规划"
            elif now - self.current_goal_started > self.goal_timeout:
                self.move_base.cancel_all_goals()
                with self.lock:
                    goal = self.current_goal
                    if goal:
                        self.failed_points.append((goal["x"], goal["y"]))
                    self.recovery_count += 1
                    self.current_goal = None
                    self.state = "planning"
                    self.message = "目标执行超时，已重新规划"

        elif state == "observing" and now >= self.observe_until:
            with self.lock:
                self.state = "planning"
                self.message = "正在选择下一个探索区域"

        elif state == "final_scan":
            if now < self.final_scan_deadline:
                cmd = Twist()
                cmd.angular.z = self.final_scan_speed
                self.raw_cmd_pub.publish(cmd)
            else:
                self.stop_raw_motion()
                self.finish_with_save("completed", "自动建图完成", "qualified")
                return

        self.publish_status()

    def stop_raw_motion(self):
        for _ in range(3):
            self.raw_cmd_pub.publish(Twist())

    def status_payload(self):
        now = time.time()
        with self.lock:
            goal = dict(self.current_goal) if self.current_goal else None
            return {
                "schema_version": 1,
                "stamp": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
                "request_id": self.request_id,
                "state": self.state,
                "stage": self.state,
                "busy": self.state in ACTIVE_STATES or self.state == "paused",
                "message": self.message,
                "options": dict(self.options),
                "stop_reason": self.stop_reason,
                "elapsed_sec": round(now - self.started_at, 1) if self.started_at else 0.0,
                "known_cells": self.known_cells,
                "frontier_cells": self.frontier_count,
                "reachable_frontiers": self.reachable_frontier_count,
                "visited_frontiers": len(self.visited_points),
                "failed_frontiers": len(self.failed_points),
                "recovery_count": self.recovery_count,
                "current_goal": goal,
                "battery_voltage": self.battery_voltage,
                "ages": {
                    "map": None if not self.map_stamp else round(now - self.map_stamp, 3),
                    "scan": None if not self.scan_stamp else round(now - self.scan_stamp, 3),
                    "odom": None if not self.odom_stamp else round(now - self.odom_stamp, 3),
                },
                "safety": self.safety_status,
                "control": self.control_status,
                "result": self.result_details,
            }

    def publish_status(self, force=False):
        now = time.time()
        if not force and now - self.last_status_time < 0.5:
            return
        self.last_status_time = now
        encoded = String(json.dumps(self.status_payload(), ensure_ascii=False))
        self.status_pub.publish(encoded)
        self.legacy_status_pub.publish(encoded)

    def publish_transient(self, state, message):
        payload = self.status_payload()
        payload.update({"state": state, "stage": state, "message": message})
        encoded = String(json.dumps(payload, ensure_ascii=False))
        self.status_pub.publish(encoded)
        self.legacy_status_pub.publish(encoded)

    def shutdown(self):
        self.move_base.cancel_all_goals()
        self.stop_raw_motion()
        self.safety_active_pub.publish(Bool(False))


if __name__ == "__main__":
    rospy.init_node("eggy_auto_mapping_manager")
    AutoMappingManager()
    rospy.spin()
