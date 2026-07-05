#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import math
import threading
import time

import rosnode
import rospy
import tf
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String, UInt8
from std_srvs.srv import Empty

try:
    import cv2
    import numpy as np
except Exception:  # pragma: no cover - reported at runtime by preflight
    cv2 = None
    np = None


class AutoRelocalizer:
    def __init__(self):
        self.lock = threading.RLock()
        self.worker = None
        self.cancel_event = threading.Event()
        self.scan_stamp = rospy.Time(0)
        self.map_stamp = rospy.Time(0)
        self.min_scan_range = None
        self.flag_stop = 1
        self.pose = None
        self.current_pose = None
        self.stable_samples = 0
        self.latest_scan = None
        self.map_info = None
        self.map_distance = None

        self.angular_speed = abs(float(rospy.get_param("~angular_speed", 0.28)))
        self.max_angular_speed = abs(float(rospy.get_param("~max_angular_speed", 0.80)))
        self.timeout = float(rospy.get_param("~timeout", 5.0))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 1.0))
        self.map_timeout = float(rospy.get_param("~map_timeout", 10.0))
        self.xy_variance_max = float(rospy.get_param("~xy_variance_max", 0.20))
        self.yaw_variance_max = float(rospy.get_param("~yaw_variance_max", 0.12))
        self.stable_required = int(rospy.get_param("~stable_samples", 8))
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.default_method = str(rospy.get_param("~default_method", "scan_match"))
        self.search_xy_window = float(rospy.get_param("~search_xy_window", 0.85))
        self.search_xy_step = float(rospy.get_param("~search_xy_step", 0.10))
        self.search_yaw_window_deg = float(rospy.get_param("~search_yaw_window_deg", 55.0))
        self.search_yaw_step_deg = float(rospy.get_param("~search_yaw_step_deg", 5.0))
        self.max_scan_points = int(rospy.get_param("~max_scan_points", 240))
        self.min_match_score = float(rospy.get_param("~min_match_score", 0.56))
        self.score_sigma = float(rospy.get_param("~score_sigma", 0.16))

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/eggy/relocalization/status", String, queue_size=10, latch=True)
        self.cancel_nav_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=3)

        rospy.Subscriber("/eggy/relocalization/request", String, self.request_cb, queue_size=2)
        rospy.Subscriber("/eggy/relocalization/cancel", String, self.cancel_cb, queue_size=2)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_cb, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=2)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber("/base/flag_stop", UInt8, self.flag_cb, queue_size=2)
        rospy.on_shutdown(self.stop_motion)
        self.publish("idle", "自动重定位节点已就绪", progress=0, method=self.default_method)

    def requested_angular_speed(self, request):
        if "angular_speed_deg" in request:
            speed = math.radians(abs(float(request.get("angular_speed_deg"))))
        else:
            speed = abs(float(request.get("angular_speed", self.angular_speed)))
        return min(self.max_angular_speed, max(0.12, speed))

    def publish(self, state, message, **extra):
        payload = {
            "state": state,
            "message": message,
            "stamp": rospy.Time.now().to_sec(),
            "dry_run": self.dry_run,
        }
        payload.update(extra)
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))

    def scan_cb(self, msg):
        valid = [value for value in msg.ranges if msg.range_min <= value <= msg.range_max]
        with self.lock:
            self.latest_scan = msg
            self.scan_stamp = rospy.Time.now()
            self.min_scan_range = min(valid) if valid else None

    def map_cb(self, msg):
        if np is None or cv2 is None:
            return
        data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        occupied = data >= 50
        if not occupied.any():
            return
        free_to_obstacle = np.where(occupied, 0, 255).astype(np.uint8)
        distance = cv2.distanceTransform(free_to_obstacle, cv2.DIST_L2, 3) * msg.info.resolution
        with self.lock:
            self.map_info = msg.info
            self.map_distance = distance
            self.map_stamp = rospy.Time.now()

    def flag_cb(self, msg):
        self.flag_stop = int(msg.data)
        if self.flag_stop != 0 and self.worker and self.worker.is_alive():
            self.cancel_event.set()

    def pose_cb(self, msg):
        cov = msg.pose.covariance
        q = msg.pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        pose_quality = (max(float(cov[0]), float(cov[7])), float(cov[35]))
        pose_value = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), float(yaw))
        with self.lock:
            self.pose = pose_quality
            self.current_pose = pose_value
            if pose_quality[0] <= self.xy_variance_max and pose_quality[1] <= self.yaw_variance_max:
                self.stable_samples += 1
            else:
                self.stable_samples = 0

    def request_cb(self, msg):
        try:
            request = json.loads(msg.data) if msg.data.strip() else {}
        except ValueError as exc:
            self.publish("rejected", "请求 JSON 无效", error=str(exc))
            return
        if str(request.get("command", "start")).lower() != "start":
            self.publish("rejected", "仅支持 command=start")
            return
        with self.lock:
            if self.worker and self.worker.is_alive():
                self.publish("busy", "自动重定位正在运行")
                return
            self.cancel_event.clear()
            self.stable_samples = 0
            self.worker = threading.Thread(target=self.run, args=(request,), daemon=True)
            self.worker.start()

    def cancel_cb(self, _msg):
        self.cancel_event.set()
        self.publish("cancelling", "正在取消自动重定位")

    def preflight_common(self):
        if "/amcl" not in set(rosnode.get_node_names()):
            return False, "AMCL 未运行，当前不是静态地图定位模式"
        if self.scan_stamp.is_zero() or (rospy.Time.now() - self.scan_stamp).to_sec() > self.scan_timeout:
            return False, "激光雷达数据超时"
        if self.min_scan_range is None:
            return False, "激光雷达没有有效距离数据"
        return True, ""

    def preflight_scan_match(self):
        ok, reason = self.preflight_common()
        if not ok:
            return False, reason
        if np is None or cv2 is None:
            return False, "缺少 numpy/cv2，无法执行地图匹配"
        with self.lock:
            has_map = self.map_info is not None and self.map_distance is not None
            map_age = (rospy.Time.now() - self.map_stamp).to_sec() if not self.map_stamp.is_zero() else 999.0
            has_pose = self.current_pose is not None
        if not has_map or map_age > self.map_timeout:
            return False, "静态地图数据不可用或超时"
        if not has_pose:
            return False, "尚未收到 AMCL 初始位姿，无法做局部匹配"
        return True, ""

    def preflight_global_spin(self):
        ok, reason = self.preflight_common()
        if not ok:
            return False, reason
        if self.flag_stop != 0:
            return False, "底盘急停或状态未知，拒绝运动"
        if self.min_scan_range < 0.30:
            return False, "周围 0.30 米内存在障碍，拒绝旋转"
        try:
            rospy.wait_for_service("/global_localization", timeout=2.0)
        except rospy.ROSException:
            return False, "/global_localization 服务不可用"
        return True, ""

    def stop_motion(self):
        zero = Twist()
        for _ in range(5):
            self.cmd_pub.publish(zero)
            rospy.sleep(0.03)

    def run(self, request):
        method = str(request.get("method", self.default_method)).strip().lower()
        if method in ("scan", "scan_match", "map_match", "local_match"):
            self.run_scan_match(request)
        else:
            self.run_global_spin(request)

    def run_scan_match(self, request):
        started = time.time()
        ok, reason = self.preflight_scan_match()
        if not ok:
            self.publish("rejected", reason, progress=0, method="scan_match")
            return
        timeout = min(8.0, max(1.0, float(request.get("timeout", self.timeout))))
        self.publish("matching", "正在匹配当前雷达轮廓与静态地图", progress=10, method="scan_match")
        if self.dry_run:
            self.publish("dry_run_complete", "Dry-run 完成：雷达、地图和 AMCL 数据正常", progress=100, method="scan_match")
            return
        best = self.find_best_pose(timeout, request)
        if self.cancel_event.is_set():
            self.publish("cancelled", "自动重定位已取消", method="scan_match")
            return
        if not best or best[3] < self.min_match_score:
            score = round(best[3], 3) if best else 0.0
            self.publish("failed", "匹配置信度不足，请手动调整", progress=100, method="scan_match", score=score)
            return
        x, y, yaw, score, points_used = best
        self.publish("applying", "匹配成功，正在写入 AMCL 初始位姿", progress=85,
                     method="scan_match", score=round(score, 3), x=round(x, 3), y=round(y, 3), yaw=round(yaw, 4))
        self.publish_initial_pose(x, y, yaw)
        elapsed = time.time() - started
        self.publish("success", "雷达轮廓匹配重定位成功", progress=100, method="scan_match",
                     elapsed_sec=round(elapsed, 3), score=round(score, 3), points_used=points_used,
                     pose={"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4)})

    def scan_points(self, scan):
        points = []
        angle = scan.angle_min
        stride = max(1, int(math.ceil(len(scan.ranges) / float(max(1, self.max_scan_points)))))
        for index, distance in enumerate(scan.ranges):
            if index % stride != 0:
                angle += scan.angle_increment
                continue
            if scan.range_min <= distance <= scan.range_max:
                points.append((distance * math.cos(angle), distance * math.sin(angle)))
            angle += scan.angle_increment
        return points

    def find_best_pose(self, timeout, request):
        with self.lock:
            scan = self.latest_scan
            info = self.map_info
            distance = None if self.map_distance is None else self.map_distance.copy()
            seed = self.current_pose
        if scan is None or info is None or distance is None or seed is None:
            return None
        points = self.scan_points(scan)
        if len(points) < 20:
            return None
        deadline = time.time() + timeout
        xy_window = float(request.get("xy_window", self.search_xy_window))
        xy_step = max(0.03, float(request.get("xy_step", self.search_xy_step)))
        yaw_window = math.radians(float(request.get("yaw_window_deg", self.search_yaw_window_deg)))
        yaw_step = math.radians(max(1.0, float(request.get("yaw_step_deg", self.search_yaw_step_deg))))
        best = None
        passes = [(xy_step, yaw_step), (max(0.03, xy_step * 0.5), max(math.radians(1.0), yaw_step * 0.5))]
        centers = [seed]
        for pass_index, (step_xy, step_yaw) in enumerate(passes):
            next_centers = []
            for cx, cy, cyaw in centers:
                xs = self.frange(cx - xy_window, cx + xy_window, step_xy)
                ys = self.frange(cy - xy_window, cy + xy_window, step_xy)
                yaws = self.frange(cyaw - yaw_window, cyaw + yaw_window, step_yaw)
                for yaw in yaws:
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    for x in xs:
                        if time.time() >= deadline or self.cancel_event.is_set():
                            return best
                        for y in ys:
                            score = self.score_pose(points, x, y, cos_yaw, sin_yaw, info, distance)
                            if best is None or score > best[3]:
                                best = (x, y, self.norm_angle(yaw), score, len(points))
            if best:
                next_centers.append((best[0], best[1], best[2]))
            centers = next_centers or centers
            xy_window = max(step_xy * 1.5, 0.08)
            yaw_window = max(step_yaw * 2.0, math.radians(3.0))
            self.publish("matching", "雷达地图匹配中", progress=35 + pass_index * 25,
                         method="scan_match", score=round(best[3], 3) if best else 0.0)
        return best

    @staticmethod
    def frange(start, stop, step):
        values = []
        value = start
        while value <= stop + step * 0.5:
            values.append(value)
            value += step
        return values

    @staticmethod
    def norm_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def score_pose(self, points, x, y, cos_yaw, sin_yaw, info, distance):
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution
        total = 0.0
        used = 0
        out = 0
        for px, py in points:
            wx = x + cos_yaw * px - sin_yaw * py
            wy = y + sin_yaw * px + cos_yaw * py
            col = int((wx - origin_x) / res)
            row = int((wy - origin_y) / res)
            if row < 0 or row >= info.height or col < 0 or col >= info.width:
                out += 1
                continue
            d = float(distance[row, col])
            total += math.exp(-d / max(0.03, self.score_sigma))
            used += 1
        if used < max(10, len(points) * 0.25):
            return 0.0
        return (total / used) * (used / float(len(points))) - 0.15 * (out / float(len(points)))

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
        msg.pose.pose = Pose()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.04
        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            rospy.sleep(0.05)

    def run_global_spin(self, request):
        ok, reason = self.preflight_global_spin()
        if not ok:
            self.stop_motion()
            self.publish("rejected", reason, progress=0, method="global_spin")
            return
        timeout = min(90.0, max(10.0, float(request.get("timeout", 35.0))))
        speed = self.requested_angular_speed(request)
        self.publish("preflight_ok", "安全检查通过", progress=5,
                     method="global_spin", angular_speed=round(speed, 3),
                     angular_speed_deg=round(math.degrees(speed), 2))
        if self.dry_run:
            self.publish("dry_run_complete", "Dry-run 完成：AMCL、雷达、急停和服务均正常", progress=100, method="global_spin")
            return
        self.cancel_nav_pub.publish(GoalID())
        self.stop_motion()
        try:
            rospy.ServiceProxy("/global_localization", Empty)()
        except rospy.ServiceException as exc:
            self.publish("failed", "全局粒子初始化失败", method="global_spin", error=str(exc))
            return
        with self.lock:
            self.stable_samples = 0
            self.pose = None
        self.publish("rotating", "正在旋转扫描并等待 AMCL 收敛", progress=10, method="global_spin")
        started = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() - started < timeout:
            if self.cancel_event.is_set() or self.flag_stop != 0:
                self.stop_motion()
                self.publish("cancelled", "自动重定位已取消或急停触发", method="global_spin")
                return
            if (rospy.Time.now() - self.scan_stamp).to_sec() > self.scan_timeout:
                self.stop_motion()
                self.publish("failed", "旋转过程中雷达数据中断", method="global_spin")
                return
            elapsed = time.time() - started
            cmd = Twist()
            cmd.angular.z = speed if elapsed <= timeout * 0.55 else -speed
            self.cmd_pub.publish(cmd)
            with self.lock:
                stable, pose = self.stable_samples, self.pose
            self.publish("converging", "AMCL 收敛检测中", method="global_spin",
                         progress=min(95, 10 + int(80 * elapsed / timeout)),
                         stable_samples=stable, required_samples=self.stable_required,
                         xy_variance=pose[0] if pose else None,
                         yaw_variance=pose[1] if pose else None)
            if elapsed >= 3.0 and stable >= self.stable_required:
                self.stop_motion()
                self.publish("success", "自动重定位成功", method="global_spin", progress=100,
                             elapsed_sec=round(elapsed, 2), xy_variance=pose[0], yaw_variance=pose[1])
                return
            rate.sleep()
        self.stop_motion()
        self.publish("failed", "自动重定位超时，AMCL 未稳定收敛", method="global_spin", progress=100)


if __name__ == "__main__":
    rospy.init_node("eggy_auto_relocalizer")
    AutoRelocalizer()
    rospy.spin()
