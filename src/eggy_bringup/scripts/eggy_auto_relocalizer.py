#!/usr/bin/env python3
import json
import threading
import time

import rosnode
import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt8
from std_srvs.srv import Empty


class AutoRelocalizer:
    def __init__(self):
        self.lock = threading.RLock()
        self.worker = None
        self.cancel_event = threading.Event()
        self.scan_stamp = rospy.Time(0)
        self.min_scan_range = None
        self.flag_stop = 1
        self.pose = None
        self.stable_samples = 0
        self.angular_speed = abs(float(rospy.get_param("~angular_speed", 0.28)))
        self.timeout = float(rospy.get_param("~timeout", 35.0))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 1.0))
        self.xy_variance_max = float(rospy.get_param("~xy_variance_max", 0.20))
        self.yaw_variance_max = float(rospy.get_param("~yaw_variance_max", 0.12))
        self.stable_required = int(rospy.get_param("~stable_samples", 8))
        self.dry_run = bool(rospy.get_param("~dry_run", True))
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/eggy/relocalization/status", String, queue_size=10, latch=True)
        self.cancel_nav_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        rospy.Subscriber("/eggy/relocalization/request", String, self.request_cb, queue_size=2)
        rospy.Subscriber("/eggy/relocalization/cancel", String, self.cancel_cb, queue_size=2)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_cb, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=2)
        rospy.Subscriber("/base/flag_stop", UInt8, self.flag_cb, queue_size=2)
        rospy.on_shutdown(self.stop_motion)
        self.publish("idle", "自动重定位节点已就绪", progress=0)

    def publish(self, state, message, **extra):
        payload = {"state": state, "message": message, "stamp": rospy.Time.now().to_sec(), "dry_run": self.dry_run}
        payload.update(extra)
        self.status_pub.publish(String(json.dumps(payload, ensure_ascii=False)))

    def scan_cb(self, msg):
        self.scan_stamp = rospy.Time.now()
        valid = [value for value in msg.ranges
                 if msg.range_min <= value <= msg.range_max]
        self.min_scan_range = min(valid) if valid else None

    def flag_cb(self, msg):
        self.flag_stop = int(msg.data)
        if self.flag_stop != 0 and self.worker and self.worker.is_alive():
            self.cancel_event.set()

    def pose_cb(self, msg):
        cov = msg.pose.covariance
        pose_quality = (max(float(cov[0]), float(cov[7])), float(cov[35]))
        with self.lock:
            self.pose = pose_quality
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

    def preflight(self):
        if "/amcl" not in set(rosnode.get_node_names()):
            return False, "AMCL 未运行，当前不是静态地图定位模式"
        if self.flag_stop != 0:
            return False, "底盘急停或状态未知，拒绝运动"
        if self.scan_stamp.is_zero() or (rospy.Time.now() - self.scan_stamp).to_sec() > self.scan_timeout:
            return False, "激光雷达数据超时"
        if self.min_scan_range is None:
            return False, "激光雷达没有有效距离数据"
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
        ok, reason = self.preflight()
        if not ok:
            self.stop_motion()
            self.publish("rejected", reason, progress=0)
            return
        timeout = min(90.0, max(10.0, float(request.get("timeout", self.timeout))))
        speed = min(0.45, max(0.12, abs(float(request.get("angular_speed", self.angular_speed)))))
        self.publish("preflight_ok", "安全检查通过", progress=5)
        if self.dry_run:
            self.publish("dry_run_complete", "Dry-run 完成：AMCL、雷达、急停和服务均正常", progress=100)
            return
        self.cancel_nav_pub.publish(GoalID())
        self.stop_motion()
        try:
            rospy.ServiceProxy("/global_localization", Empty)()
        except rospy.ServiceException as exc:
            self.publish("failed", "全局粒子初始化失败", error=str(exc))
            return
        with self.lock:
            self.stable_samples = 0
            self.pose = None
        self.publish("rotating", "正在旋转扫描并等待 AMCL 收敛", progress=10)
        started = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() - started < timeout:
            if self.cancel_event.is_set() or self.flag_stop != 0:
                self.stop_motion()
                self.publish("cancelled", "自动重定位已取消或急停触发")
                return
            if (rospy.Time.now() - self.scan_stamp).to_sec() > self.scan_timeout:
                self.stop_motion()
                self.publish("failed", "旋转过程中雷达数据中断")
                return
            elapsed = time.time() - started
            cmd = Twist()
            cmd.angular.z = speed if elapsed <= timeout * 0.55 else -speed
            self.cmd_pub.publish(cmd)
            with self.lock:
                stable, pose = self.stable_samples, self.pose
            self.publish("converging", "AMCL 收敛检测中",
                         progress=min(95, 10 + int(80 * elapsed / timeout)),
                         stable_samples=stable, required_samples=self.stable_required,
                         xy_variance=pose[0] if pose else None,
                         yaw_variance=pose[1] if pose else None)
            if elapsed >= 3.0 and stable >= self.stable_required:
                self.stop_motion()
                self.publish("success", "自动重定位成功", progress=100,
                             elapsed_sec=round(elapsed, 2),
                             xy_variance=pose[0], yaw_variance=pose[1])
                return
            rate.sleep()
        self.stop_motion()
        self.publish("failed", "自动重定位超时，AMCL 未稳定收敛", progress=100)


if __name__ == "__main__":
    rospy.init_node("eggy_auto_relocalizer")
    AutoRelocalizer()
    rospy.spin()
