#!/usr/bin/env python3
"""Recover rplidarNode when its process survives but /scan stops flowing."""

import json
import os
import signal
import time

import rospy
from std_msgs.msg import String

from eggy_bringup.lidar_watchdog_core import LidarWatchdogPolicy


class LidarWatchdog:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.driver_process_name = rospy.get_param(
            "~driver_process_name", "rplidarNode")
        self.kill_grace = float(rospy.get_param("~kill_grace", 1.5))
        self.policy = LidarWatchdogPolicy(
            startup_grace=rospy.get_param("~startup_grace", 8.0),
            stale_timeout=rospy.get_param("~scan_timeout", 2.0),
            recovery_cooldown=rospy.get_param("~recovery_cooldown", 10.0),
        )
        self.pending_pid = None
        self.pending_since = None
        self.recovery_count = 0
        self.status_pub = rospy.Publisher(
            "/eggy/lidar/watchdog_status", String, queue_size=1, latch=True)
        self.scan_sub = rospy.Subscriber(
            self.scan_topic, rospy.AnyMsg, self._on_scan,
            queue_size=1, tcp_nodelay=True)
        self.timer = rospy.Timer(rospy.Duration(0.5), self._on_timer)
        rospy.loginfo(
            "lidar watchdog active topic=%s startup_grace=%.1fs "
            "scan_timeout=%.1fs cooldown=%.1fs",
            self.scan_topic, self.policy.startup_grace,
            self.policy.stale_timeout, self.policy.recovery_cooldown)

    def _on_scan(self, _message):
        self.policy.observe_scan(time.monotonic())

    def _find_driver_pid(self):
        try:
            entries = os.listdir("/proc")
        except OSError:
            return None
        for entry in entries:
            if not entry.isdigit():
                continue
            try:
                with open("/proc/{}/comm".format(entry), "r") as stream:
                    process_name = stream.read().strip()
                if process_name == self.driver_process_name:
                    return int(entry)
            except (OSError, ValueError):
                continue
        return None

    @staticmethod
    def _is_alive(pid):
        if pid is None:
            return False
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False

    @staticmethod
    def _signal(pid, sig):
        try:
            os.kill(pid, sig)
            return True
        except OSError:
            return False

    def _publish_status(self, now, state):
        scan_age = self.policy.scan_age(now)
        process_age = self.policy.process_age(now)
        payload = {
            "state": state,
            "pid": self.policy.process_pid,
            "scan_age_s": None if scan_age is None else round(scan_age, 3),
            "process_age_s": (
                None if process_age is None else round(process_age, 3)),
            "recovery_count": self.recovery_count,
        }
        self.status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False,
                                   separators=(",", ":"))))

    def _on_timer(self, _event):
        now = time.monotonic()
        pid = self._find_driver_pid()
        self.policy.observe_process(pid, now)

        if self.pending_pid is not None:
            if not self._is_alive(self.pending_pid):
                self.pending_pid = None
                self.pending_since = None
            elif now - self.pending_since >= self.kill_grace:
                rospy.logwarn(
                    "rplidarNode pid=%d ignored SIGTERM; sending SIGKILL",
                    self.pending_pid)
                self._signal(self.pending_pid, signal.SIGKILL)
                self.pending_pid = None
                self.pending_since = None
                self._publish_status(now, "force_killed")
                return

        if self.pending_pid is None and self.policy.should_recover(now):
            stalled_pid = self.policy.process_pid
            scan_age = self.policy.scan_age(now)
            rospy.logerr(
                "lidar stream stalled (pid=%s scan_age=%s); restarting driver",
                stalled_pid,
                "never" if scan_age is None else "{:.1f}s".format(scan_age))
            self.policy.mark_recovery(now)
            self.recovery_count += 1
            if self._signal(stalled_pid, signal.SIGTERM):
                self.pending_pid = stalled_pid
                self.pending_since = now
            self._publish_status(now, "recovering")
            return

        if pid is None:
            state = "waiting_process"
        elif self.pending_pid is not None:
            state = "terminating"
        elif self.policy.last_scan_at is None:
            state = "starting"
        elif self.policy.scan_age(now) <= self.policy.stale_timeout:
            state = "online"
        else:
            state = "cooldown"
        self._publish_status(now, state)


if __name__ == "__main__":
    rospy.init_node("eggy_lidar_watchdog")
    LidarWatchdog()
    rospy.spin()
