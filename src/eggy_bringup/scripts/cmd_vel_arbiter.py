#!/usr/bin/env python3
"""Lease-based cmd_vel priority arbiter; the base still subscribes to /cmd_vel."""
import json
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from cmd_vel_policy import select_source


class CmdVelArbiter(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.sources = {}
        self.config = {
            "navigation": ("/cmd_vel/navigation", 40, 0.6),
            "mission": ("/cmd_vel/mission", 60, 0.6),
            "manual": ("/cmd_vel/manual", 80, 0.4),
            "safety": ("/cmd_vel/safety", 100, 0.5),
        }
        self.output = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status = rospy.Publisher("/eggy/cmd_vel/control", String,
                                      queue_size=1, latch=True)
        self.last_status_signature = None
        self.last_status_stamp = 0.0
        for name, (topic, priority, timeout) in self.config.items():
            rospy.Subscriber(topic, Twist, self._callback,
                             callback_args=(name, priority, timeout), queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

    def _callback(self, msg, args):
        name, priority, timeout = args
        with self.lock:
            self.sources[name] = {"stamp": rospy.get_time(), "timeout": timeout,
                                  "priority": priority, "twist": msg}

    def _tick(self, _event):
        now = rospy.get_time()
        with self.lock:
            active = select_source(self.sources, now)
            msg = self.sources[active]["twist"] if active else Twist()
            live = [name for name, item in self.sources.items()
                    if now - item["stamp"] <= item["timeout"]]
        self.output.publish(msg)
        status = {
            "schema_version": 1, "stamp": now, "active_source": active or "none",
            "live_sources": sorted(live), "output_topic": "/cmd_vel",
            "priorities": {name: cfg[1] for name, cfg in self.config.items()},
            "timeouts": {name: cfg[2] for name, cfg in self.config.items()},
        }
        signature = (status["active_source"], tuple(status["live_sources"]))
        if signature != self.last_status_signature or now - self.last_status_stamp >= 1.0:
            self.status.publish(String(data=json.dumps(status, sort_keys=True)))
            self.last_status_signature = signature
            self.last_status_stamp = now


if __name__ == "__main__":
    rospy.init_node("eggy_cmd_vel_arbiter")
    CmdVelArbiter()
    rospy.spin()
