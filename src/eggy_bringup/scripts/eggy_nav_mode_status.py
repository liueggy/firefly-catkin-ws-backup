#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish the active Eggy navigation/localization mode for Qt.

The Qt client should not have to guess whether /initialpose is meaningful.
In mapping mode gmapping owns map->odom, while in static-map navigation AMCL
owns it and accepts /initialpose.  This node publishes a small latched JSON
status so the UI and task-chain controls can make that distinction explicit.
"""

import json
import os
import socket
import time

import rosnode
import rospy
from std_msgs.msg import String


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _node_present(nodes, name):
    return name in nodes or any(n.endswith("/" + name.lstrip("/")) for n in nodes)


def _http_service_present(host, port):
    try:
        with socket.create_connection((host, int(port)), timeout=0.25):
            return True
    except (OSError, ValueError):
        return False


class EggyNavModeStatus(object):
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 1.0))
        self.use_mapping = _as_bool(rospy.get_param("~use_mapping", True))
        self.use_amcl = _as_bool(rospy.get_param("~use_amcl", False))
        self.use_navigation = _as_bool(rospy.get_param("~use_navigation", True))
        self.map_file = rospy.get_param(
            "~map_file", "/root/catkin_ws/maps/navigation/latest.yaml"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/eggy/nav_mode/status"
        )
        self.pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)

    def build_status(self):
        try:
            nodes = set(rosnode.get_node_names())
        except rosnode.ROSNodeIOException:
            nodes = set()

        amcl_running = _node_present(nodes, "/amcl")
        gmapping_running = _node_present(nodes, "/slam_gmapping")
        move_base_running = _node_present(nodes, "/move_base")
        map_server_running = _node_present(nodes, "/map_server")
        rosbridge_running = _node_present(nodes, "/rosbridge_websocket")
        runner_running = _node_present(nodes, "/inspection_servo_route_runner")
        meter_running = _node_present(nodes, "/meter_rknn_detect_cpp")
        kimi_server_running = _http_service_present("127.0.0.1", 8000)
        kimi_bridge_running = _node_present(nodes, "/kimi_inspection_bridge")
        inspection_running = runner_running or meter_running or kimi_server_running or kimi_bridge_running

        conflict = (
            (self.use_amcl and self.use_mapping)
            or (amcl_running and gmapping_running)
        )

        if inspection_running and (amcl_running or self.use_amcl):
            mode = "inspection"
            localizer = "amcl"
            initialpose_supported = True
        elif amcl_running or self.use_amcl:
            mode = "navigation"
            localizer = "amcl"
            initialpose_supported = True
        elif gmapping_running or self.use_mapping:
            mode = "mapping"
            localizer = "gmapping"
            initialpose_supported = False
        else:
            mode = "unknown"
            localizer = "none"
            initialpose_supported = False

        if conflict:
            state = "conflict"
            message = "AMCL 与 gmapping 同时启用，map->odom 可能冲突。"
        elif mode == "inspection":
            state = "ok" if move_base_running and runner_running else "starting"
            message = "巡检模式，AMCL 导航与识别/巡检节点已纳入统一状态。"
        elif mode == "navigation":
            state = "ok" if move_base_running else "starting"
            message = "静态地图 + AMCL 导航模式，Qt 重定位会发布 /initialpose。"
        elif mode == "mapping":
            state = "ok" if gmapping_running else "starting"
            message = "建图模式，雷达与边界对齐由 gmapping scan matching 完成。"
        else:
            state = "unknown"
            message = "未识别到定位模式。"

        task_chain_ready = (
            self.use_navigation
            and move_base_running
            and runner_running
            and not conflict
        )

        capabilities = {
            "initialpose": initialpose_supported and not conflict,
            "mapping": bool(gmapping_running or self.use_mapping),
            "navigation": bool(move_base_running and (amcl_running or self.use_amcl)),
            "inspection": bool(inspection_running and move_base_running),
            "camera": _node_present(nodes, "/eggy_camera"),
            "meter_detection": meter_running,
            "profile_switch": True,
            "profiles": {
                "mapping": True,
                "navigation": os.path.isfile(self.map_file),
                "inspection": os.path.isfile(self.map_file),
            },
        }

        return {
            "stamp": time.time(),
            "mode": mode,
            "profile": mode,
            "state": state,
            "localizer": localizer,
            "map_file": self.map_file,
            "configured": {
                "use_mapping": self.use_mapping,
                "use_amcl": self.use_amcl,
                "use_navigation": self.use_navigation,
            },
            "observed": {
                "amcl": amcl_running,
                "gmapping": gmapping_running,
                "move_base": move_base_running,
                "map_server": map_server_running,
                "rosbridge": rosbridge_running,
                "inspection_runner": runner_running,
                "meter_detection": meter_running,
                "kimi_server": kimi_server_running,
                "kimi_bridge": kimi_bridge_running,
            },
            "initialpose_supported": initialpose_supported and not conflict,
            "task_chain_ready": task_chain_ready,
            "capabilities": capabilities,
            "message": message,
        }

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.pub.publish(String(data=json.dumps(self.build_status(), ensure_ascii=False)))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("eggy_nav_mode_status")
    EggyNavModeStatus().spin()
