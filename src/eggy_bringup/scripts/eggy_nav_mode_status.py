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
from eggy_bringup.profile_contract import build_profile_contract


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _node_present(nodes, name):
    canonical = "/" + name.lstrip("/")
    return (
        canonical in nodes
        or any(n.endswith(canonical) for n in nodes)
        or any(n.startswith(canonical + "_") for n in nodes)
    )


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
        self.profile = str(rospy.get_param("~profile", "mapping")).strip().lower()
        self.map_file = rospy.get_param(
            "~map_file", "/root/catkin_ws/maps/navigation/latest.yaml"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/eggy/nav_mode/status"
        )
        self.pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)

    def build_status(self):
        # Runtime profile switches update these private parameters without
        # restarting the persistent base stack.
        self.profile = str(rospy.get_param("~profile", self.profile)).strip().lower()
        self.use_mapping = _as_bool(
            rospy.get_param("~use_mapping", self.use_mapping))
        self.use_amcl = _as_bool(rospy.get_param("~use_amcl", self.use_amcl))
        self.use_navigation = _as_bool(
            rospy.get_param("~use_navigation", self.use_navigation))
        self.map_file = rospy.get_param("~map_file", self.map_file)
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
        camera_running = _node_present(nodes, "/eggy_camera")

        conflict = (
            (self.use_amcl and self.use_mapping)
            or (amcl_running and gmapping_running)
        )

        observed = {
            "amcl": amcl_running, "gmapping": gmapping_running,
            "move_base": move_base_running, "map_server": map_server_running,
            "rosbridge": rosbridge_running, "mission_runner": runner_running,
            "camera": camera_running, "meter_detection": meter_running,
            "kimi_server": kimi_server_running, "kimi_bridge": kimi_bridge_running,
        }
        status = build_profile_contract(
            self.profile, observed, map_available=os.path.isfile(self.map_file))
        if conflict:
            status["state"] = "degraded"
            status["message"] = "AMCL 与 gmapping 同时启用，map->odom 可能冲突。"
        else:
            status["message"] = "profile 来自启动配置；节点仅用于 ready/degraded 健康判定。"
        status.update({
            "stamp": time.time(),
            "map_file": self.map_file,
            "configured": {
                "use_mapping": self.use_mapping,
                "use_amcl": self.use_amcl,
                "use_navigation": self.use_navigation,
            },
            "initialpose_supported": status["capabilities"]["initialpose"] and not conflict,
            "task_chain_ready": status["state"] == "ready" and self.profile != "mapping",
        })
        return status

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.pub.publish(String(data=json.dumps(self.build_status(), ensure_ascii=False)))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("eggy_nav_mode_status")
    EggyNavModeStatus().spin()
