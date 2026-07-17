#!/usr/bin/env python3
"""Closed-loop CI1302 V5 voice controller for the Eggy robot.

Recognized commands arrive from the STM32 as JSON on
``/stm32/voice_command``.  This node validates safety, routes the command to
the existing ROS owner, and only then requests a passive CI1302 broadcast on
``/stm32/voice_playback``.  Voice commands can engage, but never release, the
software emergency stop.
"""

import copy
import json
import os
import threading
import time
import uuid

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, Float32, String, UInt8
from std_srvs.srv import Empty

from eggy_bringup.voice_protocol import (
    BROADCAST,
    decode_voice_command,
    encode_playback_request,
    select_battery_broadcast,
    select_profile_broadcast,
)


class VoiceController(object):
    def __init__(self):
        rospy.init_node("eggy_voice_controller")
        self.lock = threading.RLock()
        self.forward_speed = float(rospy.get_param("~forward_speed", 0.16))
        self.backward_speed = float(rospy.get_param("~backward_speed", 0.12))
        self.lateral_speed = float(rospy.get_param("~lateral_speed", 0.12))
        self.rotate_speed = float(rospy.get_param("~rotate_speed", 0.35))
        self.linear_duration = float(rospy.get_param("~linear_duration", 0.55))
        self.lateral_duration = float(rospy.get_param("~lateral_duration", 0.45))
        self.rotate_duration = float(rospy.get_param("~rotate_duration", 0.55))
        self.sensor_timeout = float(rospy.get_param("~sensor_timeout", 1.0))
        self.camera_timeout = float(rospy.get_param("~camera_timeout", 2.0))
        self.route_file = rospy.get_param(
            "~route_file", "$(find eggy_bringup)/config/inspection_route.json")
        self.default_map_file = rospy.get_param("~default_map_file", "")

        self.emergency_stop = False
        self.base_stop = False
        self.battery_voltage = None
        self.temperature = None
        self.humidity = None
        self.profile_status = {}
        self.command_status = {}
        self.network_status = {}
        self.auto_mapping_status = {}
        self.mission_status = {}
        self.cached_mission = None
        self.paused_mission = None
        self.active_auto_mapping_id = ""
        self.pending_commands = {}
        self.ai_inspection_enabled = True
        self.recognition_target = ""
        self.active_recognition_id = ""
        self.last_scan_time = 0.0
        self.last_odom_time = 0.0
        self.last_camera_time = 0.0
        self.last_result_broadcast = BROADCAST["voice_ready"]
        self.last_auto_state = ""
        self.last_mission_state = ""
        self.motion_timer = None
        self.stop_timer = None
        self.motion_twist = Twist()
        self.tf_listener = tf.TransformListener()

        self.cmd_pub = rospy.Publisher(
            rospy.get_param("~cmd_vel_topic", "/cmd_vel/manual"), Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/eggy/voice/status", String, queue_size=10, latch=True)
        self.playback_pub = rospy.Publisher("/stm32/voice_playback", String, queue_size=20)
        self.command_pub = rospy.Publisher("/eggy/command/request", String, queue_size=10)
        self.mission_pub = rospy.Publisher("/eggy/mission/request", String, queue_size=10)
        self.auto_mapping_pub = rospy.Publisher("/eggy/auto_mapping/request", String, queue_size=10)
        self.emergency_pub = rospy.Publisher("/eggy/emergency_stop", Bool, queue_size=1, latch=True)

        rospy.Subscriber("/stm32/voice_command", String, self.on_voice, queue_size=20)
        rospy.Subscriber("/stm32/voice_playback/status", String, self.on_playback_status, queue_size=20)
        rospy.Subscriber("/eggy/emergency_stop", Bool, self.on_emergency, queue_size=1)
        rospy.Subscriber("/base/flag_stop", UInt8, self.on_base_stop, queue_size=1)
        rospy.Subscriber("/battery/voltage", Float32, self.on_battery, queue_size=1)
        rospy.Subscriber("/stm32/dht11/temperature", Float32, self.on_temperature, queue_size=1)
        rospy.Subscriber("/stm32/dht11/humidity", Float32, self.on_humidity, queue_size=1)
        rospy.Subscriber("/eggy/nav_mode/status", String, self.on_profile, queue_size=1)
        rospy.Subscriber("/eggy/command/status", String, self.on_command_status, queue_size=1)
        rospy.Subscriber("/eggy/command/response", String, self.on_command_response, queue_size=20)
        rospy.Subscriber("/eggy/network/status", String, self.on_network, queue_size=1)
        rospy.Subscriber("/eggy/auto_mapping/status", String, self.on_auto_mapping_status, queue_size=10)
        rospy.Subscriber("/eggy/auto_mapping/result", String, self.on_auto_mapping_result, queue_size=10)
        rospy.Subscriber("/eggy/mission/request", String, self.on_mission_request, queue_size=10)
        rospy.Subscriber("/eggy/mission/status", String, self.on_mission_status, queue_size=10)
        rospy.Subscriber("/eggy/mission/result", String, self.on_mission_result, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, lambda _msg: self._mark("scan"), queue_size=1)
        rospy.Subscriber("/odom", Odometry, lambda _msg: self._mark("odom"), queue_size=1)
        rospy.Subscriber("/camera/front/image/compressed", CompressedImage,
                         lambda _msg: self._mark("camera"), queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.publish_status("ready", "CI1302 V5 voice controller ready")
        self.startup_timer = rospy.Timer(
            rospy.Duration(1.0), self._announce_ready, oneshot=True)

    def _announce_ready(self, _event):
        self.play(BROADCAST["voice_ready"])

    @staticmethod
    def _json(text):
        try:
            value = json.loads(text)
            return value if isinstance(value, dict) else {}
        except (TypeError, ValueError):
            return {}

    def _mark(self, name):
        setattr(self, "last_%s_time" % name, time.time())

    def play(self, broadcast_id, remember=True):
        if remember:
            self.last_result_broadcast = int(broadcast_id)
        self.playback_pub.publish(String(data=encode_playback_request(broadcast_id)))

    def publish_status(self, state, message, **details):
        payload = {
            "schema_version": 1,
            "state": state,
            "message": message,
            "emergency_stop": self.emergency_stop,
            "base_stop": self.base_stop,
            "ai_inspection_enabled": self.ai_inspection_enabled,
            "stamp": rospy.Time.now().to_sec() if not rospy.is_shutdown() else 0.0,
            "details": details,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def on_playback_status(self, msg):
        status = self._json(msg.data)
        if status and not status.get("success", False):
            rospy.logwarn_throttle(5.0, "voice playback failed: %s", status.get("message"))

    def on_emergency(self, msg):
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.halt_motion()

    def on_base_stop(self, msg):
        self.base_stop = bool(msg.data)
        if self.base_stop:
            self.halt_motion()

    def on_battery(self, msg):
        self.battery_voltage = float(msg.data)

    def on_temperature(self, msg):
        self.temperature = float(msg.data)

    def on_humidity(self, msg):
        self.humidity = float(msg.data)

    def on_profile(self, msg):
        self.profile_status = self._json(msg.data)

    def on_command_status(self, msg):
        self.command_status = self._json(msg.data)

    def on_network(self, msg):
        self.network_status = self._json(msg.data)

    def on_command_response(self, msg):
        response = self._json(msg.data)
        request_id = str(response.get("request_id", ""))
        pending = self.pending_commands.pop(request_id, None)
        if not pending:
            return
        if response.get("success"):
            if pending.get("success_broadcast") is not None:
                self.play(pending["success_broadcast"])
        else:
            self.play(BROADCAST["cannot_execute"])
        self.publish_status("command_response", response.get("message", ""),
                            request_id=request_id, action=pending.get("action"),
                            success=bool(response.get("success")))

    def on_auto_mapping_status(self, msg):
        status = self._json(msg.data)
        if not status:
            return
        self.auto_mapping_status = status
        request_id = str(status.get("request_id", ""))
        if request_id and bool(status.get("busy")):
            self.active_auto_mapping_id = request_id
        state = str(status.get("state", ""))
        if request_id == self.active_auto_mapping_id and state in (
                "idle", "completed", "cancelled", "aborted", "failed"):
            self.active_auto_mapping_id = ""
        if state == self.last_auto_state:
            return
        self.last_auto_state = state
        mapping = {
            "preflight": "safety_checking", "selecting_frontier": "frontier_selecting",
            "navigating": "exploring", "mapping": "map_updating",
            "final_scan": "final_scan", "paused": "auto_mapping_paused",
        }
        if state in mapping:
            self.play(BROADCAST[mapping[state]])

    def on_auto_mapping_result(self, msg):
        result = self._json(msg.data)
        state = str(result.get("state", ""))
        if state in ("completed", "success"):
            self.play(BROADCAST["auto_mapping_complete"])
        elif state == "cancelled":
            self.play(BROADCAST["auto_mapping_stopped"])
        else:
            self.play(BROADCAST["auto_mapping_failed"])
        self.active_auto_mapping_id = ""

    def on_mission_request(self, msg):
        payload = self._json(msg.data)
        if str(payload.get("command", "start")).lower() == "start" and payload.get("route"):
            self.cached_mission = copy.deepcopy(payload)

    def on_mission_status(self, msg):
        status = self._json(msg.data)
        if not status:
            return
        self.mission_status = status
        state = str(status.get("state", ""))
        if state == self.last_mission_state:
            return
        self.last_mission_state = state
        mapping = {
            "accepted": "route_started", "navigating": "going_to_target",
            "arrived": "arrived", "searching_target": "searching_target",
            "search_rotating": "searching_target",
            "aligned": "target_aligned", "kimi_running": "ai_running",
            "returning_home": "returning_home",
        }
        if state == "target_confirmed":
            target = (status.get("extra") or {}).get("target") or {}
            detected_class = str(target.get("class_name", ""))
            key = {"water_meter": "water_meter_found",
                   "pressure_gauge": "pressure_gauge_found"}.get(
                       detected_class, "target_found")
            self.play(BROADCAST[key])
        elif state in mapping:
            self.play(BROADCAST[mapping[state]])

    def on_mission_result(self, msg):
        result = self._json(msg.data)
        state = str(result.get("state", ""))
        request_id = str(result.get("request_id", ""))
        is_recognition = bool(self.active_recognition_id and
                              request_id == self.active_recognition_id)
        if is_recognition:
            points = result.get("results") or []
            point = points[0] if points and isinstance(points[0], dict) else {}
            search = point.get("search") if isinstance(point.get("search"), dict) else {}
            kimi = point.get("kimi") if isinstance(point.get("kimi"), dict) else {}
            succeeded = (state == "completed" and bool(search.get("ok")) and
                         (not kimi or bool(kimi.get("ok"))))
            self.play(BROADCAST["recognition_ok" if succeeded else "recognition_failed"])
        elif state == "completed":
            key = "inspection_complete" if result.get("mission_type") == "inspection" else "task_complete"
            self.play(BROADCAST[key])
        elif state == "cancelled" and self.paused_mission is not None:
            self.play(BROADCAST["task_paused"])
        elif state == "cancelled":
            self.play(BROADCAST["task_cancelled"])
        else:
            self.play(BROADCAST["navigation_failed"])
        if is_recognition:
            self.active_recognition_id = ""

    def safety_allows_motion(self):
        if self.emergency_stop:
            self.play(BROADCAST["emergency_locked"])
            return False
        if self.base_stop:
            self.play(BROADCAST["obstacle_stopped"])
            return False
        return True

    def halt_motion(self):
        with self.lock:
            if self.motion_timer:
                self.motion_timer.shutdown()
                self.motion_timer = None
            if self.stop_timer:
                self.stop_timer.shutdown()
                self.stop_timer = None
            self.motion_twist = Twist()
        self.cmd_pub.publish(Twist())

    def start_micro_motion(self, vx, vy, wz, duration):
        if not self.safety_allows_motion():
            return
        self.halt_motion()
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = vx, vy, wz
        with self.lock:
            self.motion_twist = twist
            self.motion_timer = rospy.Timer(rospy.Duration(0.05), self.publish_motion)
            self.stop_timer = rospy.Timer(rospy.Duration(duration), self.finish_micro_motion, oneshot=True)
        self.play(BROADCAST["accepted"])

    def publish_motion(self, _event):
        if self.emergency_stop or self.base_stop:
            self.halt_motion()
            return
        self.cmd_pub.publish(self.motion_twist)

    def finish_micro_motion(self, _event=None):
        self.halt_motion()
        self.play(BROADCAST["micro_move_complete"])

    def send_command(self, action, success_broadcast=None, params=None, target=""):
        request_id = "voice-%s-%s" % (action, uuid.uuid4().hex[:10])
        request = {
            "schema_version": 1, "request_id": request_id,
            "command": action, "target": target, "params": params or {},
            "source": "voice_v5",
        }
        self.pending_commands[request_id] = {
            "action": action, "success_broadcast": success_broadcast,
        }
        self.command_pub.publish(String(data=json.dumps(request, ensure_ascii=False)))
        self.play(BROADCAST["accepted"])
        return request_id

    def send_auto_mapping(self, command, announce=True):
        if command == "start":
            if self.active_auto_mapping_id:
                self.play(BROADCAST["cannot_execute"])
                return
            if self.profile_status.get("profile") != "mapping":
                self.play(BROADCAST["cannot_execute"])
                return
            if not self.safety_allows_motion():
                return
            self.active_auto_mapping_id = "voice-auto-%s" % uuid.uuid4().hex[:12]
            payload = {
                "schema_version": 1, "request_id": self.active_auto_mapping_id,
                "command": "start", "options": {
                    "max_duration_sec": 900, "max_linear_speed": 0.22,
                    "return_home": True,
                    "save_draft_on_abort": True,
                },
            }
        else:
            if not self.active_auto_mapping_id:
                self.play(BROADCAST["cannot_execute"])
                return
            payload = {
                "schema_version": 1, "request_id": self.active_auto_mapping_id,
                "command": command,
            }
        self.auto_mapping_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if announce:
            self.play(BROADCAST["accepted"])

    def load_voice_route(self):
        path = self.route_file
        if path.startswith("$(find eggy_bringup)"):
            path = path.replace("$(find eggy_bringup)",
                                os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        try:
            with open(path, encoding="utf-8") as stream:
                payload = json.load(stream)
            route = payload.get("route", payload if isinstance(payload, list) else [])
            return [item for item in route if isinstance(item, dict) and
                    not str(item.get("id", "")).startswith("example_")]
        except (OSError, ValueError):
            return []

    def start_route(self, inspection, point_index=None, return_home=False):
        profile = str(self.profile_status.get("profile", ""))
        allowed = profile == "inspection" if inspection else profile in (
            "navigation", "inspection")
        if not allowed:
            self.play(BROADCAST["cannot_execute"])
            return
        route = self.load_voice_route()
        if point_index is not None:
            route = route[point_index:point_index + 1]
        if not route:
            self.play(BROADCAST["no_route"])
            return
        request = {
            "schema_version": 1, "request_id": "voice-mission-%s" % uuid.uuid4().hex[:12],
            "command": "start", "mission_type": "inspection" if inspection else "navigation",
            "loop": False, "return_home": bool(return_home), "on_nav_failure": "stop",
            "inspection": {
                "enabled": bool(inspection), "vision_search": bool(inspection),
                "ai_analysis": bool(inspection and self.ai_inspection_enabled),
            }, "route": route,
        }
        self.cached_mission = copy.deepcopy(request)
        self.mission_pub.publish(String(data=json.dumps(request, ensure_ascii=False)))
        self.play(BROADCAST["inspection_started" if inspection else "route_started"])

    def start_home(self):
        if self.profile_status.get("profile") not in ("navigation", "inspection"):
            self.play(BROADCAST["cannot_execute"])
            return
        route = self.load_voice_route()
        home = next((item for item in route if str(item.get("id", "")).lower()
                     in ("home", "start", "origin", "起点")), None)
        if home is None:
            self.play(BROADCAST["no_route"])
            return
        request = {
            "schema_version": 1, "request_id": "voice-home-%s" % uuid.uuid4().hex[:12],
            "command": "start", "mission_type": "navigation", "loop": False,
            "return_home": False, "on_nav_failure": "stop",
            "inspection": {"enabled": False, "vision_search": False, "ai_analysis": False},
            "route": [home],
        }
        self.cached_mission = copy.deepcopy(request)
        self.mission_pub.publish(String(data=json.dumps(request, ensure_ascii=False)))
        self.play(BROADCAST["returning_home"])

    def cancel_task(self, pause=False, announce=True):
        self.halt_motion()
        active_id = (str(self.mission_status.get("request_id", ""))
                     if bool(self.mission_status.get("busy")) else "")
        if active_id:
            if pause and self.cached_mission:
                self.paused_mission = copy.deepcopy(self.cached_mission)
            request = {
                "schema_version": 1, "request_id": active_id, "command": "cancel",
                "mission_type": self.mission_status.get("mission_type", "navigation"),
            }
            self.mission_pub.publish(String(data=json.dumps(request, ensure_ascii=False)))
        if self.active_auto_mapping_id:
            self.send_auto_mapping("pause" if pause else "cancel", announce=announce)
        if not announce:
            return
        if not active_id and not self.active_auto_mapping_id:
            self.play(BROADCAST["task_idle"])
        elif not pause:
            self.play(BROADCAST["task_cancelled"])

    def resume_task(self):
        if (self.active_auto_mapping_id and
                str(self.auto_mapping_status.get("state", "")) == "paused"):
            self.send_auto_mapping("resume")
            return
        if not self.paused_mission or not self.safety_allows_motion():
            self.play(BROADCAST["cannot_execute"])
            return
        request = copy.deepcopy(self.paused_mission)
        index = self.mission_status.get("point_index")
        if isinstance(index, int) and index > 0:
            request["route"] = request.get("route", [])[index:]
        request["request_id"] = "voice-resume-%s" % uuid.uuid4().hex[:12]
        request["command"] = "start"
        self.paused_mission = None
        self.cached_mission = copy.deepcopy(request)
        self.mission_pub.publish(String(data=json.dumps(request, ensure_ascii=False)))
        self.play(BROADCAST["task_resumed"])

    def start_recognition(self, target):
        if not self.safety_allows_motion():
            return
        if self.profile_status.get("profile") != "inspection":
            self.play(BROADCAST["cannot_execute"])
            return
        try:
            trans, rot = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
        except Exception:
            self.play(BROADCAST["localization_lost"])
            return
        self.recognition_target = target
        request_id = "voice-recognition-%s" % uuid.uuid4().hex[:12]
        request = {
            "schema_version": 1,
            "request_id": request_id,
            "command": "start", "mission_type": "inspection", "loop": False,
            "return_home": False, "on_nav_failure": "stop",
            "inspection": {"enabled": True, "vision_search": True, "ai_analysis": True},
            "route": [{
                "id": "voice_current_pose", "frame_id": "map",
                "x": float(trans[0]), "y": float(trans[1]), "yaw": float(yaw),
                "expected_class": target,
            }],
        }
        self.active_recognition_id = request_id
        self.cached_mission = copy.deepcopy(request)
        self.mission_pub.publish(String(data=json.dumps(request, ensure_ascii=False)))
        self.play(BROADCAST["searching_target"])

    def query_network(self):
        wifi = bool((self.network_status.get("wifi") or {}).get("connected"))
        mobile = bool((self.network_status.get("4g") or {}).get("connected"))
        if wifi:
            self.play(BROADCAST["wifi_connected"])
        elif mobile:
            self.play(BROADCAST["mobile_connected"])
        else:
            self.play(BROADCAST["network_abnormal"])

    def relocalize(self):
        if self.profile_status.get("profile") not in ("navigation", "inspection"):
            self.play(BROADCAST["cannot_execute"])
            return
        self.play(BROADCAST["relocalizing"])
        threading.Thread(target=self._call_global_localization, daemon=True).start()

    def _call_global_localization(self):
        try:
            rospy.wait_for_service("/global_localization", timeout=3.0)
            rospy.ServiceProxy("/global_localization", Empty)()
            self.play(BROADCAST["relocalize_ok"])
        except (rospy.ROSException, rospy.ServiceException):
            self.play(BROADCAST["relocalize_failed"])

    def handle_action(self, action):
        motion = {
            "move_forward": (self.forward_speed, 0.0, 0.0, self.linear_duration),
            "move_backward": (-self.backward_speed, 0.0, 0.0, self.linear_duration),
            "move_left": (0.0, self.lateral_speed, 0.0, self.lateral_duration),
            "move_right": (0.0, -self.lateral_speed, 0.0, self.lateral_duration),
            "rotate_left": (0.0, 0.0, self.rotate_speed, self.rotate_duration),
            "rotate_right": (0.0, 0.0, -self.rotate_speed, self.rotate_duration),
        }
        if action == "emergency_stop":
            self.halt_motion()
            self.emergency_pub.publish(Bool(data=True))
            self.cancel_task(announce=False)
            self.play(BROADCAST["emergency_locked"])
        elif action == "cancel_task": self.cancel_task()
        elif action in motion: self.start_micro_motion(*motion[action])
        elif action == "query_status":
            self.play(BROADCAST["status_abnormal"] if self.emergency_stop or self.base_stop else BROADCAST["status_ok"])
        elif action == "query_battery":
            self.play(select_battery_broadcast(self.battery_voltage) if self.battery_voltage is not None else BROADCAST["status_abnormal"])
        elif action == "query_profile": self.play(select_profile_broadcast(self.profile_status.get("profile")))
        elif action == "query_task":
            mission_state = str(self.mission_status.get("state", ""))
            active = mission_state not in ("", "ready", "completed", "cancelled", "error") or bool(self.active_auto_mapping_id)
            self.play(BROADCAST["task_running"] if active else BROADCAST["task_idle"])
        elif action == "query_environment":
            if self.temperature is None or self.humidity is None:
                self.play(BROADCAST["status_abnormal"])
            else:
                self.play(BROADCAST["temperature_ok"] if 0 <= self.temperature <= 60 else BROADCAST["temperature_abnormal"])
                self.play(BROADCAST["humidity_ok"] if 10 <= self.humidity <= 90 else BROADCAST["humidity_abnormal"])
        elif action == "camera_start": self.send_command("camera_start", BROADCAST["camera_online"])
        elif action == "camera_stop": self.send_command("camera_stop", BROADCAST["camera_closed"])
        elif action.startswith("profile_"):
            profile = action.split("_", 1)[1]
            params = {"profile": profile}
            map_file = self.default_map_file or self.profile_status.get("map_file", "")
            if profile != "mapping" and map_file:
                params["map_file"] = map_file
            self.send_command("switch_profile", None, params=params)
        elif action == "mapping_start": self.send_command("mapping_start", BROADCAST["mapping_started"])
        elif action == "mapping_stop": self.send_command("mapping_stop", BROADCAST["mapping_stopped"])
        elif action == "mapping_reset": self.send_command("mapping_reset", BROADCAST["mapping_reset"])
        elif action == "clear_costmaps": self.send_command("clear_costmaps", BROADCAST["costmaps_cleared"])
        elif action == "save_map": self.send_command("map_save", BROADCAST["map_saved"])
        elif action == "return_home": self.start_home()
        elif action.startswith("navigate_point_"): self.start_route(False, int(action.rsplit("_", 1)[1]) - 1)
        elif action == "start_navigation": self.start_route(False)
        elif action == "start_inspection": self.start_route(True)
        elif action.startswith("recognize_"):
            target = {"recognize_any_meter": "any", "recognize_water_meter": "water_meter",
                      "recognize_pressure_gauge": "pressure_gauge", "recognize_retry": self.recognition_target or "any"}[action]
            self.start_recognition(target)
        elif action == "repeat_result": self.play(self.last_result_broadcast, remember=False)
        elif action == "auto_mapping_start": self.send_auto_mapping("start")
        elif action == "auto_mapping_pause": self.send_auto_mapping("pause")
        elif action == "auto_mapping_resume": self.send_auto_mapping("resume")
        elif action == "auto_mapping_stop": self.send_auto_mapping("cancel")
        elif action == "auto_mapping_status":
            state = str(self.auto_mapping_status.get("state", ""))
            self.play(BROADCAST["task_running"] if state and state not in ("idle", "completed", "cancelled", "failed") else BROADCAST["task_idle"])
        elif action == "ai_inspection_enable":
            self.ai_inspection_enabled = True; self.play(BROADCAST["ai_inspection_enabled"])
        elif action == "ai_inspection_disable":
            self.ai_inspection_enabled = False; self.play(BROADCAST["ai_inspection_disabled"])
        elif action == "query_network": self.query_network()
        elif action == "relocalize": self.relocalize()
        elif action == "pause_task": self.cancel_task(pause=True)
        elif action == "resume_task": self.resume_task()
        elif action == "query_camera":
            self.play(BROADCAST["camera_ready"] if time.time() - self.last_camera_time <= self.camera_timeout else BROADCAST["camera_abnormal"])
        elif action == "query_safety":
            fresh = (time.time() - self.last_scan_time <= self.sensor_timeout and time.time() - self.last_odom_time <= self.sensor_timeout)
            self.play(BROADCAST["safety_ok"] if fresh and not self.emergency_stop and not self.base_stop else BROADCAST["safety_abnormal"])
        else:
            self.play(BROADCAST["cannot_execute"])

    def on_voice(self, msg):
        try:
            command = decode_voice_command(msg.data)
        except ValueError as exc:
            rospy.logwarn("invalid voice command: %s", exc)
            self.publish_status("rejected", str(exc))
            return
        rospy.loginfo("voice V5 command %02X -> %s", command.command_id, command.action)
        self.publish_status("received", "voice command received",
                            command_id="%02X" % command.command_id, action=command.action)
        try:
            self.handle_action(command.action)
        except Exception as exc:
            rospy.logerr("voice action failed: %s", exc)
            self.play(BROADCAST["cannot_execute"])
            self.publish_status("error", str(exc), action=command.action)

    def shutdown(self):
        self.halt_motion()


if __name__ == "__main__":
    try:
        VoiceController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
