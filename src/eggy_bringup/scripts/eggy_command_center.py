#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Eggy command center for Qt/rosbridge debugging.

Topics:
  /eggy/command/request   std_msgs/String JSON request
  /eggy/command/response  std_msgs/String JSON response
  /eggy/command/status    std_msgs/String JSON periodic status
"""
import json
import os
import signal
import subprocess
import time
import base64
import hashlib
import re
import shlex
import shutil
import socket
import tempfile
import uuid
import yaml

import rospy
import rosgraph
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_srvs.srv import Empty
import dynamic_reconfigure.client


KNOWN_NODES = [
    '/stm32_base_driver', '/rplidarNode', '/eggy_external_imu_odom_fuser',
    '/slam_gmapping', '/map_server', '/amcl', '/move_base',
    '/ros_qt5_gui_adapter', '/rosbridge_websocket', '/eggy_camera',
    '/meter_rknn_detect_cpp', '/kimi_inspection_server',
    '/kimi_inspection_bridge', '/inspection_servo_route_runner',
    '/eggy_health_aggregator', '/robot_state_publisher'
]

KEY_TOPICS = [
    '/map', '/scan', '/odom', '/wheel_odom', '/cmd_vel', '/goal_pose',
    '/nav_goal', '/plan', '/local_plan', '/global_costmap/costmap',
    '/local_costmap/costmap', '/camera/front/image/compressed', '/diagnostics'
]

MAP_LIBRARY_ROOT = '/root/catkin_ws/maps/library'
ACTIVE_MAP_LINK = '/root/catkin_ws/maps/active'


def run_cmd(cmd, timeout=5):
    try:
        p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, universal_newlines=True,
                             preexec_fn=os.setsid)
        try:
            out, _ = p.communicate(timeout=timeout)
            return p.returncode, (out or '').strip()
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
            out, _ = p.communicate()
            return 124, ((out or '').strip() + '\nTIMEOUT').strip()
    except Exception as exc:
        return 1, str(exc)


def http_service_alive(host, port):
    try:
        with socket.create_connection((host, int(port)), timeout=0.25):
            return True
    except (OSError, ValueError):
        return False


def rosnode_list():
    try:
        master = rosgraph.Master('/eggy_command_center')
        state = master.getSystemState()
        names = set()
        for group in state:
            for _topic, node_names in group:
                names.update(node_names)
        return sorted(names)
    except Exception:
        code, out = run_cmd('rosnode list 2>/dev/null', timeout=2)
        if code != 0:
            return []
        return [x.strip() for x in out.splitlines() if x.strip()]


def rostopic_list():
    try:
        master = rosgraph.Master('/eggy_command_center')
        pubs, subs, srvs = master.getSystemState()
        names = set()
        for topic, _nodes in pubs:
            names.add(topic)
        for topic, _nodes in subs:
            names.add(topic)
        return sorted(names)
    except Exception:
        code, out = run_cmd('rostopic list 2>/dev/null', timeout=2)
        if code != 0:
            return []
        return [x.strip() for x in out.splitlines() if x.strip()]


def rostopic_pub_sub_state():
    try:
        master = rosgraph.Master('/eggy_command_center')
        pubs, subs, srvs = master.getSystemState()
        pub_map = {topic: nodes for topic, nodes in pubs}
        sub_map = {topic: nodes for topic, nodes in subs}
        return pub_map, sub_map
    except Exception:
        return {}, {}


class EggyCommandCenter:
    def __init__(self):
        self.pub_response = rospy.Publisher('/eggy/command/response', String, queue_size=20)
        self.pub_status = rospy.Publisher('/eggy/command/status', String, queue_size=1, latch=True)
        self.sub_request = rospy.Subscriber('/eggy/command/request', String, self.on_request, queue_size=10)
        self.pub_auto_mapping_request = rospy.Publisher(
            '/eggy/auto_mapping/request', String, queue_size=5)
        self.auto_mapping_status = {}
        self.sub_auto_mapping_status = rospy.Subscriber(
            '/eggy/auto_mapping/status', String,
            self.on_auto_mapping_status, queue_size=1)
        self.profile_status = {}
        self.sub_profile = rospy.Subscriber(
            '/eggy/nav_mode/status', String, self.on_profile_status, queue_size=1)
        self.status_rate = float(rospy.get_param('~status_rate', 1.0))
        self.allow_shell = bool(rospy.get_param('~allow_shell', False))
        self.camera_stream_topic = rospy.get_param(
            '~camera_stream_topic', '/camera/front/image_source/compressed')
        self.last_status = {}
        rospy.loginfo('eggy_command_center started: request=/eggy/command/request response=/eggy/command/response status=/eggy/command/status')

    def on_profile_status(self, msg):
        try:
            status = json.loads(msg.data)
            if (isinstance(status, dict) and
                    status.get('profile') in ('mapping', 'navigation', 'inspection')):
                self.profile_status = status
        except (TypeError, ValueError):
            rospy.logwarn_throttle(10.0, 'invalid authoritative profile status ignored')

    def on_auto_mapping_status(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict) and status.get('schema_version') == 1:
                self.auto_mapping_status = status
        except (TypeError, ValueError):
            rospy.logwarn_throttle(10.0, 'invalid automatic mapping status ignored')

    def now(self):
        return rospy.Time.now().to_sec()

    def make_response(self, req, success, message, details=None):
        return {
            'request_id': req.get('request_id', ''),
            'command': req.get('command', ''),
            'target': req.get('target', ''),
            'success': bool(success),
            'message': message,
            'details': details or {},
            'stamp': self.now(),
        }

    def publish_response(self, resp):
        self.pub_response.publish(String(json.dumps(resp, ensure_ascii=False)))

    def build_status(self):
        nodes = rosnode_list()
        topics = rostopic_list()
        pub_map, sub_map = rostopic_pub_sub_state()
        node_state = {name: (name in nodes) for name in KNOWN_NODES}
        node_state['/kimi_inspection_server'] = http_service_alive('127.0.0.1', 8000)
        topic_state = {name: {
            'exists': (name in topics),
            'publishers': pub_map.get(name, []),
            'subscribers': sub_map.get(name, []),
            'has_publisher': bool(pub_map.get(name, [])),
        } for name in KEY_TOPICS}

        authority = dict(self.profile_status)
        mode = authority.get('mode', 'unknown')
        profile = authority.get('profile', 'unknown')
        profile_state = authority.get('state', 'degraded')

        load1, load5, load15 = os.getloadavg()
        camera_pid_code, camera_pid = run_cmd("pgrep -f '[e]ggy_camera_node.py' | head -1", timeout=2)
        v4l2_code, v4l2_pid = run_cmd("pgrep -x v4l2-ctl | head -1", timeout=2)

        active_map = {}
        active_metadata = os.path.join(ACTIVE_MAP_LINK, 'metadata.json')
        try:
            with open(active_metadata, 'r', encoding='utf-8') as stream:
                active_map = json.load(stream)
            active_map['yaml'] = os.path.join(ACTIVE_MAP_LINK, 'map.yaml')
        except Exception:
            active_map = {}

        capabilities = authority.get('capabilities', {})

        return {
            'stamp': self.now(),
            'mode': mode,
            'profile': profile,
            'state': profile_state,
            'loadavg': [round(load1, 2), round(load5, 2), round(load15, 2)],
            'nodes': node_state,
            'topics': topic_state,
            'camera': {
                'running': node_state.get('/eggy_camera', False),
                'pid': camera_pid if camera_pid_code == 0 else '',
                'v4l2_pid': v4l2_pid if v4l2_code == 0 else '',
            },
            'active_map': active_map,
            'capabilities': capabilities,
            'auto_mapping': dict(self.auto_mapping_status),
            'profile_authority': '/eggy/nav_mode/status',
        }

    def wait_node_state(self, node_name, should_exist=True, timeout_sec=10.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            exists = node_name in rosnode_list()
            if exists == should_exist:
                return True
            time.sleep(0.5)
        return (node_name in rosnode_list()) == should_exist

    def handle_status(self, req):
        status = self.build_status()
        self.last_status = status
        return self.make_response(req, True, '状态获取成功', status)

    def handle_camera_start(self, req):
        code, out = run_cmd('eggy-camera-start', timeout=8)
        node_ok = self.wait_node_state('/eggy_camera', True, timeout_sec=12.0)
        frame_ok = False
        frame_error = ''
        if node_ok:
            try:
                rospy.wait_for_message(
                    self.camera_stream_topic, CompressedImage, timeout=8.0)
                frame_ok = True
            except rospy.ROSException as exc:
                frame_error = str(exc)
        ok = node_ok and frame_ok
        message = '摄像头图像流已就绪' if ok else (
            '摄像头节点在线但未收到图像' if node_ok else '摄像头启动失败')
        return self.make_response(req, ok, message, {
            'exit_code': code,
            'output': out,
            'eggy_camera': node_ok,
            'image_topic': self.camera_stream_topic,
            'frame_received': frame_ok,
            'frame_error': frame_error,
        })

    def handle_camera_stop(self, req):
        code, out = run_cmd('eggy-camera-stop', timeout=8)
        ok = self.wait_node_state('/eggy_camera', False, timeout_sec=5.0)
        return self.make_response(req, ok, '摄像头已停止' if ok else '摄像头停止后仍检测到节点', {
            'exit_code': code, 'output': out, 'eggy_camera': not ok
        })

    def handle_clear_costmaps(self, req):
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=3.0)
            srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            srv()
            return self.make_response(req, True, 'move_base costmaps 清理成功')
        except Exception as exc:
            return self.make_response(req, False, 'move_base costmaps 清理失败', {'error': str(exc)})

    def handle_mapping_stop(self, req):
        code, out = run_cmd('rosnode kill /slam_gmapping 2>&1 || true', timeout=8)
        time.sleep(1.0)
        ok = '/slam_gmapping' not in rosnode_list()
        return self.make_response(req, ok, 'gmapping 已停止' if ok else 'gmapping 停止失败', {
            'exit_code': code, 'output': out
        })

    def handle_mapping_start(self, req):
        if '/slam_gmapping' in rosnode_list():
            return self.make_response(req, True, 'gmapping 已在运行')
        cmd = 'nohup roslaunch eggy_bringup mapping_light.launch scan_topic:=/scan base_frame:=base_link odom_frame:=odom >/tmp/eggy_mapping_manual.log 2>&1 &'
        code, out = run_cmd(cmd, timeout=3)
        time.sleep(2.0)
        ok = '/slam_gmapping' in rosnode_list()
        return self.make_response(req, ok, 'gmapping 启动成功' if ok else 'gmapping 启动失败', {
            'exit_code': code, 'output': out, 'log': '/tmp/eggy_mapping_manual.log'
        })


    def handle_mapping_reset(self, req):
        """一键清空当前地图并重新开始建图：先杀 gmapping，再重启。"""
        already_running = '/slam_gmapping' in rosnode_list()
        if already_running:
            kill_code, kill_out = run_cmd('rosnode kill /slam_gmapping 2>&1 ' '|| true', timeout=8)
            time.sleep(1.5)
            if '/slam_gmapping' in rosnode_list():
                return self.make_response(req, False, 'gmapping 停止失败，未能清除地图', {
                    'exit_code': kill_code, 'output': kill_out
                })
            rospy.loginfo('mapping_reset: gmapping killed (exit=%s)', kill_code)
        else:
            kill_code, kill_out = 0, 'gmapping was not running'
        # 同时清理 move_base costmaps，避免旧地图残留
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=3.0)
            srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            srv()
        except Exception:
            pass
        cmd = ('nohup roslaunch eggy_bringup mapping_light.launch '
                'scan_topic:=/scan base_frame:=base_link odom_frame:=odom '
                '>/tmp/eggy_mapping_manual.log 2>&1 &')
        code, out = run_cmd(cmd, timeout=3)
        time.sleep(2.5)
        ok = '/slam_gmapping' in rosnode_list()
        return self.make_response(
            req, ok,
            '地图已清除，gmapping 已重新启动' if ok else 'gmapping 重启失败',
            {'exit_code': code, 'output': out,
             'log': '/tmp/eggy_mapping_manual.log',
             'was_running': already_running})

    def handle_map_save(self, req):
        """Atomically save the current /map into the canonical map library."""
        if '/map' not in rostopic_list():
            return self.make_response(req, False, '当前没有可保存的地图话题')
        os.makedirs(MAP_LIBRARY_ROOT, exist_ok=True)
        map_id = 'voice_' + time.strftime('%Y%m%d_%H%M%S')
        destination = os.path.join(MAP_LIBRARY_ROOT, map_id)
        if os.path.exists(destination):
            return self.make_response(req, False, '地图名称冲突，请稍后重试')
        staging = tempfile.mkdtemp(prefix='.staging-voice-', dir=MAP_LIBRARY_ROOT)
        prefix = os.path.join(staging, 'map')
        try:
            code, output = run_cmd(
                'rosrun map_server map_saver -f ' + shlex.quote(prefix), timeout=20)
            yaml_path = prefix + '.yaml'
            pgm_path = prefix + '.pgm'
            if code != 0 or not os.path.isfile(yaml_path) or not os.path.isfile(pgm_path):
                return self.make_response(req, False, '地图保存失败', {
                    'exit_code': code, 'output': output,
                })
            os.rename(yaml_path, os.path.join(staging, 'map.yaml'))
            os.rename(pgm_path, os.path.join(staging, 'map.pgm'))
            metadata = {
                'schema_version': 1,
                'map_id': map_id,
                'source': 'voice_v5',
                'created_at': time.time(),
            }
            with open(os.path.join(staging, 'metadata.json'), 'w', encoding='utf-8') as stream:
                json.dump(metadata, stream, ensure_ascii=False, indent=2)
            os.rename(staging, destination)
            staging = ''
            return self.make_response(req, True, '地图保存成功', {
                'map_id': map_id,
                'map_file': os.path.join(destination, 'map.yaml'),
            })
        except Exception as exc:
            return self.make_response(req, False, '地图保存失败', {'error': str(exc)})
        finally:
            if staging and os.path.isdir(staging):
                shutil.rmtree(staging, ignore_errors=True)

    def handle_list_maps(self, req):
        cmd = ("find " + shlex.quote(MAP_LIBRARY_ROOT) +
               " -mindepth 2 -maxdepth 2 -name 'map.yaml' "
               "-printf '%T@ %p\\n' | sort -nr | awk '{print $2}'")
        code, out = run_cmd(cmd, timeout=5)
        maps = [x.strip() for x in out.splitlines() if x.strip()] if code == 0 else []
        return self.make_response(req, bool(maps), '地图列表读取成功' if maps else '未找到地图文件', {
            'maps': maps,
        })

    def restart_system_stack(self, use_amcl, map_file=None):
        kill_cmd = (
            "pkill -TERM -f 'roslaunch.*eggy_bringup.*eggy_system\\.launch' 2>/dev/null || true; "
            "pkill -TERM -f '/opt/ros/noetic/lib/gmapping/slam_gmapping' 2>/dev/null || true; "
            "pkill -TERM -f '/opt/ros/noetic/lib/map_server/map_server' 2>/dev/null || true; "
            "pkill -TERM -f '/opt/ros/noetic/lib/amcl/amcl' 2>/dev/null || true; "
            "sleep 3; "
            "pkill -KILL -f 'roslaunch.*eggy_bringup.*eggy_system\\.launch' 2>/dev/null || true; "
            "pkill -KILL -f '/opt/ros/noetic/lib/gmapping/slam_gmapping' 2>/dev/null || true; "
            "pkill -KILL -f '/opt/ros/noetic/lib/map_server/map_server' 2>/dev/null || true; "
            "pkill -KILL -f '/opt/ros/noetic/lib/amcl/amcl' 2>/dev/null || true; "
            "sleep 1; "
        )
        run_cmd(kill_cmd, timeout=12)
        launch_cmd = (
            f"nohup roslaunch eggy_bringup eggy_system.launch "
            f"use_lidar:=true use_navigation:=true "
            f"use_amcl:={'true' if use_amcl else 'false'} "
            f"use_mapping:={'false' if use_amcl else 'true'} "
            f"use_cpp_odom_fuser:=true use_command_center:=false"
        )
        if use_amcl and map_file:
            launch_cmd += f" map_file:={map_file}"
        launch_cmd += " >/tmp/eggy_mode_restart.log 2>&1 &"
        run_cmd(launch_cmd, timeout=5)
        time.sleep(8.0)
        status = self.build_status()
        expected = 'static_nav' if use_amcl else 'mapping_slam'
        ok = status.get('mode') == expected
        return ok, status

    def _wait_pid_gone(self, pattern, timeout_sec=3.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            code, out = run_cmd("pgrep -f '" + pattern + "'", timeout=2)
            if code != 0:
                return True
            time.sleep(0.2)
        return False

    def _kill_by_names(self, names, timeout_sec=3.0):
        """Kill ROS node executables by full process path pattern.

        The previous shell assignment tried to accumulate multiple pgrep
        results in one command, but the generated command was not reliable.
        During AMCL switching it let the boot-time slam_gmapping process
        survive, leaving both gmapping and map_server publishing /map.
        """
        patterns = {
            'slam_gmapping': r'/opt/ros/noetic/lib/gmapping/slam_gmapping',
            'map_server': r'/opt/ros/noetic/lib/map_server/map_server',
            'amcl': r'/opt/ros/noetic/lib/amcl/amcl',
            'move_base': r'/opt/ros/noetic/lib/move_base/move_base',
        }
        selected = [patterns.get(name, name) for name in names]

        for pattern in selected:
            run_cmd("pkill -TERM -f '" + pattern + "' 2>/dev/null || true", timeout=2)
        time.sleep(min(timeout_sec, 2.0))
        for pattern in selected:
            run_cmd("pkill -KILL -f '" + pattern + "' 2>/dev/null || true", timeout=2)

    def _cleanup_ros_master(self):
        """Remove stale node registrations left in ROS master after process kills."""
        run_cmd("yes y | rosnode cleanup >/tmp/eggy_rosnode_cleanup.log 2>&1 || true", timeout=8)

    def _launch_detached(self, command, log_path):
        """Launch a ROS command outside run_cmd's timeout-managed process group."""
        env = os.environ.copy()
        env.setdefault('ROS_MASTER_URI', 'http://localhost:11311')
        wrapped = (
            "source /opt/ros/noetic/setup.bash; "
            "source /root/catkin_ws/devel/setup.bash 2>/dev/null || true; "
            "exec " + command
        )
        log = open(log_path, 'ab', buffering=0)
        subprocess.Popen(
            ['bash', '-lc', wrapped],
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            preexec_fn=os.setsid,
            close_fds=True,
        )

    def _wait_mode_ready(self, mode, timeout_sec=4.0):
        expected = 'static_nav' if mode == 'navigation' else 'mapping_slam'
        deadline = time.time() + max(0.5, timeout_sec)
        status = self.build_status()
        while time.time() < deadline and not rospy.is_shutdown():
            status = self.build_status()
            if status.get('mode') == expected:
                return True, status
            if mode == 'navigation':
                map_info = (status.get('topics') or {}).get('/map') or {}
                nodes = status.get('nodes') or {}
                if map_info.get('has_publisher') and nodes.get('/amcl', False):
                    status['mode'] = 'static_nav'
                    return True, status
            elif mode == 'mapping':
                nodes = status.get('nodes') or {}
                if nodes.get('/slam_gmapping', False):
                    status['mode'] = 'mapping_slam'
                    return True, status
            time.sleep(0.25)
        return status.get('mode') == expected, status

    def _wait_map_matches(self, yaml_path, timeout_sec=6.0):
        """Wait until /static_map serves the exact selected map metadata."""
        try:
            with open(yaml_path, 'r', encoding='utf-8') as stream:
                config = yaml.safe_load(stream) or {}
            image_path = str(config.get('image', '')).strip()
            if not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(yaml_path), image_path)
            image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
            if image is None:
                return False, {'error': '地图图像无法读取'}
            expected_height, expected_width = image.shape[:2]
            expected_resolution = float(config['resolution'])
            rospy.wait_for_service('/static_map', timeout=timeout_sec)
            grid = rospy.ServiceProxy('/static_map', GetMap)().map
            actual = {
                'width': int(grid.info.width),
                'height': int(grid.info.height),
                'resolution': float(grid.info.resolution),
                'frame_id': grid.header.frame_id,
            }
            matched = (
                actual['width'] == expected_width and
                actual['height'] == expected_height and
                abs(actual['resolution'] - expected_resolution) <= 1e-6 and
                actual['frame_id'] == 'map'
            )
            return matched, actual
        except Exception as exc:
            return False, {'error': str(exc)}

    def switch_mode_fast(self, mode, map_file=None):
        if mode not in ('mapping', 'navigation'):
            raise ValueError('unsupported mode')
        if mode == 'navigation' and not map_file:
            raise ValueError('map_file is required for navigation')

        stop_patterns = [
            r"roslaunch.*eggy_bringup.*mapping_light\.launch",
            r"roslaunch eggy_bringup move_base_only.launch",
            r"roslaunch eggy_bringup move_base_nav.launch",
            r"roslaunch eggy_bringup amcl.launch",
        ]
        exact_names = ['slam_gmapping', 'map_server', 'amcl', 'move_base']
        for pat in stop_patterns:
            run_cmd("pkill -TERM -f '" + pat + "' 2>/dev/null || true", timeout=2)
        self._kill_by_names(exact_names, timeout_sec=2.0)
        for pat in stop_patterns:
            self._wait_pid_gone(pat, 3.0)
        for pat in stop_patterns:
            run_cmd("pkill -KILL -f '" + pat + "' 2>/dev/null || true", timeout=2)
        self._kill_by_names(exact_names, timeout_sec=0.5)
        self._cleanup_ros_master()

        if mode == 'mapping':
            self._launch_detached(
                'roslaunch eggy_bringup mapping_light.launch scan_topic:=/scan base_frame:=base_link odom_frame:=odom',
                '/tmp/eggy_mode_switch_mapping.log')
            self._launch_detached(
                'roslaunch eggy_bringup move_base_only.launch',
                '/tmp/eggy_mode_switch_movebase.log')
        else:
            quoted_map = shlex.quote(map_file)
            self._launch_detached(
                'rosrun map_server map_server ' + quoted_map,
                '/tmp/eggy_mode_switch_mapserver.log')
            map_ok, map_details = self._wait_map_matches(map_file, timeout_sec=6.0)
            if not map_ok:
                return False, {
                    'mode': 'unknown',
                    'map_ready': False,
                    'map_details': map_details,
                }
            self._launch_detached(
                'roslaunch eggy_bringup amcl.launch scan_topic:=/scan odom_frame_id:=odom base_frame_id:=base_link map:=/map',
                '/tmp/eggy_mode_switch_amcl.log')
            ok, _status = self._wait_mode_ready('navigation', timeout_sec=2.0)
            self._launch_detached(
                'roslaunch eggy_bringup move_base_nav.launch',
                '/tmp/eggy_mode_switch_movebase.log')

        time.sleep(0.5)
        self._cleanup_ros_master()
        expected = 'static_nav' if mode == 'navigation' else 'mapping_slam'
        ok, status = self._wait_mode_ready(mode, timeout_sec=4.0)
        if mode == 'mapping' and status.get('mode') == 'unknown':
            status['mode'] = self.last_status.get('mode', 'mapping_slam') if hasattr(self, 'last_status') else 'mapping_slam'
            expected = status['mode']
        if ok and status.get('mode') != expected:
            status['mode'] = expected
        return ok, status

    def handle_switch_nav_mode(self, req):
        params = req.get('params') or {}
        mode = str(params.get('mode', '')).strip().lower()
        map_file = str(params.get('map_file', '')).strip()

        if mode not in ('mapping', 'navigation'):
            return self.make_response(req, False, '未知模式，需要 mapping 或 navigation')

        if mode == 'navigation' and not map_file:
            return self.make_response(req, False, '切到 navigation 时必须提供 params.map_file')

        try:
            ok, status = self.switch_mode_fast(mode, map_file=map_file or None)
            return self.make_response(
                req, ok,
                '已切到固定地图导航模式' if mode == 'navigation' else '已切回建图模式',
                {
                    'requested_mode': mode,
                    'map_file': map_file,
                    'status': status,
                }
            )
        except Exception as exc:
            return self.make_response(req, False, '导航模式切换失败', {'error': str(exc)})

    def handle_switch_profile(self, req):
        params = req.get('params') or {}
        profile = str(params.get('profile', params.get('mode', ''))).strip().lower()
        map_file = str(params.get('map_file', '')).strip()
        if profile not in ('mapping', 'navigation', 'inspection'):
            return self.make_response(req, False, '未知 profile，需要 mapping、navigation 或 inspection')
        if profile in ('navigation', 'inspection') and not map_file:
            active_yaml = os.path.join(ACTIVE_MAP_LINK, 'map.yaml')
            if os.path.isfile(active_yaml):
                map_file = active_yaml
            else:
                return self.make_response(req, False, '尚未激活地图，请先从 Qt 顶部“打开地图”')

        args = ['--' + ('amcl' if profile == 'navigation' else profile)]
        if map_file:
            args.extend(['--map', shlex.quote(map_file)])
        command = '/usr/local/bin/eggy-stack-start ' + ' '.join(args)
        transition = self.build_status()
        transition.update({
            'state': 'switching',
            'requested_profile': profile,
            'request_id': req.get('request_id', ''),
        })
        transition['capabilities']['profile_switch'] = False
        self.pub_status.publish(String(json.dumps(transition, ensure_ascii=False)))
        rospy.loginfo('profile switch accepted: request_id=%s profile=%s map=%s',
                      req.get('request_id', ''), profile, map_file or '-')
        self._launch_detached(command, '/tmp/eggy_profile_switch.log')
        return self.make_response(req, True, '已请求切换运行 profile，等待节点重新上线', {
            'requested_profile': profile,
            'map_file': map_file,
            'restart': True,
            'accepted': True,
            'state': 'switching',
        })

    def handle_upload_map(self, req):
        params = req.get('params') or {}
        map_name = str(params.get('map_name', '')).strip()
        yaml_b64 = str(params.get('yaml_b64', '')).strip()
        pgm_b64 = str(params.get('pgm_b64', '')).strip()
        activate = bool(params.get('activate', True))

        if not map_name or not yaml_b64 or not pgm_b64:
            return self.make_response(req, False, '缺少 map_name / yaml_b64 / pgm_b64')

        safe_name = map_name.lower()
        if not re.fullmatch(r'[a-z0-9][a-z0-9_-]{0,47}', safe_name):
            return self.make_response(req, False, '地图名称无效：仅允许小写字母、数字、下划线和短横线，最长 48 字符')

        try:
            yaml_bytes = base64.b64decode(yaml_b64, validate=True)
            pgm_bytes = base64.b64decode(pgm_b64, validate=True)
        except Exception as exc:
            return self.make_response(req, False, '地图文件解码失败', {'error': str(exc)})

        try:
            yaml_text = yaml_bytes.decode('utf-8')
        except Exception as exc:
            return self.make_response(req, False, 'YAML 解码为文本失败', {'error': str(exc)})

        if pgm_bytes[:2] not in (b'P5', b'P6'):
            return self.make_response(req, False, 'PGM 文件头校验失败，需要标准 PGM 文件')

        digest = hashlib.sha256(yaml_bytes + b'\0' + pgm_bytes).hexdigest()
        map_id = safe_name + '_' + digest[:8]
        os.makedirs(MAP_LIBRARY_ROOT, exist_ok=True)
        dest_dir = os.path.join(MAP_LIBRARY_ROOT, map_id)
        yaml_path = os.path.join(dest_dir, 'map.yaml')
        pgm_path = os.path.join(dest_dir, 'map.pgm')
        staging_dir = tempfile.mkdtemp(prefix='.staging-', dir=MAP_LIBRARY_ROOT)
        try:
            parsed = yaml.safe_load(yaml_text) or {}
            required = ('resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh')
            missing = [key for key in required if key not in parsed]
            if missing or not isinstance(parsed.get('origin'), list) or len(parsed['origin']) != 3:
                raise ValueError('YAML 缺少必需字段或 origin 格式错误: ' + ','.join(missing))
            parsed['image'] = './map.pgm'
            stage_yaml = os.path.join(staging_dir, 'map.yaml')
            stage_pgm = os.path.join(staging_dir, 'map.pgm')
            with open(stage_yaml, 'w', encoding='utf-8') as f:
                yaml.safe_dump(parsed, f, allow_unicode=True, sort_keys=False)
                f.flush()
                os.fsync(f.fileno())
            with open(stage_pgm, 'wb') as f:
                f.write(pgm_bytes)
                f.flush()
                os.fsync(f.fileno())
            metadata = {
                'schema_version': 1,
                'map_id': map_id,
                'display_name': safe_name,
                'sha256': digest,
                'created_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                'source': 'qt',
            }
            with open(os.path.join(staging_dir, 'metadata.json'), 'w', encoding='utf-8') as f:
                json.dump(metadata, f, ensure_ascii=False, indent=2)
                f.flush()
                os.fsync(f.fileno())
            if os.path.isdir(dest_dir):
                shutil.rmtree(staging_dir)
            else:
                os.replace(staging_dir, dest_dir)
        except Exception as exc:
            shutil.rmtree(staging_dir, ignore_errors=True)
            return self.make_response(req, False, '地图校验或原子写入失败', {'error': str(exc)})

        if not activate:
            return self.make_response(req, True, '地图已上传并保存', {
                'yaml': yaml_path,
                'pgm': pgm_path,
                'map_id': map_id,
                'sha256': digest,
            })

        try:
            ok, status = self.switch_mode_fast("navigation", map_file=yaml_path)
            if ok:
                temp_link = ACTIVE_MAP_LINK + '.tmp'
                if os.path.lexists(temp_link):
                    os.unlink(temp_link)
                os.symlink(dest_dir, temp_link)
                os.replace(temp_link, ACTIVE_MAP_LINK)
            return self.make_response(req, ok, '已上传地图并切到 AMCL 导航模式' if ok else '地图上传成功，但切换导航模式失败', {
                'yaml': yaml_path,
                'pgm': pgm_path,
                'map_id': map_id,
                'sha256': digest,
                'map_ready': ok,
                'status': status,
            })
        except Exception as exc:
            return self.make_response(req, False, '地图上传后切导航模式失败', {
                'yaml': yaml_path,
                'pgm': pgm_path,
                'error': str(exc),
            })

    def handle_get_param(self, req):
        params = req.get('params') or {}
        name = str(params.get('name', '')).strip()
        if not name:
            return self.make_response(req, False, '缺少 params.name')
        try:
            value = rospy.get_param(name)
            return self.make_response(req, True, '参数读取成功', {'name': name, 'value': value})
        except Exception as exc:
            return self.make_response(req, False, '参数读取失败', {'name': name, 'error': str(exc)})

    def handle_set_param(self, req):
        params = req.get('params') or {}
        name = str(params.get('name', '')).strip()
        if not name:
            return self.make_response(req, False, '缺少 params.name')
        if 'value' not in params:
            return self.make_response(req, False, '缺少 params.value')
        value = params.get('value')
        try:
            rospy.set_param(name, value)
            actual = rospy.get_param(name)
            return self.make_response(req, True, '参数已写入 ROS parameter server；注意部分节点需 dynamic_reconfigure 或重启才会生效', {
                'name': name, 'value': value, 'actual': actual
            })
        except Exception as exc:
            return self.make_response(req, False, '参数写入失败', {'name': name, 'value': value, 'error': str(exc)})

    def handle_dyn_get(self, req):
        params = req.get('params') or {}
        namespace = str(params.get('namespace', '')).strip()
        name = str(params.get('name', '')).strip()
        if not namespace or not name:
            return self.make_response(req, False, '缺少 params.namespace 或 params.name')
        try:
            client = dynamic_reconfigure.client.Client(namespace, timeout=3.0)
            cfg = client.get_configuration(timeout=3.0)
            if name not in cfg:
                return self.make_response(req, False, '动态参数不存在', {
                    'namespace': namespace, 'name': name,
                    'available': sorted([k for k in cfg.keys() if not k.startswith('_')])[:80]
                })
            return self.make_response(req, True, '动态参数读取成功', {
                'namespace': namespace, 'name': name, 'value': cfg.get(name)
            })
        except Exception as exc:
            return self.make_response(req, False, '动态参数读取失败', {
                'namespace': namespace, 'name': name, 'error': str(exc)
            })

    def handle_dyn_set(self, req):
        params = req.get('params') or {}
        namespace = str(params.get('namespace', '')).strip()
        name = str(params.get('name', '')).strip()
        if not namespace or not name:
            return self.make_response(req, False, '缺少 params.namespace 或 params.name')
        if 'value' not in params:
            return self.make_response(req, False, '缺少 params.value')
        value = params.get('value')
        try:
            client = dynamic_reconfigure.client.Client(namespace, timeout=3.0)
            result = client.update_configuration({name: value})
            actual = result.get(name, None)
            return self.make_response(req, True, '动态参数已应用到运行中节点', {
                'namespace': namespace, 'name': name, 'value': value, 'actual': actual
            })
        except Exception as exc:
            return self.make_response(req, False, '动态参数写入失败', {
                'namespace': namespace, 'name': name, 'value': value, 'error': str(exc)
            })

    def handle_dyn_get_many(self, req):
        params = req.get('params') or {}
        items = params.get('items') or []
        if not isinstance(items, list) or not items:
            return self.make_response(req, False, '缺少 params.items 数组')
        results = []
        all_ok = True
        clients = {}
        for item in items:
            namespace = str(item.get('namespace', '')).strip()
            name = str(item.get('name', '')).strip()
            if not namespace or not name:
                all_ok = False
                results.append({'success': False, 'namespace': namespace, 'name': name, 'error': '缺少 namespace/name'})
                continue
            try:
                if namespace not in clients:
                    clients[namespace] = dynamic_reconfigure.client.Client(namespace, timeout=3.0)
                cfg = clients[namespace].get_configuration(timeout=3.0)
                if name not in cfg:
                    all_ok = False
                    results.append({'success': False, 'namespace': namespace, 'name': name, 'error': '动态参数不存在'})
                else:
                    results.append({'success': True, 'namespace': namespace, 'name': name, 'value': cfg.get(name)})
            except Exception as exc:
                all_ok = False
                results.append({'success': False, 'namespace': namespace, 'name': name, 'error': str(exc)})
        return self.make_response(req, all_ok, '参数组合读取成功' if all_ok else '参数组合部分读取失败', {
            'results': results
        })

    def handle_dyn_set_many(self, req):
        params = req.get('params') or {}
        items = params.get('items') or []
        if not isinstance(items, list) or not items:
            return self.make_response(req, False, '缺少 params.items 数组')
        results = []
        all_ok = True
        clients = {}
        for item in items:
            namespace = str(item.get('namespace', '')).strip()
            name = str(item.get('name', '')).strip()
            value = item.get('value')
            if not namespace or not name or 'value' not in item:
                all_ok = False
                results.append({'success': False, 'namespace': namespace, 'name': name, 'error': '缺少 namespace/name/value'})
                continue
            try:
                if namespace not in clients:
                    clients[namespace] = dynamic_reconfigure.client.Client(namespace, timeout=3.0)
                result = clients[namespace].update_configuration({name: value})
                results.append({'success': True, 'namespace': namespace, 'name': name, 'value': value, 'actual': result.get(name, None)})
            except Exception as exc:
                all_ok = False
                results.append({'success': False, 'namespace': namespace, 'name': name, 'value': value, 'error': str(exc)})
        return self.make_response(req, all_ok, '参数组合已全部应用' if all_ok else '参数组合部分应用失败', {
            'results': results
        })

    def handle_shell(self, req):
        if not self.allow_shell:
            return self.make_response(req, False, 'shell 命令默认禁用，请在节点参数 ~allow_shell=true 后使用')
        params = req.get('params') or {}
        cmd = str(params.get('cmd', '')).strip()
        if not cmd:
            return self.make_response(req, False, '缺少 params.cmd')
        code, out = run_cmd(cmd, timeout=float(params.get('timeout', 10)))
        return self.make_response(req, code == 0, 'shell 执行完成' if code == 0 else 'shell 执行失败', {
            'exit_code': code, 'output': out[-4000:]
        })

    def handle_auto_mapping(self, req, action):
        if action == 'status':
            return self.make_response(
                req, bool(self.auto_mapping_status),
                '自动建图状态已返回' if self.auto_mapping_status else '尚未收到自动建图状态',
                self.auto_mapping_status)
        if action == 'start':
            profile = str(self.profile_status.get('profile', '')).strip().lower()
            state = str(self.profile_status.get('state', '')).strip().lower()
            if profile != 'mapping' or state not in ('ready', 'running'):
                return self.make_response(
                    req, False, '请先切换到可用的建图模式',
                    {'profile_status': self.profile_status})
        params = req.get('params') or {}
        if not isinstance(params, dict):
            return self.make_response(req, False, 'params 必须是对象')
        allowed = {
            'max_duration_sec', 'max_linear_speed', 'return_home',
            'save_draft_on_abort', 'min_frontier_cells'
        }
        options = {key: value for key, value in params.items() if key in allowed}
        payload = {
            'schema_version': 1,
            'request_id': str(req.get('request_id', '')),
            'command': action,
            'options': options,
        }
        self.pub_auto_mapping_request.publish(
            String(json.dumps(payload, ensure_ascii=False)))
        return self.make_response(
            req, True, '自动建图指令已受理',
            {'accepted': True, 'automatic_mapping_request': payload})

    def dispatch(self, req):
        command = str(req.get('command', '')).strip()
        target = str(req.get('target', '')).strip()
        key = command if command else target

        if key in ('status', 'system_status'):
            return self.handle_status(req)
        if key in ('camera_start', 'start_camera') or (command == 'module_start' and target == 'camera'):
            return self.handle_camera_start(req)
        if key in ('camera_stop', 'stop_camera') or (command == 'module_stop' and target == 'camera'):
            return self.handle_camera_stop(req)
        if key in ('clear_costmaps', 'clear_costmap'):
            return self.handle_clear_costmaps(req)
        if key in ('mapping_stop', 'stop_mapping') or (command == 'module_stop' and target == 'mapping'):
            return self.handle_mapping_stop(req)
        if key in ('mapping_start', 'start_mapping') or (command == 'module_start' and target == 'mapping'):
            return self.handle_mapping_start(req)
        if key in ('mapping_reset', 'reset_mapping', 'clear_mapping'):
            return self.handle_mapping_reset(req)
        if key in ('map_save', 'save_map'):
            return self.handle_map_save(req)
        if key in ('auto_mapping_start', 'start_auto_mapping'):
            return self.handle_auto_mapping(req, 'start')
        if key in ('auto_mapping_stop', 'stop_auto_mapping', 'auto_mapping_cancel'):
            return self.handle_auto_mapping(req, 'cancel')
        if key == 'auto_mapping_pause':
            return self.handle_auto_mapping(req, 'pause')
        if key == 'auto_mapping_resume':
            return self.handle_auto_mapping(req, 'resume')
        if key == 'auto_mapping_status':
            return self.handle_auto_mapping(req, 'status')
        if key == 'list_maps':
            return self.handle_list_maps(req)
        if key == 'switch_nav_mode':
            return self.handle_switch_nav_mode(req)
        if key == 'switch_profile':
            return self.handle_switch_profile(req)
        if key == 'upload_map':
            return self.handle_upload_map(req)
        if key == 'get_param':
            return self.handle_get_param(req)
        if key == 'set_param':
            return self.handle_set_param(req)
        if key == 'dyn_get':
            return self.handle_dyn_get(req)
        if key == 'dyn_set':
            return self.handle_dyn_set(req)
        if key == 'dyn_get_many':
            return self.handle_dyn_get_many(req)
        if key == 'dyn_set_many':
            return self.handle_dyn_set_many(req)
        if key == 'shell':
            return self.handle_shell(req)
        if key == 'set_mode':
            return self.make_response(req, False, 'set_mode 暂未自动执行：后续将接入 mapping/static_nav 栈切换；当前先用 status/模块控制做调试闭环')
        return self.make_response(req, False, '未知命令', {'supported': [
            'status', 'camera_start', 'camera_stop', 'clear_costmaps',
            'mapping_start', 'mapping_stop', 'mapping_reset', 'map_save', 'list_maps',
            'auto_mapping_start', 'auto_mapping_stop', 'auto_mapping_pause',
            'auto_mapping_resume', 'auto_mapping_status',
            'switch_nav_mode', 'switch_profile', 'upload_map',
            'get_param', 'set_param', 'dyn_get', 'dyn_set', 'dyn_get_many', 'dyn_set_many'
        ]})

    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
            if not isinstance(req, dict):
                raise ValueError('request JSON must be object')
        except Exception as exc:
            self.publish_response({
                'request_id': '', 'command': '', 'target': '', 'success': False,
                'message': '请求 JSON 解析失败', 'details': {'error': str(exc), 'raw': msg.data[:1000]},
                'stamp': self.now(),
            })
            return

        request_id = str(req.get('request_id', '')).strip()
        if not request_id:
            request_id = 'legacy-' + uuid.uuid4().hex
            req['request_id'] = request_id
            rospy.logwarn('command request missing request_id; assigned %s', request_id)

        try:
            resp = self.dispatch(req)
        except Exception as exc:
            resp = self.make_response(req, False, '命令执行异常', {'error': str(exc)})
        self.publish_response(resp)

    def spin(self):
        rate = rospy.Rate(self.status_rate)
        while not rospy.is_shutdown():
            try:
                status = self.build_status()
                self.last_status = status
                self.pub_status.publish(String(json.dumps(status, ensure_ascii=False)))
            except Exception as exc:
                rospy.logwarn('build status failed: %s', exc)
            rate.sleep()


def main():
    rospy.init_node('eggy_command_center')
    EggyCommandCenter().spin()


if __name__ == '__main__':
    main()
