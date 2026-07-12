#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
eggy_health_aggregator.py — Firefly 小车聚合健康监控节点

在 /diagnostics (diagnostic_msgs/DiagnosticArray) 上发布全车健康状态，
Qt 客户端订阅后可一屏看全所有子系统状况。

监控项 (每条 = 一个 DiagnosticStatus):
  1. firefly/系统资源   CPU负载 / 内存 / 磁盘 / CPU温度
  2. firefly/ROS节点    关键节点存活检查
  3. firefly/电池       订阅 /battery 或 /battery/voltage，低压告警
  4. firefly/激光雷达    订阅 /scan，按更新频率/超时判断在线
  5. firefly/里程计      订阅 /odom，超时判断
  6. firefly/底盘急停    订阅 /base/flag_stop，急停状态
  7. firefly/导航        订阅 /move_base/status，当前导航状态

level: 0=OK 1=WARN 2=ERROR 3=STALE
注: /diagnostics 上 stm32_base_driver 也在发底盘串口诊断，多发布者共存聚合。
"""
import rospy
import socket
import threading
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import LaserScan, BatteryState, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt8
from actionlib_msgs.msg import GoalStatusArray

# ---- 阈值 (可按需改) ----
MEM_WARN_PCT = 85.0
DISK_WARN_PCT = 90.0
TEMP_WARN_C = 75.0
TEMP_ERR_C = 85.0
BATT_WARN_V = 11.0      # 3S 锂电，低于 11V 提醒
BATT_ERR_V = 10.5       # 低于 10.5V 严重
SCAN_TIMEOUT_S = 1.0    # /scan 超过 1s 无更新视为离线
ODOM_TIMEOUT_S = 1.0
CAMERA_TIMEOUT_S = 2.0
KEY_NODES = ['/move_base', '/rplidarNode', '/stm32_base_driver',
             '/eggy_external_imu_odom_fuser',
             '/ros_qt5_gui_adapter', '/rosbridge_websocket']

MOVE_BASE_STATUS = {
    0: 'PENDING', 1: 'ACTIVE(导航中)', 2: 'PREEMPTED', 3: 'SUCCEEDED(已到达)',
    4: 'ABORTED(失败)', 5: 'REJECTED', 6: 'PREEMPTING', 7: 'RECALLING',
    8: 'RECALLED', 9: 'LOST'}


def kv(k, v):
    return KeyValue(key=str(k), value=str(v))


def http_service_alive(host, port):
    try:
        with socket.create_connection((host, int(port)), timeout=0.25):
            return True
    except (OSError, ValueError):
        return False


def make_status(level, name, message, hardware_id, pairs):
    st = DiagnosticStatus()
    st.level = level
    st.name = name
    st.message = message
    st.hardware_id = hardware_id
    st.values = [kv(k, v) for k, v in pairs]
    return st


class HealthAggregator:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_scan = None
        self.last_odom = None
        self.last_camera = None
        self.camera_frame_id = ''
        self.battery_v = None
        self.flag_stop = None
        self.nav_status = None      # 最近一个 GoalStatus 的 status
        self.nav_text = ''
        self.nav_status_time = None
        # 收编其它发布者(如 stm32_base_driver)发到 /diagnostics 的状态，
        # 缓存后并入自己的完整数组统一发布，避免 Qt 端两套数组交替跳闪。
        self.external_status = {}   # name -> (DiagnosticStatus, last_seen_time)

        rospy.Subscriber('/stm32/diagnostics', DiagnosticArray, self._on_diag, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self._on_scan, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber('/camera/front/image/compressed', CompressedImage, self._on_camera, queue_size=1)
        rospy.Subscriber('/battery', BatteryState, self._on_battery, queue_size=1)
        rospy.Subscriber('/battery/voltage', Float32, self._on_voltage, queue_size=1)
        rospy.Subscriber('/base/flag_stop', UInt8, self._on_flag, queue_size=1)
        rospy.Subscriber('/move_base/status', GoalStatusArray,
                         self._on_navstatus, queue_size=1)

        self.pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

    # ---- 回调 ----
    def _on_diag(self, msg):
        """收编 STM32 原始诊断，并改成 Qt 友好的显示名称"""
        now = rospy.Time.now()
        with self.lock:
            for src in msg.status:
                st = DiagnosticStatus()
                st.level = src.level
                st.name = '串口通信'
                st.message = src.message
                st.hardware_id = '底盘STM32'
                st.values = list(src.values)
                self.external_status[st.name] = (st, now)

    def _on_scan(self, msg):
        with self.lock:
            self.last_scan = rospy.Time.now()

    def _on_odom(self, msg):
        with self.lock:
            self.last_odom = rospy.Time.now()

    def _on_camera(self, msg):
        with self.lock:
            self.last_camera = rospy.Time.now()
            self.camera_frame_id = msg.header.frame_id

    def _on_battery(self, msg):
        with self.lock:
            if msg.voltage == msg.voltage and msg.voltage > 0:  # 非 NaN
                self.battery_v = float(msg.voltage)

    def _on_voltage(self, msg):
        with self.lock:
            self.battery_v = float(msg.data)

    def _on_flag(self, msg):
        with self.lock:
            self.flag_stop = int(msg.data)

    def _on_navstatus(self, msg):
        with self.lock:
            if msg.status_list:
                s = msg.status_list[-1]
                if s.status != self.nav_status:
                    self.nav_status_time = rospy.Time.now()
                self.nav_status = s.status
                self.nav_text = s.text
            else:
                self.nav_status = None
                self.nav_text = ''
                self.nav_status_time = None

    # ---- 系统信息 ----
    def _read_system(self):
        load1 = 'n/a'
        try:
            with open('/proc/loadavg') as f:
                load1 = f.read().split()[0]
        except Exception:
            pass
        mem_pct = None
        try:
            mi = {}
            with open('/proc/meminfo') as f:
                for line in f:
                    p = line.split()
                    mi[p[0].rstrip(':')] = int(p[1])
            tot = mi.get('MemTotal', 0)
            av = mi.get('MemAvailable', 0)
            if tot:
                mem_pct = round((tot - av) * 100.0 / tot, 1)
        except Exception:
            pass
        temp_c = None
        try:
            temps = []
            import glob
            for zp in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
                with open(zp) as f:
                    temps.append(int(f.read().strip()) / 1000.0)
            if temps:
                temp_c = round(max(temps), 1)
        except Exception:
            pass
        disk_pct = None
        try:
            import shutil
            t, u, fr = shutil.disk_usage('/')
            disk_pct = round(u * 100.0 / t, 1)
        except Exception:
            pass
        return load1, mem_pct, temp_c, disk_pct

    # ---- 构造各 status ----
    def build_array(self):
        now = rospy.Time.now()
        arr = DiagnosticArray()
        arr.header.stamp = now
        arr.header.frame_id = ''

        # 1. 系统资源
        load1, mem_pct, temp_c, disk_pct = self._read_system()
        lvl = DiagnosticStatus.OK
        msgs = []
        if mem_pct is not None and mem_pct > MEM_WARN_PCT:
            lvl = max(lvl, DiagnosticStatus.WARN); msgs.append('内存偏高')
        if disk_pct is not None and disk_pct > DISK_WARN_PCT:
            lvl = max(lvl, DiagnosticStatus.WARN); msgs.append('磁盘将满')
        if temp_c is not None:
            if temp_c > TEMP_ERR_C:
                lvl = max(lvl, DiagnosticStatus.ERROR); msgs.append('CPU过热')
            elif temp_c > TEMP_WARN_C:
                lvl = max(lvl, DiagnosticStatus.WARN); msgs.append('CPU偏热')
        arr.status.append(make_status(
            lvl, '系统资源', '正常' if lvl == 0 else '/'.join(msgs),
            '主控Firefly', [('CPU负载(1min)', load1),
                        ('内存使用率%', mem_pct if mem_pct is not None else 'n/a'),
                        ('CPU温度℃', temp_c if temp_c is not None else 'n/a'),
                        ('磁盘使用率%', disk_pct if disk_pct is not None else 'n/a')]))

        # 2. ROS 节点存活
        try:
            import rosnode
            alive = set(rosnode.get_node_names())
        except Exception:
            alive = set()
        map_server_alive = any(
            n == '/map_server' or n.startswith('/map_server_') for n in alive)
        amcl_mode = '/amcl' in alive or map_server_alive
        inspection_nodes = ['/meter_rknn_detect_cpp', '/kimi_inspection_server',
                            '/kimi_inspection_bridge', '/inspection_servo_route_runner']
        kimi_server_alive = http_service_alive('127.0.0.1', 8000)
        inspection_mode = amcl_mode and (
            any(n in alive for n in inspection_nodes if n != '/kimi_inspection_server')
            or kimi_server_alive
        )
        mode_nodes = ['/amcl'] if amcl_mode else ['/slam_gmapping']
        required_nodes = KEY_NODES + mode_nodes
        pairs = [('定位模式', '巡检' if inspection_mode else ('AMCL' if amcl_mode else '建图'))]
        if amcl_mode:
            pairs.append(('/map_server', '在线' if map_server_alive else '掉线'))
        missing = []
        if amcl_mode and not map_server_alive:
            missing.append('/map_server')
        for n in required_nodes:
            ok = n in alive
            pairs.append((n, '在线' if ok else '掉线'))
            if not ok:
                missing.append(n)
        if inspection_mode:
            for n in inspection_nodes:
                ok = kimi_server_alive if n == '/kimi_inspection_server' else n in alive
                pairs.append((n, '在线' if ok else '掉线'))
                if not ok:
                    missing.append(n)
        nlvl = DiagnosticStatus.ERROR if missing else DiagnosticStatus.OK
        arr.status.append(make_status(
            nlvl, 'ROS节点',
            '全部在线' if not missing else ('掉线: ' + ','.join(missing)),
            '主控Firefly', pairs))

        # 3. 电池
        with self.lock:
            bv = self.battery_v
        if bv is None:
            arr.status.append(make_status(
                DiagnosticStatus.STALE, '电池电压', '无电压数据',
                '电源', [('电压V', 'n/a')]))
        else:
            blvl = DiagnosticStatus.OK; bmsg = '正常'
            if bv < BATT_ERR_V:
                blvl = DiagnosticStatus.ERROR; bmsg = '电量严重不足!'
            elif bv < BATT_WARN_V:
                blvl = DiagnosticStatus.WARN; bmsg = '电量偏低'
            arr.status.append(make_status(
                blvl, '电池电压', bmsg, '电源',
                [('电压V', round(bv, 2)), ('告警阈值V', BATT_WARN_V)]))

        # 4. 激光雷达 (按 /scan 更新时效)
        with self.lock:
            ls = self.last_scan
        if ls is None:
            arr.status.append(make_status(
                DiagnosticStatus.STALE, '激光雷达', '无scan数据',
                '传感器', [('scan_age_s', 'n/a')]))
        else:
            age = (now - ls).to_sec()
            llvl = DiagnosticStatus.OK if age < SCAN_TIMEOUT_S else DiagnosticStatus.ERROR
            arr.status.append(make_status(
                llvl, '激光雷达', '在线' if llvl == 0 else '超时无数据',
                '传感器', [('scan_age_s', round(age, 3))]))

        # 5. 摄像头
        with self.lock:
            lc = self.last_camera
            camera_frame = self.camera_frame_id
        camera_node_alive = '/eggy_camera' in alive
        if not inspection_mode:
            arr.status.append(make_status(
                DiagnosticStatus.OK, '摄像头', '非巡检模式，摄像头不作为必需项',
                '视觉', [('node', 'online' if camera_node_alive else 'disabled'),
                       ('required', 'false'),
                       ('image_topic', '/camera/front/image/compressed')]))
        elif lc is None:
            clvl = DiagnosticStatus.STALE if camera_node_alive else DiagnosticStatus.ERROR
            arr.status.append(make_status(
                clvl, '摄像头', '节点在线但无图像' if camera_node_alive else '摄像头节点离线',
                '视觉', [('node', 'online' if camera_node_alive else 'offline'),
                       ('image_topic', '/camera/front/image/compressed'),
                       ('frame_age_s', 'n/a')]))
        elif inspection_mode:
            age = (now - lc).to_sec()
            clvl = DiagnosticStatus.OK if camera_node_alive and age < CAMERA_TIMEOUT_S else DiagnosticStatus.ERROR
            if not camera_node_alive:
                cmsg = '摄像头节点离线'
            elif age >= CAMERA_TIMEOUT_S:
                cmsg = '图像流超时'
            else:
                cmsg = '在线'
            arr.status.append(make_status(
                clvl, '摄像头', cmsg,
                '视觉', [('node', 'online' if camera_node_alive else 'offline'),
                       ('image_topic', '/camera/front/image/compressed'),
                       ('frame_age_s', round(age, 3)),
                       ('frame_id', camera_frame or 'n/a')]))
        # 5. 里程计
        with self.lock:
            lo = self.last_odom
        if lo is None:
            arr.status.append(make_status(
                DiagnosticStatus.STALE, '里程计', '无odom数据',
                '定位', [('odom_age_s', 'n/a')]))
        else:
            age = (now - lo).to_sec()
            olvl = DiagnosticStatus.OK if age < ODOM_TIMEOUT_S else DiagnosticStatus.ERROR
            arr.status.append(make_status(
                olvl, '里程计', '在线' if olvl == 0 else '超时无数据',
                '定位', [('odom_age_s', round(age, 3))]))

        # 6. 底盘急停
        with self.lock:
            fs = self.flag_stop
        if fs is None:
            arr.status.append(make_status(
                DiagnosticStatus.STALE, '急停状态', '无数据',
                '底盘STM32', [('flag_stop', 'n/a')]))
        else:
            flvl = DiagnosticStatus.WARN if fs else DiagnosticStatus.OK
            arr.status.append(make_status(
                flvl, '急停状态', '急停触发!' if fs else '正常',
                '底盘STM32', [('flag_stop', fs)]))

        # 7. 导航状态
        with self.lock:
            ns = self.nav_status; nt = self.nav_text; nst = self.nav_status_time
        if ns is not None and ns != 1 and nst is not None and (
                now - nst).to_sec() > 5.0:
            ns = None
        if ns is None:
            arr.status.append(make_status(
                DiagnosticStatus.OK, 'move_base状态', '空闲(无目标)',
                '导航', [('status', 'IDLE')]))
        else:
            txt = MOVE_BASE_STATUS.get(ns, str(ns))
            nlvl2 = DiagnosticStatus.OK
            if ns == 4:        # ABORTED
                nlvl2 = DiagnosticStatus.ERROR
            elif ns in (2, 5, 9):
                nlvl2 = DiagnosticStatus.WARN
            arr.status.append(make_status(
                nlvl2, 'move_base状态', txt, '导航',
                [('status_code', ns), ('text', nt)]))

        # 8. 收编的外部诊断 (如 stm32_base_driver)，超过 5s 未更新视为 STALE
        with self.lock:
            ext_items = list(self.external_status.items())
        for name, (st, t) in ext_items:
            age = (now - t).to_sec()
            if age > 5.0:
                st.level = DiagnosticStatus.STALE
                st.message = '数据超时(%.1fs)' % age
            arr.status.append(st)

        return arr


def main():
    rospy.init_node('eggy_health_aggregator')
    rate_hz = rospy.get_param('~rate', 1.0)
    agg = HealthAggregator()
    rate = rospy.Rate(rate_hz)
    rospy.loginfo('eggy_health_aggregator 已启动, 聚合 /diagnostics @ %.1fHz', rate_hz)
    while not rospy.is_shutdown():
        try:
            agg.pub.publish(agg.build_array())
        except Exception as e:
            rospy.logwarn('build_array 异常: %s', e)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




