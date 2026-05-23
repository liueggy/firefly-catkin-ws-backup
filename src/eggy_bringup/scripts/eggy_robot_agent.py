#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Eggy robot cloud relay agent with lightweight map preview.

Design:
- Use rospy locally for ROS topics, so raw /map never goes through rosbridge JSON.
- Build a small preview map: crop known area + margin, adaptive downsample, occupied-priority pooling.
- Send preview in ordinary occupancy_grid.data so the existing web frontend can draw it.
"""
import asyncio
import base64
import json
import math
import os
import sys
import threading
import time

# systemd does not source ROS setup.bash; make ROS Python packages visible.
for p in (
    "/opt/ros/noetic/lib/python3/dist-packages",
    "/root/catkin_ws/devel/lib/python3/dist-packages",
):
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Float32, String
    import tf
except Exception as e:
    raise SystemExit("Missing ROS Python environment: %r" % (e,))

try:
    import websockets
except ImportError:
    raise SystemExit("Missing websockets. Install with: apt-get install -y python3-websockets")

CLOUD_URL = os.environ.get("EGGY_CLOUD_URL", "ws://167.71.221.110:8088/robot/eggy-001")
ROBOT_ID = os.environ.get("EGGY_ROBOT_ID", "eggy-001")
SEND_HZ = float(os.environ.get("EGGY_AGENT_HZ", "3"))
BAT_FULL = float(os.environ.get("EGGY_BAT_FULL", "12.0"))
BAT_EMPTY = float(os.environ.get("EGGY_BAT_EMPTY", "10.5"))
EFFECTIVE_LIDAR_MIN = float(os.environ.get("EGGY_EFFECTIVE_LIDAR_MIN", "0.15"))
MAP_MAX_DIM = int(os.environ.get("EGGY_MAP_MAX_DIM", "576"))
MAP_MARGIN = int(os.environ.get("EGGY_MAP_MARGIN", "0"))
MAP_MIN_PERIOD = float(os.environ.get("EGGY_MAP_MIN_PERIOD", "1.0"))
STREAM_MODE = os.environ.get("EGGY_STREAM_MODE", "map").lower()  # lite | map

state_lock = threading.RLock()
state = {
    "odom": None,
    "scan": None,
    "battery": None,
    "map_preview": None,
    "last": {},
    "count": {},
    "map_version": 0,
    "map_sig": None,
    "last_map_process": 0.0,
    "stream_mode": STREAM_MODE,
}
recent_logs = []
cmd_pub = None
map_pub = None
goal_pub = None
simple_goal_pub = None
simple_cancel_pub = None
auto_exploring = False
simple_nav_status = "idle"
tf_listener = None


def now():
    return time.time()


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def add_log(level, msg, source="agent"):
    item = {"time": now(), "level": level, "source": source, "message": str(msg)[:180]}
    with state_lock:
        recent_logs.insert(0, item)
        del recent_logs[20:]
    print(time.strftime("%F %T"), level, source, msg, flush=True)


def mark(topic):
    with state_lock:
        state["last"][topic] = now()
        state["count"][topic] = state["count"].get(topic, 0) + 1


def battery_percent(v):
    if v is None:
        return None
    try:
        v = float(v)
    except Exception:
        return None
    if v >= BAT_FULL:
        return 100
    return int(round(clamp((v - BAT_EMPTY) / (BAT_FULL - BAT_EMPTY) * 100.0, 0, 100)))


def yaw_from_q(q):
    if not q:
        return 0.0
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def yaw_from_tuple(q):
    if not q:
        return 0.0
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def lookup_map_base_pose():
    # Use map->base_link for web map overlay. /odom and /map diverge during SLAM.
    global tf_listener
    if tf_listener is None:
        return None
    for base in ("base_link", "base_footprint"):
        try:
            tf_listener.waitForTransform("map", base, rospy.Time(0), rospy.Duration(0.05))
            trans, rot = tf_listener.lookupTransform("map", base, rospy.Time(0))
            return float(trans[0]), float(trans[1]), yaw_from_tuple(rot)
        except Exception:
            continue
    return None


def odom_cb(msg):
    mark("/odom")
    with state_lock:
        state["odom"] = msg


def scan_cb(msg):
    mark("/scan")
    with state_lock:
        state["scan"] = msg


def battery_cb(msg):
    mark("/battery/voltage")
    with state_lock:
        state["battery"] = float(msg.data)


def imu_cb(msg):
    mark("/imu/data_raw")


def simple_nav_status_cb(msg):
    global simple_nav_status
    simple_nav_status = getattr(msg, "data", "") or "idle"
    mark("/simple_nav/status")


def compute_map_signature(values, width, height):
    n = len(values)
    if n <= 0:
        return None
    step = max(1, n // 2048)
    s = (width * 1000003 + height) & 0xFFFFFFFF
    for i in range(0, n, step):
        s = (s * 131 + int(values[i]) + 2) & 0xFFFFFFFF
    return [n, s]


def build_map_preview(msg):
    width = int(msg.info.width)
    height = int(msg.info.height)
    data = msg.data
    total = width * height
    if width <= 0 or height <= 0 or total <= 0:
        return None

    min_x, min_y = width, height
    max_x, max_y = -1, -1
    known = free = occ = 0

    # One pass: stats + known bounding box.
    for idx, v in enumerate(data):
        if v != -1:
            known += 1
            y = idx // width
            x = idx - y * width
            if x < min_x: min_x = x
            if x > max_x: max_x = x
            if y < min_y: min_y = y
            if y > max_y: max_y = y
            if v >= 50:
                occ += 1
            elif v == 0:
                free += 1

    origin_x = float(msg.info.origin.position.x)
    origin_y = float(msg.info.origin.position.y)
    res = float(msg.info.resolution)

    if known == 0:
        return {
            "frame_id": msg.header.frame_id or "map",
            "resolution": res,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "width": 0,
            "height": 0,
            "data": [],
            "stats": {"known": 0, "free": 0, "occupied": 0, "total": total, "known_percent": 0},
            "version": state["map_version"],
            "age_sec": 0,
            "stale": False,
            "preview": {"crop": False, "downsample": 1, "max_dim": MAP_MAX_DIM},
        }

    x0 = max(0, min_x - MAP_MARGIN)
    x1 = min(width - 1, max_x + MAP_MARGIN)
    y0 = max(0, min_y - MAP_MARGIN)
    y1 = min(height - 1, max_y + MAP_MARGIN)
    crop_w = x1 - x0 + 1
    crop_h = y1 - y0 + 1
    factor = max(1, int(math.ceil(max(crop_w, crop_h) / float(max(1, MAP_MAX_DIM)))))
    out_w = int(math.ceil(crop_w / float(factor)))
    out_h = int(math.ceil(crop_h / float(factor)))
    out = []

    # Occupied-priority pooling: preserve walls/obstacles in preview.
    for gy in range(out_h):
        yy0 = y0 + gy * factor
        yy1 = min(y1 + 1, yy0 + factor)
        for gx in range(out_w):
            xx0 = x0 + gx * factor
            xx1 = min(x1 + 1, xx0 + factor)
            has_known = False
            has_free = False
            has_occ = False
            for yy in range(yy0, yy1):
                base = yy * width
                for xx in range(xx0, xx1):
                    v = data[base + xx]
                    if v == -1:
                        continue
                    has_known = True
                    if v >= 50:
                        has_occ = True
                        break
                    if v == 0:
                        has_free = True
                if has_occ:
                    break
            if has_occ:
                out.append(100)
            elif has_free:
                out.append(0)
            elif has_known:
                out.append(50)
            else:
                out.append(-1)

    sig = compute_map_signature(out, out_w, out_h)
    with state_lock:
        if sig != state.get("map_sig"):
            state["map_sig"] = sig
            state["map_version"] += 1
        version = state["map_version"]

    return {
        "frame_id": msg.header.frame_id or "map",
        "resolution": res * factor,
        "origin_x": origin_x + x0 * res,
        "origin_y": origin_y + y0 * res,
        "width": out_w,
        "height": out_h,
        "data": out,
        "stats": {
            "known": known,
            "free": free,
            "occupied": occ,
            "total": total,
            "known_percent": round(known * 100.0 / total, 1) if total else 0,
        },
        "version": version,
        "age_sec": 0,
        "stale": False,
        "preview": {
            "crop": True,
            "source_width": width,
            "source_height": height,
            "crop_x": x0,
            "crop_y": y0,
            "crop_width": crop_w,
            "crop_height": crop_h,
            "downsample": factor,
            "max_dim": MAP_MAX_DIM,
            "encoding": "int8-preview",
        },
    }


def map_cb(msg):
    t = now()
    with state_lock:
        if t - state.get("last_map_process", 0) < MAP_MIN_PERIOD:
            return
        state["last_map_process"] = t
    mark("/map")
    try:
        preview = build_map_preview(msg)
        with state_lock:
            state["map_preview"] = preview
        if preview:
            add_log("INFO", "map preview %sx%s ds=%s known=%s%%" % (
                preview.get("width"), preview.get("height"),
                preview.get("preview", {}).get("downsample"),
                preview.get("stats", {}).get("known_percent"),
            ))
    except Exception as e:
        add_log("ERROR", "map preview failed: %r" % (e,))


def ros_publish_cmd(msg):
    lx = clamp(float(msg.get("linear_x", 0)), -0.35, 0.35)
    ly = clamp(float(msg.get("linear_y", 0)), -0.35, 0.35)
    az = clamp(float(msg.get("angular_z", 0)), -0.9, 0.9)
    tw = Twist()
    tw.linear.x = lx
    tw.linear.y = ly
    tw.angular.z = az
    if cmd_pub is not None:
        cmd_pub.publish(tw)
    add_log("INFO", "cmd_vel x=%.2f y=%.2f z=%.2f" % (lx, ly, az), "cloud")


def lidar_points(scan, odom, max_points=90, pose=None):
    if not scan:
        return [], {"front": None, "nearest": None, "nearest_angle": None, "range_min": None, "effective_min": EFFECTIVE_LIDAR_MIN, "range_max": None, "front_sector_deg": 24, "note": "front=前方±12°；nearest=360°全局最近障碍"}
    ranges = scan.ranges
    amin = scan.angle_min
    inc = scan.angle_increment or 0
    sensor_min = scan.range_min
    rmin = max(sensor_min, EFFECTIVE_LIDAR_MIN)
    rmax = min(scan.range_max, 6.0)
    rx = ry = yaw = 0.0
    if pose:
        rx, ry, yaw = pose
    elif odom:
        pos = odom.pose.pose.position
        rx, ry = pos.x, pos.y
        yaw = yaw_from_q(odom.pose.pose.orientation)
    step = max(1, len(ranges) // max_points) if ranges else 1
    pts = []
    nearest = 99.0
    nearest_angle = 0
    front_vals = []
    for i in range(0, len(ranges), step):
        r = ranges[i]
        if not isinstance(r, (int, float)) or not math.isfinite(r) or r < rmin:
            continue
        rr = min(r, rmax)
        a = amin + i * inc
        deg = math.degrees(a)
        if abs(deg) <= 12:
            front_vals.append(rr)
        if rr < nearest:
            nearest = rr
            nearest_angle = int(deg)
        wa = yaw + a
        pts.append({"x": round(rx + rr * math.cos(wa), 2), "y": round(ry + rr * math.sin(wa), 2), "hit": r <= rmax})
    return pts, {
        "front": round(min(front_vals), 2) if front_vals else None,
        "nearest": round(nearest, 2) if nearest < 90 else None,
        "nearest_angle": nearest_angle if nearest < 90 else None,
        "range_min": sensor_min,
        "effective_min": EFFECTIVE_LIDAR_MIN,
        "range_max": scan.range_max,
        "front_sector_deg": 24,
        "note": "front=小车正前方±12°内最近有效障碍；nearest=雷达360°范围内最近有效障碍",
    }


def topic_status():
    meta = {
        "/odom": ("nav_msgs/Odometry", 8, "底盘里程计/姿态"),
        "/scan": ("sensor_msgs/LaserScan", 5, "RPLIDAR 激光雷达"),
        "/map": ("nav_msgs/OccupancyGrid", 1, "gmapping 实时地图预览"),
        "/battery/voltage": ("std_msgs/Float32", 1, "电池电压"),
    }
    out = []
    with state_lock:
        last_map = dict(state["last"])
        count_map = dict(state["count"])
    for topic, (typ, hz, desc) in meta.items():
        last = last_map.get(topic)
        age = now() - last if last else None
        ok = age is not None and age < (8.0 if topic == "/map" else 3.0)
        out.append({"name": topic, "type": typ, "description": desc, "status": "ok" if ok else "error", "interval": round(age, 2) if age is not None else None, "msg_count": count_map.get(topic, 0), "expected_hz": hz})
    return out


def empty_occ():
    return {"frame_id": "map", "resolution": 0.05, "origin_x": -10, "origin_y": -10, "width": 0, "height": 0, "data": [], "stats": {"known": 0, "free": 0, "occupied": 0, "total": 0, "known_percent": 0}, "version": state.get("map_version", 0), "age_sec": None, "stale": True}


def build_alerts(sysinfo, batt, summary, occ):
    alerts = []
    def add(level, name, msg):
        alerts.append({"level": level, "name": name, "message": msg, "time": now()})
    if not sysinfo["base"]:
        add("red", "里程计断流", "/odom 超过 2 秒没有新数据")
    if not sysinfo["lidar"]:
        add("red", "雷达断流", "/scan 超过 2 秒没有新数据")
    if occ.get("stale"):
        add("yellow", "地图未更新", "/map 预览超过 8 秒没有新数据")
    if batt.get("percent") is not None and batt["percent"] < 20:
        add("red", "低电量", "电池约 %s%% / %sV" % (batt["percent"], batt.get("voltage")))
    if summary.get("front") is not None and summary["front"] < 0.55:
        add("yellow", "前方近障碍", "前方 %sm" % summary["front"])
    if not alerts:
        add("green", "系统正常", "关键数据在线")
    return alerts[:4]



def list_navigation_maps():
    import os, glob, json
    out=[]
    nav_dir='/root/catkin_ws/maps/navigation'
    os.makedirs(nav_dir, exist_ok=True)
    for y in sorted([f for f in glob.glob(nav_dir+"/*.yaml") if f and os.path.exists(f) and not os.path.islink(f) and os.path.basename(f)!="latest.yaml"], key=os.path.getmtime, reverse=True)[:20]:
        try:
            st=os.stat(y)
            name=os.path.splitext(os.path.basename(y))[0]
            out.append({'id':name,'name':name,'path':y,'created_at':st.st_mtime,'size':st.st_size})
        except Exception:
            pass
    return out


def save_navigation_map(name=None):
    import os, shutil, time, re
    src_yaml='/root/catkin_ws/maps/manual/latest.yaml'
    src_pgm='/root/catkin_ws/maps/manual/latest.pgm'
    nav_dir='/root/catkin_ws/maps/navigation'
    os.makedirs(nav_dir, exist_ok=True)
    if not os.path.exists(src_yaml) or not os.path.exists(src_pgm):
        raise RuntimeError('manual latest map not found')
    safe=re.sub(r'[^A-Za-z0-9_-]+','_', name or time.strftime('nav_%Y%m%d_%H%M%S')).strip('_')[:64] or time.strftime('nav_%Y%m%d_%H%M%S')
    dst_yaml=os.path.join(nav_dir, safe+'.yaml')
    dst_pgm=os.path.join(nav_dir, safe+'.pgm')
    shutil.copyfile(src_pgm, dst_pgm)
    text=open(src_yaml).read().splitlines()
    fixed=[]
    for line in text:
        if line.strip().startswith('image:'):
            fixed.append('image: '+safe+'.pgm')
        else:
            fixed.append(line)
    open(dst_yaml,'w').write('\n'.join(fixed)+'\n')
    latest_yaml=os.path.join(nav_dir,'latest.yaml')
    latest_pgm=os.path.join(nav_dir,'latest.pgm')
    for link,target in [(latest_yaml,dst_yaml),(latest_pgm,dst_pgm)]:
        try:
            if os.path.islink(link) or os.path.exists(link): os.unlink(link)
            os.symlink(os.path.basename(target), link)
        except Exception:
            pass
    return dst_yaml

def nav_view():
    with state_lock:
        odom = state.get("odom")
        scan = state.get("scan")
        bv = state.get("battery")
        mode = state.get("stream_mode", "lite")
        occ = dict(state.get("map_preview") or empty_occ())
        last = dict(state.get("last", {}))
        logs = list(recent_logs[:8])

    if occ.get("age_sec") is not None and last.get("/map"):
        occ["age_sec"] = round(now() - last.get("/map"), 2)
        occ["stale"] = occ["age_sec"] > 8.0

    robot = {"x": 0, "y": 0, "yaw": 0, "vx": 0, "wz": 0, "pose_age_sec": None}
    map_pose = None if mode == "map" else lookup_map_base_pose()
    if odom:
        pos = odom.pose.pose.position
        tw = odom.twist.twist
        px, py, pyaw = (map_pose if map_pose else (pos.x, pos.y, yaw_from_q(odom.pose.pose.orientation)))
        robot.update({
            "x": round(px, 3),
            "y": round(py, 3),
            "yaw": round(pyaw, 3),
            "vx": round(tw.linear.x, 3),
            "wz": round(tw.angular.z, 3),
            "pose_age_sec": round(now() - last.get("/odom", 0), 2),
        })

    pts, summary = lidar_points(scan, odom, max_points=90 if mode == "map" else 36, pose=map_pose)
    batt = {"voltage": round(float(bv), 2) if bv is not None else None, "percent": battery_percent(bv), "rule": ">=%sV=100%%, %s-%sV linear" % (BAT_FULL, BAT_EMPTY, BAT_FULL)}
    sysinfo = {
        "ros": True,
        "base": now() - last.get("/odom", 0) < 2,
        "imu": now() - last.get("/imu/data_raw", 0) < 2,
        "lidar": now() - last.get("/scan", 0) < 2,
        "map": now() - last.get("/map", 0) < 8,
        "navigation": (last.get("/navigation", 0) and now() - last.get("/navigation", 0) < 60),
        "simple_nav_status": simple_nav_status,
        "diagnostics": False,
        "robot_id": ROBOT_ID,
        "stream_mode": mode,
        "map_source": "navigation" if state.get("navigation_active") else "mapping",
        "map_preview": True,
        "auto_explore": auto_exploring,
    }

    if mode != "map":
        occ = {k: v for k, v in occ.items() if k != "data"}
        occ.update({"data": [], "width": 0, "height": 0})

    width_m = (occ.get("width") or 0) * (occ.get("resolution") or 0.05) or 20
    height_m = (occ.get("height") or 0) * (occ.get("resolution") or 0.05) or 20
    return {
        "type": "nav_view",
        "ts": now(),
        "robot": robot,
        "map": {"frame_id": "map", "resolution": occ.get("resolution", 0.05), "origin_x": occ.get("origin_x", -10), "origin_y": occ.get("origin_y", -10), "width_m": width_m, "height_m": height_m},
        "occupancy_grid": occ,
        "raw_map_base64": occ.get("data") and base64.b64encode(bytes([min(254,max(0,(v+128)&255)) for v in occ["data"]])).decode("ascii") or "",
        "lidar_points": pts,
        "global_plan": [],
        "local_plan": [],
        "goal": {"x": 0, "y": 0, "yaw": 0},
        "nav_status": "ACTIVE" if sysinfo["base"] else "NO_ODOM",
        "battery": batt,
        "summary": summary,
        "system": sysinfo,
        "topics": topic_status(),
        "logs": logs,
        "alerts": build_alerts(sysinfo, batt, summary, occ),
        "saved_maps": [],
        "navigation_maps": list_navigation_maps(),
        "scenes": [],
    }


async def cloud_loop():
    while not rospy.is_shutdown():
        try:
            add_log("INFO", "connecting cloud %s" % CLOUD_URL)
            async with websockets.connect(CLOUD_URL, ping_interval=15, ping_timeout=8, max_size=4*1024*1024) as cws:
                async def cloud_reader():
                    async for raw in cws:
                        try:
                            d = json.loads(raw)
                        except Exception:
                            continue
                        typ = d.get("type")
                        if typ == "cmd_vel":
                            ros_publish_cmd(d)
                        elif typ == "set_mode":
                            mode = str(d.get("mode", "lite")).lower()
                            if mode in ("lite", "map"):
                                with state_lock:
                                    state["stream_mode"] = mode
                                add_log("INFO", "switch stream mode -> %s" % mode, "cloud")
                        elif typ == "load_map":
                            grid_raw = d.get("grid", [])
                            w = int(d.get("width", 0))
                            h = int(d.get("height", 0))
                            if w > 0 and h > 0 and len(grid_raw) == w * h:
                                msg = OccupancyGrid()
                                msg.header.frame_id = "map"
                                msg.header.stamp = rospy.Time.now()
                                msg.info.resolution = float(d.get("resolution", 0.05))
                                msg.info.width = w
                                msg.info.height = h
                                msg.info.origin.position.x = float(d.get("origin_x", 0.0))
                                msg.info.origin.position.y = float(d.get("origin_y", 0.0))
                                msg.info.origin.orientation.w = 1.0
                                msg.data = [max(-128, min(127, int(v))) for v in grid_raw]
                                map_pub.publish(msg)
                                add_log("INFO", "load_map: published %dx%d id=%s" % (w, h, d.get("id","")))
                            else:
                                add_log("WARN", "load_map: bad grid %dx%d len=%d" % (w, h, len(grid_raw)))
                        elif typ == "start_mapping":
                            import subprocess
                            subprocess.Popen(["run", "mapping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            add_log("INFO", "mapping: started", "cloud")
                        elif typ == "stop_mapping":
                            import subprocess
                            subprocess.Popen(["stop", "mapping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            add_log("INFO", "mapping: stopped", "cloud")
                        elif typ == "start_mapping":
                            import subprocess
                            subprocess.Popen(["run", "mapping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            add_log("INFO", "mapping: started", "cloud")
                        elif typ == "stop_mapping":
                            import subprocess
                            subprocess.Popen(["stop", "mapping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            add_log("INFO", "mapping: stopped", "cloud")
                        elif typ == "restart_agent":
                            import subprocess
                            add_log("INFO", "agent: restart requested", "cloud")
                            subprocess.Popen(["systemctl", "restart", "eggy-robot-agent"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        elif typ == "save_navigation_map":
                            try:
                                path = save_navigation_map(d.get("name"))
                                add_log("INFO", "navigation map saved: %s" % path, "cloud")
                            except Exception as e:
                                add_log("ERROR", "save navigation map failed: %r" % e, "cloud")
                        elif typ == "delete_navigation_map":
                            import subprocess
                            name = str(d.get("name") or "").strip()
                            if name:
                                subprocess.Popen(["bash","-lc", "rm -f /root/catkin_ws/maps/navigation/"+name+".yaml /root/catkin_ws/maps/navigation/"+name+".pgm"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                                add_log("INFO", "navigation map deleted: %s" % name, "cloud")
                            else:
                                add_log("WARN", "delete_navigation_map: no name", "cloud")
                        elif typ == "set_navigation_map":
                            # Selection is passed by start_navigation(map_path); kept for future UI state.
                            add_log("INFO", "navigation map selected: %s" % d.get("map_path"), "cloud")
                        elif typ == "start_navigation":
                            import subprocess
                            map_path = str(d.get("map_path") or "/root/catkin_ws/maps/manual/latest.yaml")
                            subprocess.Popen(["/usr/local/bin/eggy-run-navigation", map_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            mark("/navigation")
                            state["navigation_active"] = True
                            add_log("INFO", "navigation: started map=%s" % map_path, "cloud")
                        elif typ == "stop_navigation":
                            import subprocess
                            subprocess.Popen(["/usr/local/bin/eggy-stop-navigation"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            mark("/navigation_stop")
                            state["navigation_active"] = False
                            add_log("INFO", "navigation: stopped", "cloud")
                        elif typ == "cancel_goal":
                            import subprocess
                            subprocess.Popen(["rostopic", "pub", "-1", "/move_base/cancel", "actionlib_msgs/GoalID", "{}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            add_log("INFO", "navigation: goal cancelled", "cloud")
                        elif typ == "start_simple_nav":
                            import subprocess
                            subprocess.Popen(["rosrun", "eggy_bringup", "eggy_simple_navigator.py"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            add_log("INFO", "simple_nav: started", "cloud")
                        elif typ == "stop_simple_nav":
                            import subprocess
                            if simple_cancel_pub is not None:
                                simple_cancel_pub.publish(String(data="cancel"))
                            subprocess.Popen(["rosnode", "kill", "/eggy_simple_navigator"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            ros_publish_cmd({"linear_x": 0, "linear_y": 0, "angular_z": 0})
                            add_log("INFO", "simple_nav: stopped", "cloud")
                        elif typ == "simple_goal":
                            try:
                                msg = PoseStamped()
                                msg.header.frame_id = str(d.get("frame_id") or "map")
                                msg.header.stamp = rospy.Time.now()
                                msg.pose.position.x = float(d.get("x", 0.0))
                                msg.pose.position.y = float(d.get("y", 0.0))
                                yaw = float(d.get("yaw", 0.0))
                                msg.pose.orientation.z = math.sin(yaw/2.0)
                                msg.pose.orientation.w = math.cos(yaw/2.0)
                                if simple_goal_pub is not None:
                                    simple_goal_pub.publish(msg)
                                add_log("INFO", "simple goal: %.2f %.2f" % (msg.pose.position.x, msg.pose.position.y), "cloud")
                            except Exception as e:
                                add_log("ERROR", "simple goal failed: %r" % e, "cloud")
                        elif typ == "cancel_simple_goal":
                            if simple_cancel_pub is not None:
                                simple_cancel_pub.publish(String(data="cancel"))
                            ros_publish_cmd({"linear_x": 0, "linear_y": 0, "angular_z": 0})
                            add_log("INFO", "simple goal: cancelled", "cloud")
                        elif typ == "goal":
                            try:
                                msg = PoseStamped()
                                msg.header.frame_id = str(d.get("frame_id") or "map")
                                msg.header.stamp = rospy.Time.now()
                                msg.pose.position.x = float(d.get("x", 0.0))
                                msg.pose.position.y = float(d.get("y", 0.0))
                                yaw = float(d.get("yaw", 0.0))
                                msg.pose.orientation.z = math.sin(yaw/2.0)
                                msg.pose.orientation.w = math.cos(yaw/2.0)
                                if goal_pub is not None:
                                    goal_pub.publish(msg)
                                add_log("INFO", "navigation goal: %.2f %.2f" % (msg.pose.position.x, msg.pose.position.y), "cloud")
                            except Exception as e:
                                add_log("ERROR", "goal failed: %r" % e, "cloud")
                        elif typ == "auto_explore":
                            import subprocess
                            if d.get("enabled"):
                                subprocess.Popen(["run", "auto"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                                add_log("INFO", "auto_explore: started", "cloud")
                                auto_exploring = True
                            else:
                                subprocess.Popen(["stop", "auto"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                                add_log("INFO", "auto_explore: stopped", "cloud")
                                auto_exploring = False
                        elif typ == "reset":
                            import subprocess
                            subprocess.Popen(["stop", "auto"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            ros_publish_cmd({"linear_x": 0, "linear_y": 0, "angular_z": 0})
                            auto_exploring = False
                            empty = OccupancyGrid()
                            empty.header.frame_id = "map"
                            empty.header.stamp = rospy.Time.now()
                            empty.info.resolution = 0.05
                            # no publish to /map here; keep gmapping/map_server authoritative
                            add_log("INFO", "reset: cleared", "cloud")
                        elif typ in ("stop", "emergency_stop"):
                            ros_publish_cmd({"linear_x": 0, "linear_y": 0, "angular_z": 0})
                        elif typ == "ping":
                            pass
                reader = asyncio.create_task(cloud_reader())
                while not rospy.is_shutdown():
                    payload = json.dumps(nav_view(), ensure_ascii=False, separators=(",", ":"))
                    await cws.send(payload)
                    await asyncio.sleep(1.0 / max(0.5, SEND_HZ))
                reader.cancel()
        except Exception as e:
            import traceback, sys; traceback.print_exc(); sys.stderr.flush(); add_log("ERROR", "reconnect after error: %r errno=%s filepath=%s" % (e, getattr(e,"errno","?"), getattr(e,"filename","?")))
            await asyncio.sleep(3)


def main():
    global cmd_pub, map_pub, tf_listener
    rospy.init_node("eggy_robot_agent", anonymous=False, disable_signals=True)
    tf_listener = tf.TransformListener()
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
    map_pub = rospy.Publisher("/eggy/map_override", OccupancyGrid, queue_size=1, latch=True)
    global goal_pub
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    global simple_goal_pub, simple_cancel_pub
    simple_goal_pub = rospy.Publisher("/simple_goal", PoseStamped, queue_size=1)
    simple_cancel_pub = rospy.Publisher("/simple_nav/cancel", String, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=5)
    rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=3)
    rospy.Subscriber("/map", OccupancyGrid, map_cb, queue_size=1)
    rospy.Subscriber("/battery/voltage", Float32, battery_cb, queue_size=3)
    rospy.Subscriber("/imu/data_raw", Imu, imu_cb, queue_size=5)
    rospy.Subscriber("/simple_nav/status", String, simple_nav_status_cb, queue_size=10)
    add_log("INFO", "Eggy rospy preview agent cloud=%s max_dim=%s map_period=%ss" % (CLOUD_URL, MAP_MAX_DIM, MAP_MIN_PERIOD))
    # sync auto_exploring flag with existing tmux session
    import subprocess as _sp
    global auto_exploring
    _r = _sp.run(["tmux", "has-session", "-t", "eggy_auto_map"], stdout=_sp.DEVNULL, stderr=_sp.DEVNULL)
    auto_exploring = _r.returncode == 0
    if auto_exploring:
        add_log("INFO", "detected running auto exploration", "agent")
    asyncio.run(cloud_loop())


if __name__ == "__main__":
    main()
