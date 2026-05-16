#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Continuous map autosaver for Eggy gmapping.
Saves map every N seconds using map_server/map_saver into /root/catkin_ws/maps/auto.
Also maintains latest.yaml/latest.pgm symlinks and a manifest.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

import rospy
from nav_msgs.msg import OccupancyGrid

MAP_DIR = os.environ.get('EGGY_AUTO_MAP_DIR', '/root/catkin_ws/maps/auto')
INTERVAL = float(os.environ.get('EGGY_AUTO_SAVE_INTERVAL', '30'))
MANIFEST = os.path.join(MAP_DIR, 'manifest.jsonl')

running = True
last_map = None


def on_signal(signum, frame):
    global running
    running = False


def map_cb(msg):
    global last_map
    last_map = msg


def atomic_symlink(target, link):
    tmp = link + '.tmp'
    try:
        if os.path.lexists(tmp):
            os.unlink(tmp)
        os.symlink(os.path.basename(target), tmp)
        os.replace(tmp, link)
    except Exception:
        # Filesystem may not like symlink replacement; ignore.
        pass


def save_once(reason='periodic'):
    os.makedirs(MAP_DIR, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    prefix = os.path.join(MAP_DIR, 'map_' + ts)
    cmd = ['rosrun', 'map_server', 'map_saver', '-f', prefix]
    rospy.loginfo('Saving map: %s', prefix)
    p = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out = ((p.stdout or '') + (p.stderr or '')).strip()
    yaml_file = prefix + '.yaml'
    pgm_file = prefix + '.pgm'
    ok = (p.returncode == 0 and os.path.exists(yaml_file) and os.path.exists(pgm_file))
    if ok:
        # fsync files and directory to improve power-loss survivability after save.
        for path in (yaml_file, pgm_file):
            try:
                fd = os.open(path, os.O_RDONLY)
                os.fsync(fd)
                os.close(fd)
            except Exception:
                pass
        atomic_symlink(yaml_file, os.path.join(MAP_DIR, 'latest.yaml'))
        atomic_symlink(pgm_file, os.path.join(MAP_DIR, 'latest.pgm'))
        rec = {
            'time': ts,
            'reason': reason,
            'yaml': yaml_file,
            'pgm': pgm_file,
            'map_received': last_map is not None,
            'width': getattr(getattr(last_map, 'info', None), 'width', None) if last_map else None,
            'height': getattr(getattr(last_map, 'info', None), 'height', None) if last_map else None,
            'resolution': getattr(getattr(last_map, 'info', None), 'resolution', None) if last_map else None,
        }
        with open(MANIFEST, 'a', encoding='utf-8') as f:
            f.write(json.dumps(rec, ensure_ascii=False) + '\n')
            f.flush()
            os.fsync(f.fileno())
        try:
            dfd = os.open(MAP_DIR, os.O_RDONLY)
            os.fsync(dfd)
            os.close(dfd)
        except Exception:
            pass
        rospy.loginfo('Map saved OK: %s.yaml', prefix)
        return True
    rospy.logwarn('Map save failed rc=%s output=%s', p.returncode, out)
    return False


def main():
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    rospy.init_node('eggy_auto_map_saver', anonymous=False)
    rospy.Subscriber('/map', OccupancyGrid, map_cb, queue_size=1)
    os.makedirs(MAP_DIR, exist_ok=True)
    rospy.loginfo('Waiting for /map before first save...')
    try:
        rospy.wait_for_message('/map', OccupancyGrid, timeout=60)
    except Exception as e:
        rospy.logwarn('No /map yet after timeout: %s; will keep waiting/saving attempts', e)
    next_save = time.time()
    saved_any = False
    while running and not rospy.is_shutdown():
        now = time.time()
        if now >= next_save:
            saved_any = save_once('periodic') or saved_any
            next_save = now + INTERVAL
        time.sleep(0.5)
    rospy.loginfo('Auto map saver stopping, final save...')
    if last_map is not None:
        save_once('final')


if __name__ == '__main__':
    main()
