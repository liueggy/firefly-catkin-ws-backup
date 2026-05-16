#!/usr/bin/env python3
import argparse, subprocess, time, os, signal, sys

parser = argparse.ArgumentParser()
parser.add_argument('--angle', type=float, required=True)
parser.add_argument('--max-wz', type=float, default=0.95)
parser.add_argument('--min-wz', type=float, default=0.18)
parser.add_argument('--slow-zone', type=float, default=35.0)
parser.add_argument('--timeout', type=float, default=12.0)
parser.add_argument('--result', default='/tmp/rotate_result.txt')
args = parser.parse_args()

setup = 'source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; '
rotate = (
        'python3 /root/catkin_ws/src/eggy_bringup/scripts/rotate_by_external_imu.py '
    f'_angle_deg:={args.angle} _max_wz:={args.max_wz} _min_wz:={args.min_wz} _slow_zone_deg:={args.slow_zone}'
)
cmd = ['/bin/bash', '-lc', setup + rotate]
with open(args.result, 'w') as f:
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        rc = p.wait(timeout=args.timeout)
    except subprocess.TimeoutExpired:
        f.write('\nWRAPPER_TIMEOUT killing rotate process\n')
        f.flush()
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception:
            pass
        time.sleep(0.5)
        if p.poll() is None:
            try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception: pass
        rc = 124

# The rotate script itself sends multiple zero Twist messages in finally.
# Avoid an extra rospy publisher here: rospy shutdown/registration can block on flaky ROS master state
# and used to hide otherwise successful motion results.
with open(args.result, 'a') as f:
    f.write('\nWRAPPER_STOP handled_by_rotate_script\n')

print(open(args.result).read(), end='')
sys.exit(rc if rc in (0,124) else rc)
