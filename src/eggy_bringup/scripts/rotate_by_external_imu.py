#!/usr/bin/env python3
import math, os, time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class ImuRotator:
    def __init__(self):
        self.gz = None
        self.last_stamp = None
        self.pub = rospy.Publisher(
            rospy.get_param('~cmd_vel_topic', '/cmd_vel/manual'), Twist, queue_size=10)
        self.sub = rospy.Subscriber('/external_imu/imu/data_raw', Imu, self.cb, queue_size=200)

    def cb(self, msg):
        self.gz = msg.angular_velocity.z
        self.last_stamp = time.time()

    def wait_imu(self, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end and not rospy.is_shutdown():
            if self.gz is not None and self.last_stamp and time.time() - self.last_stamp < 0.5:
                return True
            rospy.sleep(0.02)
        return False

    def stop(self, n=12):
        z = Twist()
        # Do not use rospy.Rate here: during shutdown it can block/raise and hide results.
        for _ in range(n):
            try:
                self.pub.publish(z)
            except Exception:
                pass
            time.sleep(0.02)

    def estimate_bias(self, sec=1.2):
        vals=[]; end=time.time()+sec
        while time.time()<end and not rospy.is_shutdown():
            if self.gz is not None:
                vals.append(self.gz)
            rospy.sleep(0.01)
        if not vals:
            raise RuntimeError('no imu gyro samples for bias')
        return sum(vals)/len(vals)

    def rotate(self, target_deg, max_wz=0.85, min_wz=0.16, slow_zone_deg=28.0, settle_sec=0.35):
        if not self.wait_imu():
            raise RuntimeError('no /external_imu/imu/data_raw')
        self.stop(8)
        bias = self.estimate_bias()
        sign = 1.0 if target_deg >= 0 else -1.0
        target = abs(math.radians(target_deg))
        angle = 0.0
        last_t = time.time()
        last_g = self.gz - bias
        r = rospy.Rate(80)
        started = time.time()
        max_seen = 0.0
        samples = 0
        # timeout: enough for 180 deg, prevents runaway
        timeout = max(4.0, target / max(0.2, max_wz) * 4.0 + 2.0)
        while not rospy.is_shutdown():
            now = time.time()
            dt = now - last_t
            g = (self.gz if self.gz is not None else bias) - bias
            # integrate signed rotation in target direction
            angle += sign * 0.5 * (last_g + g) * dt
            last_t = now; last_g = g
            samples += 1
            max_seen = max(max_seen, abs(g))
            remain = target - angle
            if remain <= math.radians(1.2):
                break
            if now - started > timeout:
                print('WARN timeout before target, remain_deg=%.2f' % math.degrees(remain), flush=True)
                break
            # proportional slowdown near target, faster far away
            if remain > math.radians(slow_zone_deg):
                wz = max_wz
            else:
                ratio = max(0.0, min(1.0, remain / math.radians(slow_zone_deg)))
                wz = min_wz + (max_wz - min_wz) * ratio
            tw = Twist(); tw.angular.z = sign * wz
            self.pub.publish(tw)
            r.sleep()
        self.stop(18)
        # capture residual drift after stop
        drift0 = angle; t0=time.time(); last_t=time.time(); last_g=(self.gz if self.gz is not None else bias)-bias
        while time.time()-t0 < settle_sec and not rospy.is_shutdown():
            now=time.time(); dt=now-last_t; g=(self.gz if self.gz is not None else bias)-bias
            angle += sign * 0.5*(last_g+g)*dt
            last_t=now; last_g=g
            rospy.sleep(0.01)
        return {
            'target_deg': target_deg,
            'integrated_deg_before_settle': math.degrees(drift0)*sign,
            'integrated_deg_after_settle': math.degrees(angle)*sign,
            'error_deg': target_deg - math.degrees(angle)*sign,
            'bias_rads': bias,
            'max_abs_gz_rads': max_seen,
            'duration_s': time.time()-started,
            'samples': samples,
        }

if __name__ == '__main__':
    rospy.init_node('rotate_by_external_imu', disable_signals=True)
    target = float(rospy.get_param('~angle_deg', 90.0))
    max_wz = float(rospy.get_param('~max_wz', 0.85))
    min_wz = float(rospy.get_param('~min_wz', 0.16))
    slow_zone = float(rospy.get_param('~slow_zone_deg', 30.0))
    rot = ImuRotator()
    exit_code = 0
    try:
        res = rot.rotate(target, max_wz=max_wz, min_wz=min_wz, slow_zone_deg=slow_zone)
        print('ROTATE_RESULT ' + ' '.join('%s=%.6f'%(k,v) if isinstance(v,float) else '%s=%s'%(k,v) for k,v in res.items()), flush=True)
    except Exception as e:
        exit_code = 2
        print('ROTATE_ERROR %s' % e, flush=True)
    finally:
        try:
            rot.stop(20)
        finally:
            rospy.signal_shutdown('rotate complete')
            # rospy can leave non-daemon XMLRPC/socket threads alive; force process exit
            # after result and stop commands have been flushed.
            os._exit(exit_code)
