#!/usr/bin/env python3
"""
Map-oriented auto explorer for Eggy Firefly car.
Low-speed right-wall following + obstacle avoidance for gmapping.
Only uses linear.x and angular.z. No lateral mecanum motion.
"""
import math
import signal
import sys
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MapperExplorer:
    def __init__(self):
        rospy.init_node('eggy_mapper_explorer', anonymous=False)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan = None
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)

        # Conservative params for wooden floor + gmapping
        self.v_slow = 0.075
        self.v_nom = 0.115
        self.w_slow = 0.18
        self.w_nom = 0.32
        self.w_turn = 0.38

        self.front_stop = 0.42
        self.front_slow = 0.70
        self.side_min = 0.30
        self.wall_target = 0.58
        self.wall_far = 0.95

        self.state = 'wait_scan'
        self.turn_until = rospy.Time.now()
        self.last_log = rospy.Time.now()
        self.start_time = rospy.Time.now()
        self.max_runtime = rospy.Duration(8 * 60)  # safety cap 8 min

    def scan_cb(self, msg):
        self.scan = msg

    def sector_min(self, start_deg, end_deg):
        if self.scan is None:
            return 99.0
        vals = []
        a0 = self.scan.angle_min
        inc = self.scan.angle_increment
        n = len(self.scan.ranges)
        for deg in range(int(start_deg), int(end_deg) + 1):
            idx = int((math.radians(deg) - a0) / inc)
            if 0 <= idx < n:
                r = self.scan.ranges[idx]
                if math.isfinite(r) and 0.08 < r < 8.0:
                    vals.append(r)
        return min(vals) if vals else 99.0

    def publish_stop(self):
        self.pub.publish(Twist())

    def log_status(self, front, left, right, cmd):
        now = rospy.Time.now()
        if (now - self.last_log).to_sec() > 3.0:
            rospy.loginfo('state=%s front=%.2f left=%.2f right=%.2f cmd vx=%.2f wz=%.2f',
                          self.state, front, left, right, cmd.linear.x, cmd.angular.z)
            self.last_log = now

    def run(self):
        rospy.loginfo('Waiting for /scan...')
        try:
            rospy.wait_for_message('/scan', LaserScan, timeout=10)
        except Exception as e:
            rospy.logerr('No /scan: %s', e)
            return
        rospy.loginfo('Mapper explorer started: low-speed right-wall following.')
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if rospy.Time.now() - self.start_time > self.max_runtime:
                rospy.logwarn('Max runtime reached, stopping.')
                break

            front = self.sector_min(-18, 18)
            front_left = self.sector_min(18, 55)
            front_right = self.sector_min(-55, -18)
            left = self.sector_min(65, 105)
            right = self.sector_min(-105, -65)
            right_front = self.sector_min(-55, -25)

            cmd = Twist()
            now = rospy.Time.now()

            # Emergency / close obstacle: turn away from more blocked side.
            if front < self.front_stop:
                self.state = 'avoid_turn'
                cmd.linear.x = 0.0
                # If right side has more room, turn right; else left.
                # For wall following, usually left turn at front obstacle.
                if front_right > front_left + 0.15 and right > self.side_min:
                    cmd.angular.z = -self.w_turn
                else:
                    cmd.angular.z = self.w_turn

            else:
                self.state = 'follow_right_wall'
                # Forward speed: slow near front obstacles.
                cmd.linear.x = self.v_slow if front < self.front_slow else self.v_nom

                # Right-wall following control.
                # If no wall on right, gently search right.
                if right > self.wall_far and right_front > self.wall_far and front > self.front_slow:
                    cmd.angular.z = -self.w_slow
                    self.state = 'search_right_wall'
                else:
                    # Keep right wall around target distance.
                    # Too close => steer left, too far => steer right.
                    err = right - self.wall_target
                    cmd.angular.z = -0.45 * err
                    cmd.angular.z = max(-self.w_nom, min(self.w_nom, cmd.angular.z))

                    # Corner compensation: if front-right is close, steer left.
                    if right_front < 0.45:
                        cmd.angular.z += 0.18
                    # If left very close, steer right a bit only if front is clear.
                    if left < self.side_min and front > self.front_slow:
                        cmd.angular.z -= 0.10

                # Clamp angular speed for gmapping smoothness.
                cmd.angular.z = max(-self.w_turn, min(self.w_turn, cmd.angular.z))

            self.pub.publish(cmd)
            self.log_status(front, left, right, cmd)
            rate.sleep()

        rospy.loginfo('Stopping mapper explorer.')
        for _ in range(20):
            self.publish_stop()
            rate.sleep()

if __name__ == '__main__':
    explorer = MapperExplorer()
    def handler(signum, frame):
        explorer.publish_stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    explorer.run()
