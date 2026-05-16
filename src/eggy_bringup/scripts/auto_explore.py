#!/usr/bin/env python3
import rospy, math, random, sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

rospy.init_node('auto_explorer')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.sleep(1.0)

LINEAR = 0.15
ANGULAR = 0.5
OBS_DIST = 0.40
SLOW_DIST = 0.60

def get_min_range(ranges, angle_min, angle_inc, deg_start, deg_end):
    vals = []
    for d in range(int(deg_start), int(deg_end)):
        idx = int((math.radians(d) - angle_min) / angle_inc)
        if 0 <= idx < len(ranges):
            r = ranges[idx]
            if 0.05 < r < 50.0:
                vals.append(r)
    return min(vals) if vals else 99.0

rospy.loginfo('Waiting for first /scan...')
msg = rospy.wait_for_message('/scan', LaserScan, timeout=10)
rospy.loginfo(f'Got scan: {len(msg.ranges)} rays. Starting exploration.')

rate = rospy.Rate(7)
state = 'forward'
turn_end = rospy.Time.now()

while not rospy.is_shutdown():
    try:
        msg = rospy.wait_for_message('/scan', LaserScan, timeout=2)
    except:
        rospy.logwarn('No scan, waiting...')
        continue

    a_min = msg.angle_min
    a_inc = msg.angle_increment
    r = msg.ranges

    front = get_min_range(r, a_min, a_inc, -20, 20)
    fl = get_min_range(r, a_min, a_inc, 20, 60)
    fr = get_min_range(r, a_min, a_inc, -60, -20)
    left = get_min_range(r, a_min, a_inc, 60, 100)
    right = get_min_range(r, a_min, a_inc, -100, -60)

    twist = Twist()

    if state == 'forward':
        if front < OBS_DIST:
            state = 'turn_left' if fl > fr else 'turn_right'
            turn_end = rospy.Time.now() + rospy.Duration(random.uniform(1.0, 3.0))
            rospy.loginfo(f'Obstacle {front:.2f}m -> {state}')
        elif front < SLOW_DIST:
            twist.linear.x = LINEAR * 0.5
            twist.angular.z = 0.15 if fl > fr else -0.15
        else:
            twist.linear.x = LINEAR
            twist.angular.z = random.uniform(-0.08, 0.08)
            if left < 0.30:
                twist.angular.z -= 0.2
            if right < 0.30:
                twist.angular.z += 0.2
    else:
        if rospy.Time.now() > turn_end and front > SLOW_DIST:
            state = 'forward'
            rospy.loginfo(f'Clear {front:.2f}m -> forward')
        else:
            twist.angular.z = ANGULAR if state == 'turn_left' else -ANGULAR
            if rospy.Time.now() > turn_end and front < OBS_DIST:
                turn_end += rospy.Duration(0.5)

    pub.publish(twist)
    rate.sleep()
