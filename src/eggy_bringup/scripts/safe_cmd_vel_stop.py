#!/usr/bin/env python3
import os, time
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('safe_cmd_vel_stop', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# give publisher a short moment; don't wait forever for subscribers
end = time.time() + 0.8
msg = Twist()
while time.time() < end and not rospy.is_shutdown():
    pub.publish(msg)
    time.sleep(0.05)
print('SAFE_STOP_SENT', flush=True)
os._exit(0)
