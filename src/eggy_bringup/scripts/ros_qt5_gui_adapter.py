#!/usr/bin/env python3
import json

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PolygonStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, String

from mission_protocol import build_goal_pose_mission

class RosQt5GuiAdapter:
    def __init__(self):
        self.voltage = None
        self.battery_pub = rospy.Publisher('/battery', BatteryState, queue_size=1, latch=True)
        self.mission_request_pub = rospy.Publisher(
            '/eggy/mission/request', String, queue_size=5)
        self.plan_pub = rospy.Publisher('/plan', Path, queue_size=1, latch=True)
        self.local_plan_pub = rospy.Publisher('/local_plan', Path, queue_size=1, latch=True)
        self.global_costmap_pub = rospy.Publisher('/global_costmap/costmap', OccupancyGrid, queue_size=1, latch=True)
        self.local_costmap_pub = rospy.Publisher('/local_costmap/costmap', OccupancyGrid, queue_size=1, latch=True)
        self.footprint_pub = rospy.Publisher('/local_costmap/published_footprint', PolygonStamped, queue_size=1, latch=True)

        rospy.Subscriber('/goal_pose', PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber('/battery/voltage', Float32, self.on_voltage, queue_size=1)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.plan_pub.publish, queue_size=1)
        rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, self.plan_pub.publish, queue_size=1)
        rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.local_plan_pub.publish, queue_size=1)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.global_costmap_pub.publish, queue_size=1)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.local_costmap_pub.publish, queue_size=1)
        rospy.Subscriber('/move_base/local_costmap/footprint', PolygonStamped, self.footprint_pub.publish, queue_size=1)

    def on_goal(self, msg):
        yaw = tf.transformations.euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])[2]
        request = build_goal_pose_mission(
            msg.header.frame_id or 'map', msg.pose.position.x,
            msg.pose.position.y, yaw)
        self.mission_request_pub.publish(
            String(json.dumps(request, ensure_ascii=False)))
        rospy.loginfo(
            'ros_qt5_gui_adapter converted /goal_pose to plain navigation mission: %s',
            request['request_id'])

    def on_voltage(self, msg):
        bs = BatteryState()
        bs.header.stamp = rospy.Time.now()
        bs.voltage = float(msg.data)
        # Unknown capacity/current on this base. Keep percentage NaN per BatteryState convention.
        bs.percentage = min(float(msg.data) / 12.0, 1.0)
        bs.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        bs.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        bs.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        bs.present = True
        self.battery_pub.publish(bs)

if __name__ == '__main__':
    rospy.init_node('ros_qt5_gui_adapter')
    RosQt5GuiAdapter()
    rospy.loginfo('ros_qt5_gui_adapter started: /goal_pose bridge enabled')
    rospy.spin()
