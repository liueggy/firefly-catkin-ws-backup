#!/usr/bin/env python3
import json

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PolygonStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, String

class RosQt5GuiAdapter:
    def __init__(self):
        self.voltage = None
        self.single_goal_inspection_enabled = bool(
            rospy.get_param('~single_goal_inspection_enabled', True))
        self.single_goal_return_home = bool(
            rospy.get_param('~single_goal_return_home', False))
        self.single_goal_expected_class = str(
            rospy.get_param('~single_goal_expected_class', 'any')).strip() or 'any'
        self.battery_pub = rospy.Publisher('/battery', BatteryState, queue_size=1, latch=True)
        # 简化目标点链路 2026-06-01: 直接发 move_base 监听的 /nav_goal，省掉 legacy relay 中转
        self.goal_pub = rospy.Publisher('/nav_goal', PoseStamped, queue_size=1)
        self.inspection_request_pub = rospy.Publisher(
            '/inspection_servo_route/request', String, queue_size=5)
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
        if self.single_goal_inspection_enabled:
            request = self.build_single_goal_inspection_request(msg)
            self.inspection_request_pub.publish(String(json.dumps(request, ensure_ascii=False)))
            rospy.loginfo(
                'ros_qt5_gui_adapter promoted /goal_pose to single-point inspection: %s (%.3f, %.3f)',
                request['route'][0]['id'],
                request['route'][0]['x'],
                request['route'][0]['y'])
            return
        # Qt /goal_pose -> 直接转发到 move_base 监听的 /nav_goal (单跳)
        self.goal_pub.publish(msg)

    def build_single_goal_inspection_request(self, msg):
        yaw = tf.transformations.euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])[2]
        point_id = 'single_goal_{:.2f}_{:.2f}'.format(
            float(msg.pose.position.x), float(msg.pose.position.y))
        return {
            'command': 'start',
            'loop': False,
            'return_home': self.single_goal_return_home,
            'source': 'goal_pose_single_inspection',
            'route': [{
                'id': point_id,
                'frame_id': msg.header.frame_id or 'map',
                'x': round(float(msg.pose.position.x), 3),
                'y': round(float(msg.pose.position.y), 3),
                'yaw': round(float(yaw), 4),
                'expected_class': self.normalize_expected_class(
                    self.single_goal_expected_class),
                'allow_vision_intercept': False,
            }],
        }

    @staticmethod
    def normalize_expected_class(value):
        text = str(value or 'any').strip().lower()
        aliases = {
            'water': 'water_meter',
            'meter': 'water_meter',
            'water_meter': 'water_meter',
            'pressure': 'pressure_gauge',
            'gauge': 'pressure_gauge',
            'pressure_gauge': 'pressure_gauge',
            'any': 'any',
            'auto': 'any',
        }
        return aliases.get(text, 'any')

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
