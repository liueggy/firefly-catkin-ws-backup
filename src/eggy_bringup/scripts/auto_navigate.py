#!/usr/bin/env python3
"""
自动导航：加载地图 → 启动导航栈 → 发送目标点 → 自动规划路线运动
使用 move_base action 接口
"""
import rospy
import subprocess
import time
import math
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class AutoNavigator:
    def __init__(self):
        rospy.init_node('auto_navigator', anonymous=True)
        
        # 启动导航栈（map_server + amcl + move_base）
        print("[1/3] Starting navigation stack...")
        self.nav_proc = subprocess.Popen(
            ['roslaunch', 'eggy_bringup', 'navigation.launch'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(5)
        
        # 连接 move_base action server
        print("[2/3] Connecting to move_base...")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        connected = self.client.wait_for_server(timeout=rospy.Duration(10))
        if not connected:
            print("ERROR: Cannot connect to move_base!")
            self.shutdown()
            return
        print("[3/3] Navigation ready!")
        
    def send_goal(self, x, y, yaw=0.0):
        """发送导航目标点"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        print(f"  Goal: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.0f}°")
        self.client.send_goal(goal)
        
        # 等待结果，超时 30s
        finished = self.client.wait_for_result(timeout=rospy.Duration(30))
        state = self.client.get_state()
        
        if finished and state == GoalStatus.SUCCEEDED:
            print(f"  ✅ Reached ({x:.2f}, {y:.2f})")
            return True
        else:
            state_text = {
                0: "PENDING", 1: "ACTIVE", 2: "PREEMPTED",
                3: "SUCCEEDED", 4: "ABORTED", 5: "REJECTED",
                6: "PREEMPTING", 7: "RECALLING", 8: "RECALLED", 9: "LOST"
            }.get(state, f"UNKNOWN({state})")
            print(f"  ⚠️ Failed: {state_text}")
            return False
    
    def run_patrol(self, waypoints):
        """按顺序巡航多个目标点"""
        print(f"\n=== Auto Navigation: {len(waypoints)} waypoints ===")
        success = 0
        for i, (x, y, yaw) in enumerate(waypoints):
            print(f"\n[Waypoint {i+1}/{len(waypoints)}]")
            if self.send_goal(x, y, yaw):
                success += 1
            time.sleep(1)
        
        print(f"\n=== Done: {success}/{len(waypoints)} reached ===")
        return success
    
    def shutdown(self):
        """关闭导航栈"""
        print("\nShutting down navigation...")
        self.client.cancel_all_goals()
        self.nav_proc.terminate()
        self.nav_proc.wait(timeout=5)
        print("Navigation stopped.")


if __name__ == '__main__':
    nav = AutoNavigator()
    
    # 在小场地中设置几个近距离目标点（相对于地图原点）
    # 地图原点在 (-5, -5)，小车起始大约在地图中心 (0, 0)
    # 设置一个小范围的巡航路线
    waypoints = [
        (0.5, 0.0, 0.0),       # 前方 0.5m
        (0.5, 0.5, 1.57),      # 右前方
        (0.0, 0.5, 3.14),      # 右侧，面朝回来
        (0.0, 0.0, 0.0),       # 回到原点
    ]
    
    try:
        nav.run_patrol(waypoints)
    except rospy.ROSInterruptException:
        pass
    finally:
        nav.shutdown()
