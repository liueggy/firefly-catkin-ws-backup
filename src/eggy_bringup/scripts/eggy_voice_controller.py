#!/usr/bin/env python3
"""语音指令 → 持续 10Hz Twist (麦克纳姆轮, vy 横向平移)"""
import rospy, json
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VoiceController:
    def __init__(self):
        rospy.init_node("eggy_voice_controller")
        self.fwd_spd = rospy.get_param("~forward_speed", 0.6)
        self.bwd_spd = rospy.get_param("~backward_speed", 0.4)
        self.lat_spd = rospy.get_param("~lateral_speed", 0.4)
        self.rot_spd = rospy.get_param("~rotate_speed", 0.8)
        self.spn_spd = rospy.get_param("~spin_speed", 1.0)
        self.fwd_dur = rospy.get_param("~forward_duration", 1.5)
        self.bwd_dur = rospy.get_param("~backward_duration", 1.5)
        self.lat_dur = rospy.get_param("~lateral_duration", 1.0)
        self.rot_dur = rospy.get_param("~rotate_duration", 1.5)
        self.spn_dur = rospy.get_param("~spin_duration", 1.5)

        self.cmd_pub = rospy.Publisher(rospy.get_param("~cmd_vel_topic", "/cmd_vel/manual"), Twist, queue_size=1)
        self.stat_pub = rospy.Publisher("/eggy/voice/status", String, queue_size=5)
        rospy.Subscriber("/stm32/voice_command", String, self.on_voice, queue_size=10)

        # (vx, vy, vz, dur) 麦克纳姆: vy=横向, vz=旋转
        self.ACTIONS = {
            (0,1): (0,0,0,0), (0,2): (0,0,0,0), (0,3): (0,0,0,0),
            (0,4): (self.fwd_spd, 0, 0, self.fwd_dur),   # 前进
            (0,5): (-self.bwd_spd, 0, 0, self.bwd_dur),  # 后退
            (0,6): (0, self.lat_spd, 0, self.lat_dur),    # 左转→左移
            (0,7): (0, -self.lat_spd, 0, self.lat_dur),   # 右转→右移
            (0,8): (0, 0, self.spn_spd, self.spn_dur),    # 左旋→原地转
            (0,9): (0, 0, -self.spn_spd, self.spn_dur),   # 右旋→原地转
        }
        self.NAMES = {
            (0,1): "停车", (0,2): "停车", (0,3): "休眠",
            (0,4): "前进", (0,5): "后退", (0,6): "左移",
            (0,7): "右移", (0,8): "左旋", (0,9): "右旋",
        }
        self._t = None; self._s = None
        rospy.loginfo("语音就绪 前%.1f×%.1fs 后%.1f×%.1fs 横%.1f×%.1fs 旋%.1f×%.1fs",
            self.fwd_spd,self.fwd_dur,self.bwd_spd,self.bwd_dur,
            self.lat_spd,self.lat_dur,self.spn_spd,self.spn_dur)

    def on_voice(self, msg):
        try:
            d = json.loads(msg.data)
            k = (int(d["func"],16), int(d["cmd"],16))
        except Exception: return
        if k in self.ACTIONS:
            vx, vy, vz, dur = self.ACTIONS[k]
            n = self.NAMES.get(k, "?")
            rospy.loginfo("← %s (vx=%.2f vy=%.2f vz=%.2f)", n, vx, vy, vz)
            self.stat_pub.publish(json.dumps({"action": n}))
            self._go(vx, vy, vz, dur)

    def _go(self, vx, vy, vz, dur):
        self._halt()
        if dur <= 0: return
        t = Twist(); t.linear.x = vx; t.linear.y = vy; t.angular.z = vz
        self._twist = t
        self._t = rospy.Timer(rospy.Duration(0.1), self._tick)
        self._s = rospy.Timer(rospy.Duration(dur), lambda _: self._halt(), oneshot=True)

    def _tick(self, _): self.cmd_pub.publish(self._twist)
    def _halt(self):
        if self._t: self._t.shutdown(); self._t = None
        if self._s: self._s.shutdown(); self._s = None
        self.cmd_pub.publish(Twist())

if __name__ == "__main__":
    try: VoiceController(); rospy.spin()
    except rospy.ROSInterruptException: pass
