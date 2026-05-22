#!/usr/bin/env python3
import math, struct, threading, time
import serial
import rospy
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Temperature
from std_msgs.msg import Float32

HEAD1=0x7E; HEAD2=0x23
F_RAW=0x04; F_QUAT=0x16; F_EULER=0x26; F_BARO=0x32

class YbImuNode:
    def __init__(self):
        self.port=rospy.get_param('~port','/dev/ttyS4')
        self.baud=rospy.get_param('~baud',115200)
        self.frame_id=rospy.get_param('~frame_id','external_imu_link')
        self.pub_rate=rospy.get_param('~publish_rate',50.0)
        self.ser=serial.Serial(self.port,self.baud,timeout=0.02)
        self.lock=threading.Lock()
        self.ax=self.ay=self.az=0.0
        self.gx=self.gy=self.gz=0.0
        self.mx=self.my=self.mz=0.0
        self.q0=1.0; self.q1=self.q2=self.q3=0.0
        self.roll=self.pitch=self.yaw=0.0
        self.height=self.temperature=self.pressure=self.pressure_contrast=0.0
        self.have_raw=False; self.have_quat=False; self.have_euler=False; self.have_baro=False
        self.pub_imu=rospy.Publisher('imu/data', Imu, queue_size=10)
        self.pub_raw=rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        self.pub_mag=rospy.Publisher('imu/mag', MagneticField, queue_size=10)
        self.pub_pressure=rospy.Publisher('imu/pressure', FluidPressure, queue_size=10)
        self.pub_temp=rospy.Publisher('imu/temperature', Temperature, queue_size=10)
        self.pub_height=rospy.Publisher('imu/height', Float32, queue_size=10)
        self.pub_euler=rospy.Publisher('imu/euler', Float32, queue_size=10)  # yaw deg, debug only
        self.reader=threading.Thread(target=self.read_loop,daemon=True)
        self.reader.start()
        rospy.loginfo('external YB IMU opened %s baud=%s frame_id=%s', self.port, self.baud, self.frame_id)

    def parse_frame(self, f, data):
        with self.lock:
            if f==F_RAW and len(data)>=18:
                ar=16.0/32767.0*9.80665
                gr=(2000.0/32767.0)*math.pi/180.0
                mr=800.0/32767.0*1e-6  # uT -> Tesla
                vals=struct.unpack('<hhhhhhhhh', data[:18])
                self.ax,self.ay,self.az=[v*ar for v in vals[:3]]
                self.gx,self.gy,self.gz=[v*gr for v in vals[3:6]]
                self.mx,self.my,self.mz=[v*mr for v in vals[6:9]]
                self.have_raw=True
            elif f==F_QUAT and len(data)>=16:
                self.q0,self.q1,self.q2,self.q3=struct.unpack('<ffff', data[:16])
                self.have_quat=True
            elif f==F_EULER and len(data)>=12:
                self.roll,self.pitch,self.yaw=struct.unpack('<fff', data[:12])
                self.have_euler=True
            elif f==F_BARO and len(data)>=16:
                self.height,self.temperature,self.pressure,self.pressure_contrast=struct.unpack('<ffff', data[:16])
                self.have_baro=True

    def read_loop(self):
        buf=bytearray()
        while not rospy.is_shutdown():
            try:
                b=self.ser.read(128)
                if not b: continue
                buf.extend(b)
                while len(buf)>=4:
                    i=buf.find(b'\x7e\x23')
                    if i<0:
                        del buf[:]
                        break
                    if i: del buf[:i]
                    if len(buf)<3: break
                    ln=buf[2]
                    if ln<5 or ln>64:
                        del buf[0]
                        continue
                    if len(buf)<ln: break
                    frame=bytes(buf[:ln]); del buf[:ln]
                    if (sum(frame[:-1]) & 0xff) != frame[-1]:
                        continue
                    self.parse_frame(frame[3], frame[4:-1])
            except Exception as e:
                rospy.logwarn_throttle(5.0, 'external imu read error: %s', e)
                time.sleep(0.1)

    def publish_loop(self):
        rate=rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            now=rospy.Time.now()
            with self.lock:
                ax,ay,az,gx,gy,gz=self.ax,self.ay,self.az,self.gx,self.gy,self.gz
                mx,my,mz=self.mx,self.my,self.mz
                q0,q1,q2,q3=self.q0,self.q1,self.q2,self.q3
                h,t,p=self.height,self.temperature,self.pressure
                roll,pitch,yaw=self.roll,self.pitch,self.yaw
                have_raw,have_quat,have_euler,have_baro=self.have_raw,self.have_quat,self.have_euler,self.have_baro
            if have_raw:
                raw=Imu(); raw.header.stamp=now; raw.header.frame_id=self.frame_id
                raw.orientation_covariance[0]=-1.0
                raw.angular_velocity.x=gx; raw.angular_velocity.y=gy; raw.angular_velocity.z=gz
                raw.linear_acceleration.x=ax; raw.linear_acceleration.y=ay; raw.linear_acceleration.z=az
                self.pub_raw.publish(raw)
                mag=MagneticField(); mag.header=raw.header
                mag.magnetic_field.x=mx; mag.magnetic_field.y=my; mag.magnetic_field.z=mz
                self.pub_mag.publish(mag)
                if have_quat:
                    imu=Imu(); imu.header=raw.header
                    imu.orientation.w=q0; imu.orientation.x=q1; imu.orientation.y=q2; imu.orientation.z=q3
                    imu.angular_velocity=raw.angular_velocity
                    imu.linear_acceleration=raw.linear_acceleration
                    self.pub_imu.publish(imu)
                if have_euler:
                    self.pub_euler.publish(Float32(yaw))
            if have_baro:
                fp=FluidPressure(); fp.header.stamp=now; fp.header.frame_id=self.frame_id; fp.fluid_pressure=p
                tm=Temperature(); tm.header=fp.header; tm.temperature=t
                self.pub_pressure.publish(fp); self.pub_temp.publish(tm); self.pub_height.publish(Float32(h))
            rate.sleep()

if __name__=='__main__':
    rospy.init_node('external_yb_imu_node')
    node=YbImuNode()
    node.publish_loop()
