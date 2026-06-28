#!/usr/bin/env python3
"""
eggy_usart1_node.py — STM32 USART1 数据接收节点
=================================================
从 /dev/eggy_usart1 (115200) 读取 STM32 串口1 发来的文本数据，解析后发布到 ROS 话题。

STM32 USART1 发送的两类数据:
  1. DHT11 温湿度  (1Hz 固定频率)
     格式: "DHT11,<温度>,<湿度>\r\n"  或  "DHT11,ERR,<错误码>\r\n"
  2. 语音模块转发  (事件触发)
     格式: "VOICE,<功能ID>,<命令ID>\r\n"  (ID 为 2 位大写 HEX)

发布话题:
  /stm32/dht11/temperature  std_msgs/Float32  温度 (°C)
  /stm32/dht11/humidity     std_msgs/Float32  湿度 (%)
  /stm32/voice_command      std_msgs/String   语音命令 (JSON: {"func":"00","cmd":"04"})

参数:
  ~port       串口设备路径 (默认 /dev/eggy_usart1)
  ~baudrate   波特率 (默认 115200)
"""

import threading
import traceback
import json
import time

import rospy
import serial
from std_msgs.msg import Float32, String


class USART1Node:
    """STM32 USART1 串口数据接收与解析节点"""

    def __init__(self):
        rospy.init_node("eggy_usart1_node")

        # ---- 参数 ----
        self.port = rospy.get_param("~port", "/dev/eggy_usart1")
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        self.default_temperature = float(rospy.get_param("~default_temperature", 25.0))
        self.default_humidity = float(rospy.get_param("~default_humidity", 50.0))
        self.dht_publish_rate = float(rospy.get_param("~dht_publish_rate", 1.0))
        self.serial_retry_interval = float(rospy.get_param("~serial_retry_interval", 2.0))

        # ---- 发布者 ----
        self.pub_temp = rospy.Publisher(
            "/stm32/dht11/temperature", Float32, queue_size=5
        )
        self.pub_humi = rospy.Publisher(
            "/stm32/dht11/humidity", Float32, queue_size=5
        )
        self.pub_voice = rospy.Publisher(
            "/stm32/voice_command", String, queue_size=10
        )

        # ---- 串口 ----
        self.ser = None
        self._lock = threading.Lock()
        self._running = True

        self._dht_lock = threading.Lock()
        self._current_temp = self.default_temperature
        self._current_humi = self.default_humidity
        self._dht_source = "default"
        self._last_real_dht_stamp = None
        self._open_serial()

        # ---- 统计 ----
        self._dht_count = 0
        self._voice_count = 0
        self._err_count = 0

        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "[%s] 节点就绪  port=%s baud=%d",
            rospy.get_name(), self.port, self.baudrate,
        )

    # ------------------------------------------------------------------ #
    #  串口管理
    # ------------------------------------------------------------------ #
    def _open_serial(self):
        """打开串口，失败时循环重试"""
        while not rospy.is_shutdown() and self._running:
            try:
                s = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=0.5,
                    write_timeout=1.0,
                )
                s.reset_input_buffer()
                with self._lock:
                    self.ser = s
                rospy.loginfo("[%s] 串口已打开: %s", rospy.get_name(), self.port)
                return
            except serial.SerialException as e:
                rospy.logwarn_throttle(
                    5, "[%s] 打开 %s 失败: %s，2s 后重试...",
                    rospy.get_name(), self.port, e,
                )
                time.sleep(2)

    def _close_serial(self):
        """安全关闭串口"""
        with self._lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    # ------------------------------------------------------------------ #
    #  数据解析
    # ------------------------------------------------------------------ #
    @staticmethod
    def _parse_dht11(fields):
        """
        解析 DHT11 行
        fields: ["DHT11", "28.5", "45.0"] 或 ["DHT11", "ERR", "1"]
        返回 (temperature, humidity) 或 None
        """
        if len(fields) < 3:
            return None
        if fields[1] == "ERR":
            rospy.logwarn_throttle(
                10, "[DHT11] 传感器错误, code=%s", fields[2]
            )
            return None
        try:
            temp = float(fields[1])
            humi = float(fields[2])
            if -40.0 <= temp <= 80.0 and 0.0 <= humi <= 100.0:
                return (temp, humi)
            else:
                rospy.logwarn_throttle(
                    10, "[DHT11] 数据越界: temp=%.1f humi=%.1f", temp, humi
                )
                return None
        except ValueError:
            return None

    @staticmethod
    def _parse_voice(fields):
        """
        解析 VOICE 行
        fields: ["VOICE", "00", "04"]
        返回 JSON 字符串或 None
        """
        if len(fields) < 3:
            return None
        func_id = fields[1].strip()
        cmd_id = fields[2].strip()
        if len(func_id) != 2 or len(cmd_id) != 2:
            return None
        try:
            int(func_id, 16)
            int(cmd_id, 16)
        except ValueError:
            return None
        return json.dumps({"func": func_id, "cmd": cmd_id})

    # ------------------------------------------------------------------ #
    #  读取线程
    # ------------------------------------------------------------------ #
    def _reader_thread(self):
        """
        专用读取线程。
        使用 read + 缓冲区拆行，比 readline 更可靠。
        """
        buf = b""
        while not rospy.is_shutdown() and self._running:
            try:
                # 安全获取串口引用
                with self._lock:
                    s = self.ser
                if s is None or not s.is_open:
                    if time.time() - self._last_open_attempt >= self.serial_retry_interval:
                        self._open_serial(block=False)
                    time.sleep(0.2)
                    continue

                # 每次读取可用字节
                try:
                    waiting = s.in_waiting
                    if waiting > 0:
                        data = s.read(waiting)
                    else:
                        # 无数据时阻塞等待 (timeout=0.5s)
                        data = s.read(1)
                except (TypeError, OSError):
                    # 串口 fd 已失效 (shutdown 竞态 / USB 拔出)
                    if not self._running:
                        break
                    raise serial.SerialException("串口 fd 失效")

                if not data:
                    continue

                buf += data

                # 按行拆分处理
                while b"\n" in buf:
                    line_bytes, buf = buf.split(b"\n", 1)
                    line = line_bytes.decode("ascii", errors="ignore").strip()
                    if not line:
                        continue

                    fields = line.split(",")
                    if not fields:
                        continue

                    msg_type = fields[0]

                    # ---- DHT11 ----
                    if msg_type == "DHT11":
                        result = self._parse_dht11(fields)
                        if result is not None:
                            temp, humi = result
                            with self._dht_lock:
                                self._current_temp = temp
                                self._current_humi = humi
                                self._dht_source = "real"
                                self._last_real_dht_stamp = rospy.Time.now()
                            self._dht_count += 1

                    # ---- VOICE ----
                    elif msg_type == "VOICE":
                        voice_json = self._parse_voice(fields)
                        if voice_json is not None:
                            self.pub_voice.publish(String(data=voice_json))
                            self._voice_count += 1
                            rospy.loginfo("[VOICE] %s", voice_json)

                    else:
                        rospy.logdebug("[USART1] 未知行: %s", line)

            except serial.SerialException as e:
                if not self._running:
                    break
                rospy.logwarn("[%s] 串口断开: %s，尝试重连...", rospy.get_name(), e)
                self._close_serial()
                self._open_serial()
            except Exception as e:
                if not self._running:
                    break
                self._err_count += 1
                rospy.logwarn_throttle(
                    5, "[%s] 读取异常: %s\n%s",
                    rospy.get_name(), e, traceback.format_exc(),
                )

    # ------------------------------------------------------------------ #
    #  统计日志 (每 60s)
    # ------------------------------------------------------------------ #
    # ------------------------------------------------------------------ #
    #  ?????????? Qt ???
    # ------------------------------------------------------------------ #
    def _dht_publish_loop(self):
        rate_hz = max(0.1, self.dht_publish_rate)
        rospy.loginfo(
            "[%s] dht cache publisher started temp=%.1f humi=%.1f rate=%.2fHz",
            rospy.get_name(), self._current_temp, self._current_humi, rate_hz,
        )
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown() and self._running:
            try:
                with self._dht_lock:
                    temp = self._current_temp
                    humi = self._current_humi
                self.pub_temp.publish(Float32(data=temp))
                self.pub_humi.publish(Float32(data=humi))
                rate.sleep()
            except Exception as e:
                rospy.logwarn_throttle(5, "[%s] dht cache publish error: %s", rospy.get_name(), e)
                rospy.sleep(1.0)

    def _stats_loop(self):
        while not rospy.is_shutdown() and self._running:
            rospy.sleep(60.0)
            rospy.loginfo(
                "[%s] 统计  DHT11=%d  VOICE=%d  ERR=%d",
                rospy.get_name(),
                self._dht_count, self._voice_count, self._err_count,
            )

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def _shutdown(self):
        self._running = False
        self._close_serial()
        rospy.loginfo("[%s] 已关闭", rospy.get_name())

    def run(self):
        t_reader = threading.Thread(target=self._reader_thread, daemon=True)
        t_reader.start()

        t_dht = threading.Thread(target=self._dht_publish_loop, daemon=True)
        t_dht.start()

        t_stats = threading.Thread(target=self._stats_loop, daemon=True)
        t_stats.start()

        rospy.spin()


if __name__ == "__main__":
    try:
        node = USART1Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
