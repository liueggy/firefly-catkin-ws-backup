#!/usr/bin/env python3
"""Read the Eggy CH340 GPS directly with PyUSB and publish ROS1 messages."""

import json
import math
import threading
import time

import rospy
import usb.core
import usb.util
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from eggy_bringup.gps_nmea import enu_velocity, parse_sentence


class EggyGpsPyUsbNode:
    def __init__(self):
        rospy.init_node("eggy_gps_pyusb")
        self.vendor_id = self._int_param("~vendor_id", "0x1a86")
        self.product_id = self._int_param("~product_id", "0x7523")
        self.baud = int(rospy.get_param("~baud", 9600))
        self.frame_id = rospy.get_param("~frame_id", "gps_link")
        self.reconnect_sec = max(0.5, float(rospy.get_param("~reconnect_sec", 2.0)))
        self.read_timeout_ms = max(50, int(rospy.get_param("~read_timeout_ms", 300)))
        self.detach_kernel_driver = bool(rospy.get_param("~detach_kernel_driver", True))

        self.fix_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=5)
        self.vel_pub = rospy.Publisher("/gps/vel", TwistStamped, queue_size=5)
        self.nmea_pub = rospy.Publisher("/gps/nmea", String, queue_size=20)
        self.status_pub = rospy.Publisher("/gps/status", String, queue_size=5, latch=True)

        self.device = None
        self.detached_driver = False
        self.buffer = b""
        self.lock = threading.Lock()
        self.state = {
            "connected": False,
            "nmea_online": False,
            "fix": False,
            "fix_quality": 0,
            "satellites": 0,
            "satellites_visible": 0,
            "hdop": None,
            "latitude": None,
            "longitude": None,
            "altitude": None,
            "last_error": "starting",
            "last_nmea_monotonic": None,
            "last_fix_monotonic": None,
        }
        rospy.Timer(rospy.Duration(1.0), self._publish_status)
        rospy.on_shutdown(self.close)

    @staticmethod
    def _int_param(name, default):
        value = rospy.get_param(name, default)
        return int(str(value), 0)

    def _control_out(self, request, value=0, index=0):
        return self.device.ctrl_transfer(
            0x40, request, value, index, None, timeout=1000)

    def _set_baud(self):
        factor = 1532620800 // self.baud
        divisor = 3
        while factor > 0xFFF0 and divisor:
            factor >>= 3
            divisor -= 1
        if factor > 0xFFF0:
            raise ValueError("unsupported CH340 baud rate: %d" % self.baud)
        factor = 0x10000 - factor
        register = (factor & 0xFF00) | divisor | 0x80
        self._control_out(0x9A, 0x1312, register)
        self._control_out(0x9A, 0x2518, 0xC3)  # 8 data bits, no parity, 1 stop bit

    def connect(self):
        self.close()
        device = usb.core.find(idVendor=self.vendor_id, idProduct=self.product_id)
        if device is None:
            raise RuntimeError("GPS USB device %04x:%04x not found" % (
                self.vendor_id, self.product_id))
        self.device = device
        if self.detach_kernel_driver and device.is_kernel_driver_active(0):
            device.detach_kernel_driver(0)
            self.detached_driver = True
        device.set_configuration()
        usb.util.claim_interface(device, 0)
        version = bytes(device.ctrl_transfer(0xC0, 0x5F, 0, 0, 2, timeout=1000))
        self._control_out(0xA1, 0, 0)
        self._set_baud()
        self._control_out(0xA4, 0xFF, 0)
        self.buffer = b""
        with self.lock:
            self.state.update({
                "connected": True,
                "last_error": "",
                "usb_version": version.hex(),
            })
        rospy.loginfo("GPS connected via PyUSB %04x:%04x baud=%d version=%s",
                      self.vendor_id, self.product_id, self.baud, version.hex())

    def close(self):
        device = self.device
        self.device = None
        if device is not None:
            try:
                usb.util.release_interface(device, 0)
            except Exception:
                pass
            if self.detached_driver:
                try:
                    device.attach_kernel_driver(0)
                except Exception:
                    pass
            usb.util.dispose_resources(device)
        self.detached_driver = False
        with self.lock:
            self.state["connected"] = False

    def _publish_fix(self, parsed):
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.status.service = NavSatStatus.SERVICE_GPS
        valid = bool(parsed["fix"] and parsed["latitude"] is not None and
                     parsed["longitude"] is not None)
        msg.status.status = NavSatStatus.STATUS_FIX if valid else NavSatStatus.STATUS_NO_FIX
        msg.latitude = parsed["latitude"] if parsed["latitude"] is not None else math.nan
        msg.longitude = parsed["longitude"] if parsed["longitude"] is not None else math.nan
        msg.altitude = parsed["altitude"] if parsed["altitude"] is not None else math.nan
        if parsed["hdop"] is not None:
            variance = max(1.0, parsed["hdop"] * parsed["hdop"])
            msg.position_covariance[0] = variance
            msg.position_covariance[4] = variance
            msg.position_covariance[8] = variance * 4.0
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(msg)

    def _publish_velocity(self, parsed):
        if not parsed["fix"]:
            return
        east, north = enu_velocity(parsed["speed_mps"], parsed["course_deg"])
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = east
        msg.twist.linear.y = north
        self.vel_pub.publish(msg)

    def _handle_line(self, line):
        text = line.decode("ascii", errors="ignore").strip()
        if not text:
            return
        parsed = parse_sentence(text)
        if parsed is None:
            return
        now = time.monotonic()
        self.nmea_pub.publish(String(data=text))
        with self.lock:
            self.state["nmea_online"] = True
            self.state["last_nmea_monotonic"] = now
            if parsed["type"] == "GGA":
                for key in ("fix", "fix_quality", "satellites", "hdop",
                            "latitude", "longitude", "altitude"):
                    self.state[key] = parsed[key]
                if parsed["fix"]:
                    self.state["last_fix_monotonic"] = now
            elif parsed["type"] == "GSV":
                self.state["satellites_visible"] = parsed["satellites_visible"]
        if parsed["type"] == "GGA":
            self._publish_fix(parsed)
        elif parsed["type"] == "RMC":
            self._publish_velocity(parsed)

    def _publish_status(self, _event):
        now = time.monotonic()
        with self.lock:
            state = dict(self.state)
        last_nmea = state.pop("last_nmea_monotonic", None)
        last_fix = state.pop("last_fix_monotonic", None)
        state["nmea_age_sec"] = round(now - last_nmea, 2) if last_nmea else None
        state["fix_age_sec"] = round(now - last_fix, 2) if last_fix else None
        state["nmea_online"] = bool(last_nmea and now - last_nmea < 3.0)
        state["fix"] = bool(state["fix"] and last_fix and now - last_fix < 3.0)
        state.update({
            "stamp": rospy.Time.now().to_sec(),
            "frame_id": self.frame_id,
            "baud": self.baud,
            "vendor_id": "0x%04x" % self.vendor_id,
            "product_id": "0x%04x" % self.product_id,
        })
        self.status_pub.publish(String(data=json.dumps(state, ensure_ascii=False,
                                                       separators=(",", ":"))))

    def run(self):
        while not rospy.is_shutdown():
            if self.device is None:
                try:
                    self.connect()
                except Exception as exc:
                    with self.lock:
                        self.state.update({"connected": False, "last_error": str(exc)})
                    rospy.logwarn_throttle(10, "GPS connect failed: %s", exc)
                    rospy.sleep(self.reconnect_sec)
                    continue
            try:
                data = bytes(self.device.read(0x82, 64, timeout=self.read_timeout_ms))
                self.buffer += data
                while b"\n" in self.buffer:
                    line, self.buffer = self.buffer.split(b"\n", 1)
                    self._handle_line(line)
            except usb.core.USBTimeoutError:
                continue
            except Exception as exc:
                rospy.logwarn("GPS USB read failed: %s; reconnecting", exc)
                with self.lock:
                    self.state["last_error"] = str(exc)
                self.close()
                rospy.sleep(self.reconnect_sec)


if __name__ == "__main__":
    try:
        EggyGpsPyUsbNode().run()
    except rospy.ROSInterruptException:
        pass
