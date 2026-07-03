#!/usr/bin/env python3
"""Publish compact Wi-Fi and 4G status as std_msgs/String JSON."""

import json
import os
import re
import subprocess

import rospy
from std_msgs.msg import String


WIFI_IFACE = "wlan0"
MODEM_IFACE = "enx020c29a39b6d"


def run(argv, timeout=2.0):
    try:
        result = subprocess.run(
            argv, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, timeout=timeout, check=False
        )
        return result.stdout.strip() if result.returncode == 0 else ""
    except (OSError, subprocess.SubprocessError):
        return ""


def interface_connected(name):
    base = os.path.join("/sys/class/net", name)
    if not os.path.isdir(base):
        return False
    try:
        with open(os.path.join(base, "operstate"), encoding="ascii") as stream:
            if stream.read().strip() not in ("up", "unknown"):
                return False
        carrier_path = os.path.join(base, "carrier")
        if os.path.exists(carrier_path):
            with open(carrier_path, encoding="ascii") as stream:
                if stream.read().strip() != "1":
                    return False
    except OSError:
        return False
    return bool(run(["ip", "-o", "-4", "addr", "show", "dev", name, "scope", "global"]))


def wifi_name():
    name = run(["iwgetid", WIFI_IFACE, "--raw"])
    if name:
        return name
    output = run(["iw", "dev", WIFI_IFACE, "link"])
    match = re.search(r"^\s*SSID:\s*(.+?)\s*$", output, re.MULTILINE)
    if match:
        return match.group(1)
    output = run(["nmcli", "-t", "-f", "GENERAL.CONNECTION", "device", "show", WIFI_IFACE])
    return output.partition(":")[2].strip().replace(r"\:", ":") if ":" in output else ""


def modem_operator():
    output = run(["mmcli", "-m", "any", "--output-keyvalue"], timeout=3.0)
    for key in ("modem.3gpp.operator-name", "modem.generic.operator-name"):
        match = re.search(r"^%s\s*:\s*(.*?)\s*$" % re.escape(key), output, re.MULTILINE)
        if match and match.group(1) not in ("--", "unknown"):
            return normalize_operator(match.group(1))
    output = run(
        ["journalctl", "-u", "quectel-cm.service", "-n", "200", "--no-pager"],
        timeout=3.0,
    )
    for pattern in (
        r"operator(?:-name)?\s*[:=]\s*[\"']?([^\"'\r\n,]+)",
        r"\b(CHINA MOBILE|CMCC|CHN-UNICOM|CHINA UNICOM|CHN-CT|CHINA TELECOM)\b",
    ):
        matches = re.findall(pattern, output, re.IGNORECASE)
        if matches:
            return normalize_operator(matches[-1].strip())
    numeric = re.findall(r'\+COPS:\s*\d+,\s*2,\s*"(\d{5,6})"', output)
    if numeric:
        return operator_from_plmn(numeric[-1])
    return ""


def normalize_operator(name):
    upper = name.upper()
    if "MOBILE" in upper or "CMCC" in upper:
        return "中国移动"
    if "UNICOM" in upper:
        return "中国联通"
    if "TELECOM" in upper or "CHN-CT" in upper:
        return "中国电信"
    return name


def operator_from_plmn(plmn):
    if plmn in ("46000", "46002", "46004", "46007", "46008"):
        return "中国移动"
    if plmn in ("46001", "46006", "46009"):
        return "中国联通"
    if plmn in ("46003", "46005", "46011"):
        return "中国电信"
    return plmn


def build_status():
    wifi_connected = interface_connected(WIFI_IFACE)
    modem_connected = interface_connected(MODEM_IFACE)
    return {
        "wifi": {
            "connected": wifi_connected,
            "name": wifi_name() if wifi_connected else "",
        },
        "4g": {
            "connected": modem_connected,
            "operator": modem_operator() if modem_connected else "",
        },
    }


def main():
    rospy.init_node("eggy_network_status")
    publisher = rospy.Publisher("/eggy/network/status", String, queue_size=1, latch=True)
    rate = rospy.Rate(max(0.1, float(rospy.get_param("~rate", 0.5))))
    while not rospy.is_shutdown():
        publisher.publish(String(data=json.dumps(build_status(), ensure_ascii=False)))
        rate.sleep()


if __name__ == "__main__":
    main()
