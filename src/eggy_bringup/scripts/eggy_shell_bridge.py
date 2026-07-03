#!/usr/bin/env python3
"""Bounded, allowlisted ROS shell bridge using std_msgs/String JSON."""

import codecs
import json
import os
import re
import selectors
import shlex
import signal
import subprocess
import threading
import time
import uuid

import rospy
from std_msgs.msg import String


PLAIN_COMMANDS = {
    "pwd", "uname", "hostname", "date", "uptime", "whoami", "id", "ls", "stat",
    "df", "du", "free", "cat", "head", "tail", "grep", "find", "ps", "pgrep",
    "ip", "iw", "iwgetid", "lsusb", "lspci", "dmesg", "journalctl",
}
ROS_SUBCOMMANDS = {
    "rosnode": {"list", "info", "ping", "machine", "cleanup", "kill"},
    "rostopic": {"list", "info", "type", "find", "echo", "hz", "bw"},
    "rosparam": {"list", "get"},
    "rosservice": {"list", "info", "type", "find", "uri"},
    "rospack": {"find", "list", "depends", "depends1", "plugins"},
    "rosmsg": {"show", "list", "package", "packages"},
    "rossrv": {"show", "list", "package", "packages"},
    "rosversion": None,
    "roswtf": None,
}
SYSTEMCTL_ACTIONS = {"status", "show", "is-active", "is-enabled", "list-units", "list-unit-files"}
NMCLI_OBJECTS = {"general", "networking", "radio", "connection", "device"}
BLOCKED_WORDS = {
    "rm", "rmdir", "shred", "wipefs", "mkfs", "fdisk", "sfdisk", "parted",
    "dd", "reboot", "shutdown", "poweroff", "halt", "init", "useradd", "userdel",
    "usermod", "groupadd", "groupdel", "passwd", "chpasswd", "iptables",
    "ip6tables", "nft", "ufw", "firewall-cmd", "mount", "umount", "sudo", "su",
    "chmod", "chown", "chgrp", "killall", "pkill",
}
SHELL_META = {"|", "||", "&&", ";", ">", ">>", "<", "<<", "&"}
ROSLAUNCH_PARAM = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*:=[A-Za-z0-9_./:+-]+$")


def json_message(**fields):
    fields["stamp"] = rospy.Time.now().to_sec()
    return String(data=json.dumps(fields, ensure_ascii=False))


def validate_command(command):
    if not isinstance(command, str) or not command.strip():
        raise ValueError("command must be a non-empty string")
    try:
        argv = shlex.split(command, posix=True)
    except ValueError as exc:
        raise ValueError("invalid command quoting: %s" % exc)
    if not argv:
        raise ValueError("empty command")
    lowered = [part.lower() for part in argv]
    if any(part in SHELL_META for part in argv):
        raise ValueError("shell operators and redirections are not allowed")
    if any(part in BLOCKED_WORDS for part in lowered):
        raise ValueError("destructive or privileged command pattern blocked")
    program = os.path.basename(argv[0]).lower()
    if program == "roslaunch":
        if len(argv) < 3 or argv[1] != "eggy_bringup" or argv[2] != "auto_explore_mapping.launch":
            raise ValueError("only auto_explore_mapping.launch can be started from this terminal")
        for part in argv[3:]:
            if not ROSLAUNCH_PARAM.match(part):
                raise ValueError("auto mapping roslaunch arguments must be name:=value pairs")
        return argv, True
    if program == "rostopic" and len(argv) >= 6 and lowered[1] == "pub":
        once_flag = argv[2] in ("-1", "--once")
        stop_topic = argv[3] == "/auto_explore/stop"
        stop_msg = argv[4] == "std_msgs/Bool" and argv[5].lower() in ("true", "false", "1", "0")
        if once_flag and stop_topic and stop_msg:
            return argv, True
        raise ValueError("only publishing /auto_explore/stop is allowed")
    if program in PLAIN_COMMANDS:
        if program == "find" and any(part in ("-delete", "-exec", "-execdir", "-ok", "-okdir") for part in lowered):
            raise ValueError("mutating find actions are not allowed")
        if program == "ip" and any(part in {
            "add", "delete", "del", "set", "flush", "replace", "change", "append"
        } for part in lowered[1:]):
            raise ValueError("mutating ip actions are not allowed")
        if program == "iw" and any(part in {
            "set", "del", "connect", "disconnect", "interface", "station"
        } for part in lowered[1:]):
            # "station dump/get" is read-only and is handled below.
            station_index = lowered[1:].index("station") + 1 if "station" in lowered[1:] else -1
            if station_index < 0 or not any(
                part in ("dump", "get") for part in lowered[station_index + 1:]
            ):
                raise ValueError("mutating iw actions are not allowed")
        if program == "dmesg" and any(part in ("-c", "--clear") for part in lowered[1:]):
            raise ValueError("clearing the kernel log is not allowed")
        if program == "journalctl" and any(
            part.startswith(("--vacuum", "--rotate", "--flush", "--sync", "--relinquish-var"))
            for part in lowered[1:]
        ):
            raise ValueError("mutating journalctl actions are not allowed")
        return argv, False
    if program == "systemctl":
        action = next((part for part in lowered[1:] if not part.startswith("-")), "")
        if action not in SYSTEMCTL_ACTIONS:
            raise ValueError("only read-only systemctl actions are allowed")
        return argv, False
    if program == "nmcli":
        obj = next((part for part in lowered[1:] if not part.startswith("-")), "")
        if obj not in NMCLI_OBJECTS or any(part in {"up", "down", "delete", "modify", "add", "connect", "disconnect"} for part in lowered):
            raise ValueError("mutating nmcli actions are not allowed")
        return argv, False
    if program in ROS_SUBCOMMANDS:
        allowed = ROS_SUBCOMMANDS[program]
        subcommand = next((part for part in lowered[1:] if not part.startswith("-")), "")
        if allowed is not None and subcommand not in allowed:
            raise ValueError("ROS subcommand is not allowed")
        return argv, True
    raise ValueError("command is not allowlisted")


class ShellBridge:
    def __init__(self):
        self.output_pub = rospy.Publisher("/eggy/shell/output", String, queue_size=100)
        self.status_pub = rospy.Publisher("/eggy/shell/status", String, queue_size=20, latch=True)
        self.request_sub = rospy.Subscriber("/eggy/shell/request", String, self.on_request, queue_size=10)
        self.cancel_sub = rospy.Subscriber("/eggy/shell/cancel", String, self.on_cancel, queue_size=10)
        self.lock = threading.Lock()
        self.process = None
        self.command_id = ""
        self.cancel_reason = ""
        self.default_timeout = float(rospy.get_param("~timeout", 30.0))
        self.max_timeout = float(rospy.get_param("~max_timeout", 300.0))
        self.default_output_limit = int(rospy.get_param("~output_limit", 262144))
        self.max_output_limit = int(rospy.get_param("~max_output_limit", 1048576))
        rospy.on_shutdown(self.shutdown)
        self.publish_status(state="idle", command_id="")

    def publish_status(self, **fields):
        self.status_pub.publish(json_message(**fields))

    def on_request(self, msg):
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                raise ValueError("request JSON must be an object")
            command_id = str(request.get("command_id") or uuid.uuid4().hex)
            argv, needs_ros = validate_command(request.get("command", ""))
            timeout = min(max(float(request.get("timeout", self.default_timeout)), 0.1), self.max_timeout)
            output_limit = min(
                max(int(request.get("output_limit", self.default_output_limit)), 1024),
                self.max_output_limit,
            )
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.publish_status(state="rejected", command_id="", error=str(exc))
            return
        with self.lock:
            if self.process is not None or self.command_id:
                self.publish_status(state="rejected", command_id=command_id, error="another command is running")
                return
            self.command_id = command_id
            self.cancel_reason = ""
            thread = threading.Thread(
                target=self.execute, args=(command_id, argv, needs_ros, timeout, output_limit), daemon=True
            )
            thread.start()

    def on_cancel(self, msg):
        try:
            request = json.loads(msg.data)
            requested_id = str(request.get("command_id", "")) if isinstance(request, dict) else ""
        except (ValueError, TypeError, json.JSONDecodeError):
            requested_id = msg.data.strip()
        with self.lock:
            if not self.command_id or (requested_id and requested_id != self.command_id):
                self.publish_status(state="cancel_ignored", command_id=requested_id, error="command is not running")
                return
            self.cancel_reason = "cancelled"
            process = self.process
        if process is not None:
            self.terminate_group(process)

    @staticmethod
    def terminate_group(process):
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except (OSError, ProcessLookupError):
            return
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except (OSError, ProcessLookupError):
                pass

    def execute(self, command_id, argv, needs_ros, timeout, output_limit):
        if needs_ros:
            command = [
                "/bin/bash", "-lc",
                "source /opt/ros/noetic/setup.bash; "
                "source /root/catkin_ws/devel/setup.bash 2>/dev/null || true; exec \"$@\"",
                "eggy-shell",
            ] + argv
        else:
            command = argv
        started = time.monotonic()
        total = 0
        sequence = 0
        truncated = False
        reason = ""
        process = None
        try:
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0,
                start_new_session=True
            )
            with self.lock:
                self.process = process
            self.publish_status(state="running", command_id=command_id, command=shlex.join(argv))
            selector = selectors.DefaultSelector()
            decoders = {}
            for stream_name, pipe in (("stdout", process.stdout), ("stderr", process.stderr)):
                selector.register(pipe, selectors.EVENT_READ, stream_name)
                decoders[stream_name] = codecs.getincrementaldecoder("utf-8")("replace")
            while selector.get_map():
                if time.monotonic() - started > timeout:
                    reason = "timeout"
                    self.terminate_group(process)
                for key, _ in selector.select(timeout=0.1):
                    chunk = os.read(key.fileobj.fileno(), 4096)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        tail = decoders[key.data].decode(b"", final=True)
                        if tail and total < output_limit:
                            self.output_pub.publish(json_message(
                                command_id=command_id, sequence=sequence, stream=key.data, data=tail
                            ))
                            sequence += 1
                        continue
                    remaining = output_limit - total
                    if remaining <= 0:
                        truncated = True
                        continue
                    accepted = chunk[:remaining]
                    total += len(accepted)
                    truncated = truncated or len(accepted) < len(chunk)
                    text = decoders[key.data].decode(accepted)
                    if text:
                        self.output_pub.publish(json_message(
                            command_id=command_id, sequence=sequence, stream=key.data, data=text
                        ))
                        sequence += 1
                if process.poll() is not None and not selector.get_map():
                    break
                with self.lock:
                    if self.cancel_reason:
                        reason = self.cancel_reason
            exit_code = process.wait()
            state = reason or ("completed" if exit_code == 0 else "failed")
            self.publish_status(
                state=state, command_id=command_id, exit_code=exit_code,
                output_bytes=total, output_truncated=truncated,
                duration=round(time.monotonic() - started, 3),
            )
        except Exception as exc:
            if process is not None and process.poll() is None:
                self.terminate_group(process)
            self.publish_status(state="failed", command_id=command_id, exit_code=None, error=str(exc))
        finally:
            with self.lock:
                self.process = None
                self.command_id = ""
                self.cancel_reason = ""

    def shutdown(self):
        with self.lock:
            process = self.process
            if process is not None:
                self.cancel_reason = "shutdown"
        if process is not None:
            self.terminate_group(process)


def main():
    rospy.init_node("eggy_shell_bridge")
    ShellBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
