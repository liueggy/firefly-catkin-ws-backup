#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Serve a ROS sensor_msgs/CompressedImage topic as a browser MJPEG stream."""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rospy
from sensor_msgs.msg import CompressedImage


latest_frame = None
latest_stamp = 0.0
latest_seq = 0
frame_cond = threading.Condition()


def on_image(msg):
    global latest_frame, latest_stamp, latest_seq
    with frame_cond:
        latest_frame = bytes(msg.data)
        latest_stamp = msg.header.stamp.to_sec() or time.time()
        latest_seq += 1
        frame_cond.notify_all()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = (
                b"<html><head><title>Robot Camera</title></head>"
                b"<body style='margin:0;background:#111;display:grid;place-items:center;height:100vh'>"
                b"<img src='/stream' style='max-width:100vw;max-height:100vh;object-fit:contain'/>"
                b"</body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/snapshot":
            frame = self.wait_frame(timeout=3.0)
            if frame is None:
                self.send_error(503, "No camera frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(frame)
            return

        if self.path != "/stream":
            self.send_error(404, "Not found")
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        last_sent_seq = -1
        while not rospy.is_shutdown():
            frame, seq = self.wait_new_frame(last_sent_seq, timeout=1.0)
            if frame is None:
                continue
            last_sent_seq = seq
            try:
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n\r\n" % len(frame)).encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

    def wait_frame(self, timeout):
        deadline = time.time() + timeout
        with frame_cond:
            while latest_frame is None and time.time() < deadline and not rospy.is_shutdown():
                frame_cond.wait(timeout=0.2)
            return latest_frame

    def wait_new_frame(self, previous_seq, timeout):
        deadline = time.time() + timeout
        with frame_cond:
            while latest_frame is None and time.time() < deadline and not rospy.is_shutdown():
                frame_cond.wait(timeout=0.2)
            while latest_seq <= previous_seq and time.time() < deadline and not rospy.is_shutdown():
                frame_cond.wait(timeout=0.2)
            return latest_frame, latest_seq

    def log_message(self, _fmt, *_args):
        return


def main():
    rospy.init_node("ros_camera_mjpeg_viewer", anonymous=False)
    topic = rospy.get_param("~image_topic", "/camera/front/image_source/compressed")
    port = int(rospy.get_param("~port", 8081))
    rospy.Subscriber(topic, CompressedImage, on_image, queue_size=1, buff_size=2**24)
    ThreadingHTTPServer.allow_reuse_address = True
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    rospy.loginfo("ROS camera MJPEG viewer: topic=%s url=http://0.0.0.0:%d/", topic, port)
    server.serve_forever()


if __name__ == "__main__":
    main()
