#!/usr/bin/env python3
"""
eggy_camera_node.py — low-CPU MJPEG passthrough node

Fast path:
  Dabai DC1 UVC camera outputs MJPEG directly.
  v4l2-ctl streams frame-headered JPEG packets.
  Node publishes those bytes as sensor_msgs/CompressedImage.

This avoids OpenCV BGR decode + cv2.imencode JPEG re-encode on RK3568.
"""
import struct
import subprocess
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image


def bool_param(name, default=False):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def read_exact(stream, n):
    chunks = []
    remain = n
    while remain > 0 and not rospy.is_shutdown():
        chunk = stream.read(remain)
        if not chunk:
            return None
        chunks.append(chunk)
        remain -= len(chunk)
    return b''.join(chunks)


def read_next_v4l2_hdr(stream):
    """Read until the frame-header magic b'rdhV' appears, then return 8-byte header.

    v4l2-ctl may print human text such as 'Frame rate set...' to stdout before
    the binary stream, so strict read_exact(8) is not reliable on this board.
    """
    magic = b'rdhV'
    window = b''
    while not rospy.is_shutdown():
        ch = stream.read(1)
        if not ch:
            return None
        window = (window + ch)[-4:]
        if window == magic:
            rest = read_exact(stream, 4)
            if not rest:
                return None
            return magic + rest
    return None


def parse_v4l2_hdr(hdr):
    if len(hdr) != 8 or hdr[:4] != b'rdhV':
        return None
    be = struct.unpack('>I', hdr[4:8])[0]
    le = struct.unpack('<I', hdr[4:8])[0]
    # Typical 640x480 MJPEG frame is 10-80KB. Max image size reported by UVC is ~615KB.
    if 0 < be < 2_000_000:
        return be
    if 0 < le < 2_000_000:
        return le
    return None


def trim_jpeg(data):
    start = data.find(b'\xff\xd8')
    end = data.rfind(b'\xff\xd9')
    if start >= 0 and end >= start:
        return data[start:end + 2]
    return data


def publish_raw_from_bgr(pub_raw, frame, stamp, frame_id, width, height):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = width * 3
    msg.data = rgb.tobytes()
    pub_raw.publish(msg)


def run_v4l2ctl_mjpeg(device, width, height, fps, frame_id, pub_comp, pub_raw, compressed_topic):
    cmd = [
        'v4l2-ctl', '-d', device,
        f'--set-fmt-video=width={width},height={height},pixelformat=MJPG',
        f'--set-parm={int(fps)}',
        '--stream-mmap=3',
        '--stream-to-hdr=-',
    ]
    rospy.loginfo('Camera: %s %dx%d MJPG @ %.0ffps [v4l2-ctl MJPEG passthrough]',
                  device, width, height, fps)
    rospy.loginfo('Publishing: %s (camera JPEG bytes, no CPU re-encode)', compressed_topic)
    if pub_raw:
        rospy.loginfo('           /camera/front/image_raw enabled only when subscribed; requires JPEG decode')
    else:
        rospy.loginfo('           raw image publishing disabled (~use_raw=false)')

    proc = None
    frame_count = 0
    last_log = time.time()

    while not rospy.is_shutdown():
        if proc is None or proc.poll() is not None:
            if proc is not None:
                rospy.logwarn('v4l2-ctl exited with code %s, restarting...', proc.poll())
                time.sleep(1.0)
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
            frame_count = 0

        hdr = read_next_v4l2_hdr(proc.stdout)
        if not hdr:
            try:
                proc.kill()
            except Exception:
                pass
            proc = None
            continue

        size = parse_v4l2_hdr(hdr)
        if not size:
            rospy.logwarn('Bad v4l2 frame header: %r', hdr)
            try:
                proc.kill()
            except Exception:
                pass
            proc = None
            continue

        data = read_exact(proc.stdout, size)
        if not data:
            try:
                proc.kill()
            except Exception:
                pass
            proc = None
            continue

        jpg_data = trim_jpeg(data)
        stamp = rospy.Time.now()

        comp_msg = CompressedImage()
        comp_msg.header.stamp = stamp
        comp_msg.header.frame_id = frame_id
        comp_msg.format = 'jpeg'
        comp_msg.data = jpg_data
        pub_comp.publish(comp_msg)

        if pub_raw and pub_raw.get_num_connections() > 0:
            decoded = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if decoded is not None:
                publish_raw_from_bgr(pub_raw, decoded, stamp, frame_id, width, height)

        frame_count += 1
        now = time.time()
        if now - last_log > 30:
            rospy.loginfo('MJPEG passthrough alive: %d frames in last %.1fs', frame_count, now - last_log)
            frame_count = 0
            last_log = now

    if proc is not None:
        try:
            proc.terminate()
        except Exception:
            pass


def run_opencv_fallback(device, width, height, fps, pixel_format, frame_id, jpeg_quality, pub_comp, pub_raw):
    fourcc = pixel_format.upper()
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        rospy.logerr('Cannot open camera: %s', device)
        return
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    for _ in range(5):
        cap.read(); time.sleep(0.05)
    rospy.loginfo('Camera: %s %dx%d %s @ %.0ffps [OpenCV software JPEG encode]',
                  device, width, height, fourcc, fps)
    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rate.sleep(); continue
        stamp = rospy.Time.now()
        ok, jpg_buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if ok:
            comp_msg = CompressedImage()
            comp_msg.header.stamp = stamp
            comp_msg.header.frame_id = frame_id
            comp_msg.format = 'jpeg'
            comp_msg.data = jpg_buf.tobytes()
            pub_comp.publish(comp_msg)
        if pub_raw and pub_raw.get_num_connections() > 0:
            publish_raw_from_bgr(pub_raw, frame, stamp, frame_id, width, height)
        rate.sleep()
    cap.release()


def main():
    rospy.init_node('eggy_camera')

    device = rospy.get_param('~device', '/dev/orbbec_rgb23')
    width = int(rospy.get_param('~width', 640))
    height = int(rospy.get_param('~height', 480))
    fps = float(rospy.get_param('~fps', 15.0))
    pixel_format = rospy.get_param('~pixel_format', 'mjpg').strip().lower()
    frame_id = rospy.get_param('~frame_id', 'camera_rgb_frame')
    jpeg_quality = int(rospy.get_param('~jpeg_quality', 80))
    use_raw = bool_param('~use_raw', False)
    mjpeg_passthrough = bool_param('~mjpeg_passthrough', pixel_format in ('mjpg', 'mjpeg'))
    compressed_topic = rospy.get_param('~compressed_topic', '/camera/front/image/compressed')

    pub_comp = rospy.Publisher(compressed_topic, CompressedImage, queue_size=2)
    pub_raw = rospy.Publisher('/camera/front/image_raw', Image, queue_size=2) if use_raw else None

    if mjpeg_passthrough:
        run_v4l2ctl_mjpeg(device, width, height, fps, frame_id, pub_comp, pub_raw, compressed_topic)
    else:
        run_opencv_fallback(device, width, height, fps, pixel_format, frame_id, jpeg_quality, pub_comp, pub_raw)


if __name__ == '__main__':
    main()
