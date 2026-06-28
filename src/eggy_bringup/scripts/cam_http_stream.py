#!/usr/bin/env python3
"""Lightweight MJPEG HTTP streamer. Serves camera feed on port 8080."""
import cv2, time, signal, sys
from http.server import HTTPServer, BaseHTTPRequestHandler

DEVICE = '/dev/orbbec_rgb23'
WIDTH, HEIGHT = 320, 240

cap = None

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/' or self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        time.sleep(0.05)
                        continue
                    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    jpg = buf.tobytes()
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(('Content-Length: %d\r\n\r\n' % len(jpg)).encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b'\r\n')
                    time.sleep(0.066)  # ~15fps
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
        elif self.path == '/snapshot':
            ret, frame = cap.read()
            if ret:
                _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                jpg = buf.tobytes()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(jpg))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(jpg)
            else:
                self.send_error(503, 'Camera read failed')
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><body style="margin:0;background:#000">')
            self.wfile.write(b'<img src="/stream" style="width:100%%">')
            self.wfile.write(b'</body></html>')

    def log_message(self, format, *args):
        pass  # suppress logs

def main():
    global cap
    cap = cv2.VideoCapture(DEVICE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        print('cannot open', DEVICE)
        sys.exit(1)
    print('camera opened: %dx%d' % (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

    server = HTTPServer(('0.0.0.0', 8080), StreamHandler)
    print('HTTP stream ready at http://0.0.0.0:8080/stream')
    server.serve_forever()

if __name__ == '__main__':
    main()
