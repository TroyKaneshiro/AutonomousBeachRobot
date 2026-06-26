import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

_PAGE = b"""<!DOCTYPE html>
<html><head><title>Beach Robot Camera</title></head>
<body style="margin:0;background:#000">
<img src="/stream" style="display:block;max-width:100%">
</body></html>"""

_node = None


class StreamNode(Node):
    def __init__(self):
        super().__init__('stream_camera')
        self._lock = threading.Lock()
        self._frame = None
        self._bbox = None
        self._detection = None
        self._bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)
        self.create_subscription(String, '/trash_detections', self._on_detection, 10)
        self.get_logger().info('stream_camera ready — waiting for /camera/image_raw')

    def _on_image(self, msg):
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        with self._lock:
            self._frame = frame.copy()

    def _on_detection(self, msg):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._bbox = data.get('bbox')
                self._detection = data
        except (json.JSONDecodeError, AttributeError):
            pass

    def get_annotated_jpeg(self):
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()
            bbox = self._bbox
            detection = self._detection

        if bbox is not None:
            x1, y1, x2, y2 = (int(v) for v in bbox)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = 'trash'
            if detection:
                conf = detection.get('conf', 0)
                mode = 'tracking' if detection.get('tracking') else 'YOLO'
                label = f'trash {conf:.2f} [{mode}]'
            cv2.putText(frame, label, (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # Status bar at the bottom of the frame
        h, w = frame.shape[:2]
        if bbox is not None and detection:
            conf = detection.get('conf', 0)
            mode = 'TRACKING' if detection.get('tracking') else 'YOLO HIT'
            status = f'{mode}  conf={conf:.2f}  cx={detection.get("cx", 0):.0f}  cy={detection.get("cy", 0):.0f}'
            color = (0, 200, 255) if detection.get('tracking') else (0, 255, 0)
        else:
            status = 'Scanning — no detection'
            color = (180, 180, 180)

        cv2.rectangle(frame, (0, h - 28), (w, h), (30, 30, 30), -1)
        cv2.putText(frame, status, (8, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return bytes(buf) if ok else None


class MjpegHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # silence per-request console noise

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(_PAGE)))
            self.end_headers()
            self.wfile.write(_PAGE)

        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    if _node is None:
                        continue
                    jpg = _node.get_annotated_jpeg()
                    if jpg is None:
                        continue
                    header = (
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(jpg)).encode() + b'\r\n\r\n'
                    )
                    self.wfile.write(header + jpg + b'\r\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

        else:
            self.send_response(404)
            self.end_headers()


def main():
    global _node

    parser = argparse.ArgumentParser(description='Stream /camera/image_raw over HTTP MJPEG')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()

    rclpy.init()
    _node = StreamNode()

    server = HTTPServer(('0.0.0.0', args.port), MjpegHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    _node.get_logger().info(f'Streaming at http://0.0.0.0:{args.port}')

    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        _node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
