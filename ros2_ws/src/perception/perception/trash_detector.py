import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class TrashDetector(Node):
    def __init__(self):
        super().__init__('trash_detector')

        self.declare_parameter('model_path', 'models/trash_v1_best.onnx')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('yolo_interval', 0.1)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.yolo_interval = self.get_parameter('yolo_interval').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value

        model_path = self.get_parameter('model_path').value
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded YOLO model from {model_path}')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, '/trash_detections', 10)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.tracker = None
        self.tracker_bbox = None   # (x1, y1, x2, y2)
        self.tracker_conf = 0.0

        self.last_yolo_time = 0.0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # CSRT update every frame
        if self.tracker is not None:
            ok, bbox_xywh = self.tracker.update(frame)
            if ok:
                x, y, w, h = (int(v) for v in bbox_xywh)
                bbox = (x, y, x + w, y + h)
                self.tracker_bbox = bbox
                self.publish_detection(bbox, self.tracker_conf, tracking=True)
            else:
                self.tracker = None
                self.tracker_bbox = None

        # YOLO at reduced rate
        now = time.monotonic()
        if now - self.last_yolo_time >= self.yolo_interval:
            self.last_yolo_time = now
            best = self.run_yolo(frame)
            if best is not None:
                bbox, conf, score = best
                current_score = (
                    self.score_detection(self.tracker_bbox, self.tracker_conf)
                    if self.tracker_bbox is not None else -1.0
                )
                if score > current_score:
                    x1, y1, x2, y2 = bbox
                    self.tracker = cv2.TrackerCSRT_create()
                    self.tracker.init(frame, (x1, y1, x2 - x1, y2 - y1))
                    self.tracker_bbox = bbox
                    self.tracker_conf = conf
                    self.publish_detection(bbox, conf, tracking=False)

    def run_yolo(self, frame):
        results = self.model(frame, verbose=False)
        best = None
        best_score = -1.0

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self.conf_thresh:
                    continue
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                bbox = (x1, y1, x2, y2)
                score = self.score_detection(bbox, conf)
                if score > best_score:
                    best_score = score
                    best = (bbox, conf, score)

        return best

    def score_detection(self, bbox, confidence):
        if bbox is None:
            return -1.0
        x1, y1, x2, y2 = bbox
        area = (x2 - x1) * (y2 - y1)
        norm_area = area / (self.frame_width * self.frame_height)
        return 0.7 * norm_area + 0.3 * confidence

    def publish_detection(self, bbox, confidence, tracking):
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        area = (x2 - x1) * (y2 - y1)

        payload = {
            'cx': round(cx, 1),
            'cy': round(cy, 1),
            'area': area,
            'conf': round(confidence, 4),
            'tracking': tracking,
            'bbox': [x1, y1, x2, y2],
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = TrashDetector()
    rclpy.spin(node)
    rclpy.shutdown()
