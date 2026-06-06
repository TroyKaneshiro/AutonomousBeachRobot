import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
from ultralytics import YOLO


class TrashDetector(Node):
    """
    ROS 2 node that detects trash in camera frames and publishes the best
    target's bounding box to /trash_detections as a JSON string.

    Pipeline:
      - YOLO runs at ~10 fps (expensive) to find and score trash candidates.
      - CSRT tracker runs every frame (~30 fps) to smoothly follow the chosen
        target between YOLO cycles.
      - When CSRT loses the target it falls back to YOLO for re-acquisition.
      - When YOLO finds a higher-scoring target than the one being tracked,
        it re-initialises CSRT on the new target.
    """

    def __init__(self):
        super().__init__('trash_detector')

        # Declare all tunable values as ROS 2 parameters so they can be
        # overridden from a launch file without editing this source file.
        self.declare_parameter('model_path', '/home/ttkan/AutonomousBeachRobot/ml/models/trash_v1_best.onnx')
        self.declare_parameter('confidence_threshold', 0.15) #lowered for testing purposes
        self.declare_parameter('yolo_interval', 0.1)   # seconds between YOLO calls
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.yolo_interval = self.get_parameter('yolo_interval').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value

        # Load the YOLO model once at startup — inference is called per frame.
        model_path = self.get_parameter('model_path').value
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded YOLO model from {model_path}')

        # CvBridge converts ROS sensor_msgs/Image ↔ OpenCV BGR ndarray.
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(String, '/trash_detections', 10)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(
            Empty, '/trash_detector/reset', self.reset_callback, 10)

        # CSRT tracker state. None means no active tracker; a new one is
        # created each time YOLO identifies a better target.
        self.tracker = None
        self.tracker_bbox = None    # current tracked bbox as (x1, y1, x2, y2)
        self.tracker_conf = 0.0     # YOLO confidence of the tracked detection

        # Timestamp of the last YOLO inference (monotonic clock, seconds).
        # Initialised to 0 so YOLO runs immediately on the first frame.
        self.last_yolo_time = 0.0

    def reset_callback(self, msg):
        """Clear tracker state so YOLO re-acquires on the next inference cycle.
        Called by mission_fsm when transitioning PICKUP/FLAG → SCAN."""
        self.tracker = None
        self.tracker_bbox = None
        self.tracker_conf = 0.0
        self.get_logger().info('Tracker reset — re-acquiring target')

    def image_callback(self, msg):
        # Convert ROS image message to an OpenCV BGR frame.
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # --- CSRT update (runs every frame at camera rate ~30 fps) ---
        # This is cheap and keeps the published bbox smooth between YOLO cycles.
        if self.tracker is not None:
            ok, bbox_xywh = self.tracker.update(frame)
            if ok:
                # OpenCV trackers return (x, y, w, h); convert to (x1, y1, x2, y2).
                x, y, w, h = (int(v) for v in bbox_xywh)
                bbox = (x, y, x + w, y + h)
                self.tracker_bbox = bbox
                self.publish_detection(bbox, self.tracker_conf, tracking=True)
            else:
                # Tracker lost the target (occlusion, fast motion, etc.).
                # Clear state so YOLO can re-acquire on the next cycle.
                self.tracker = None
                self.tracker_bbox = None

        # --- YOLO inference (throttled to ~10 fps) ---
        now = time.monotonic()
        if now - self.last_yolo_time >= self.yolo_interval:
            self.last_yolo_time = now
            best = self.run_yolo(frame)
            if best is not None:
                bbox, conf, score = best

                # Only switch targets if YOLO found something better than what
                # we are already tracking. This prevents unnecessary tracker
                # resets when the current target is already well-tracked.
                current_score = (
                    self.score_detection(self.tracker_bbox, self.tracker_conf)
                    if self.tracker_bbox is not None else -1.0
                )
                if score > current_score:
                    # Re-initialise CSRT on the new best target.
                    # tracker.init() requires (x, y, w, h), not (x1, y1, x2, y2).
                    x1, y1, x2, y2 = bbox
                    self.tracker = cv2.TrackerCSRT_create()
                    self.tracker.init(frame, (x1, y1, x2 - x1, y2 - y1))
                    self.tracker_bbox = bbox
                    self.tracker_conf = conf
                    # Publish with tracking=False to signal a fresh YOLO acquisition.
                    self.publish_detection(bbox, conf, tracking=False)

    def run_yolo(self, frame):
        """Run YOLO inference and return the highest-scoring detection.

        Returns (bbox, conf, score) or None if no detection passes the
        confidence threshold.
        """
        # verbose=False suppresses per-frame console output from ultralytics.
        # device='cpu' forces ONNX Runtime to use the CPUExecutionProvider —
        # avoids the overhead of trying CUDA/TensorRT on a Raspberry Pi.
        results = self.model(frame, verbose=False, device='cpu')
        best = None
        best_score = -1.0

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self.conf_thresh:
                    continue
                # xyxy gives absolute pixel coords: x1, y1, x2, y2.
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                bbox = (x1, y1, x2, y2)
                score = self.score_detection(bbox, conf)
                if score > best_score:
                    best_score = score
                    best = (bbox, conf, score)

        return best

    def score_detection(self, bbox, confidence):
        """Score a detection to rank candidates against each other.

        score = 0.7 * normalised_area + 0.3 * confidence

        Normalised area is bbox_area / frame_area, so it is always 0–1.
        This weights proximity (bigger bbox = closer trash) more heavily
        than model confidence, with confidence as a tiebreaker.
        """
        if bbox is None:
            return -1.0
        x1, y1, x2, y2 = bbox
        area = (x2 - x1) * (y2 - y1)
        norm_area = area / (self.frame_width * self.frame_height)
        return 0.7 * norm_area + 0.3 * confidence

    def publish_detection(self, bbox, confidence, tracking):
        """Serialize the detection to JSON and publish on /trash_detections."""
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        area = (x2 - x1) * (y2 - y1)

        payload = {
            'cx': round(cx, 1),       # bbox centre x (pixels)
            'cy': round(cy, 1),       # bbox centre y (pixels)
            'area': area,             # proxy for distance — larger = closer
            'conf': round(confidence, 4),
            'tracking': tracking,     # True = CSRT frame, False = fresh YOLO hit
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
