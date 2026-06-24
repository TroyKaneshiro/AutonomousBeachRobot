# MOVED to ros2_ws/src/perception/perception/debug_detector.py
# This copy is kept for reference only — run via: ros2 run perception debug_detector

from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

_MODEL_PATH = str(Path(__file__).resolve().parents[1] / 'ml' / 'models' / 'trash_v1_best.onnx')

class DebugDetector(Node):
    def __init__(self):
        super().__init__('debug_detector')
        self.bridge = CvBridge()
        self.model = YOLO(_MODEL_PATH, task='detect')
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        self.count = 0

    def cb(self, msg):
        self.count += 1
        if self.count % 30 != 0:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, verbose=False, device='cpu', conf=0.05)
        boxes = results[0].boxes
        print(f'Frame {self.count}: {len(boxes)} detections')
        for box in boxes:
            print(f'  conf={float(box.conf[0]):.3f}')

def main():
    rclpy.init()
    rclpy.spin(DebugDetector())

if __name__ == '__main__':
    main()
