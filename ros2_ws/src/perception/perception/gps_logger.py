import csv
import json
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


class GpsLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')

        # Parameters
        self.declare_parameter('log_path', '/home/ttkan/AutonomousBeachRobot/logs/detections.csv')
        self.declare_parameter('max_fix_age', 5.0)
        self.declare_parameter('min_confidence', 0.45)

        log_path = self.get_parameter('log_path').get_parameter_value().string_value
        self.max_fix_age = self.get_parameter('max_fix_age').get_parameter_value().double_value
        self.min_confidence = self.get_parameter('min_confidence').get_parameter_value().double_value

        # Create logs/ directory if missing
        os.makedirs(os.path.dirname(log_path), exist_ok=True)

        # Open CSV file in append mode; write header only for new files
        file_exists = os.path.exists(log_path)
        self.csv_file = open(log_path, 'a', newline='')
        self.writer = csv.writer(self.csv_file)
        if not file_exists:
            self.writer.writerow([
                'timestamp', 'latitude', 'longitude', 'gps_accuracy_m',
                'confidence', 'area', 'bbox_x1', 'bbox_y1', 'bbox_x2', 'bbox_y2'
            ])
            self.csv_file.flush()

        # State
        self.latest_fix: NavSatFix | None = None
        self.latest_fix_time: float | None = None

        # Subscribers
        self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        self.create_subscription(String, '/trash_detections', self.detection_callback, 10)

        self.get_logger().info(f'GPS logger started — writing to {log_path}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def fix_callback(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            self.get_logger().warn('No GPS fix — fix not stored')
            return

        self.latest_fix = msg
        self.latest_fix_time = self.get_clock().now().nanoseconds * 1e-9

    def detection_callback(self, msg: String):
        # Parse JSON safely
        try:
            detection = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Malformed detection JSON: {e}')
            return

        # Confidence threshold
        confidence = detection.get('conf', 0.0)
        if confidence < self.min_confidence:
            self.get_logger().debug(
                f'Detection confidence {confidence:.2f} below threshold '
                f'{self.min_confidence:.2f} — skipping'
            )
            return

        # GPS fix must be available
        if self.latest_fix is None or self.latest_fix_time is None:
            self.get_logger().warn('No GPS fix available — skipping detection log')
            return

        # GPS fix must not be stale
        now = self.get_clock().now().nanoseconds * 1e-9
        fix_age = now - self.latest_fix_time
        if fix_age > self.max_fix_age:
            self.get_logger().warn(
                f'GPS fix is stale ({fix_age:.1f}s old, max {self.max_fix_age}s) '
                f'— skipping detection log'
            )
            return

        self.write_row(detection, self.latest_fix)

    # ------------------------------------------------------------------
    # CSV writing
    # ------------------------------------------------------------------

    def write_row(self, detection: dict, fix: NavSatFix):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        lat = fix.latitude
        lon = fix.longitude
        accuracy = math.sqrt(fix.position_covariance[0]) if fix.position_covariance[0] >= 0 else float('nan')

        confidence = detection.get('conf', '')
        area = detection.get('area', '')
        bbox = detection.get('bbox', [None, None, None, None])
        bbox_x1, bbox_y1, bbox_x2, bbox_y2 = (bbox + [None] * 4)[:4]

        self.writer.writerow([
            timestamp, lat, lon, f'{accuracy:.1f}',
            confidence, area, bbox_x1, bbox_y1, bbox_x2, bbox_y2
        ])
        self.csv_file.flush()

        self.get_logger().info(
            f'Logged detection at {lat}, {lon} '
            f'(accuracy: {accuracy:.1f}m, conf: {confidence})'
        )

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info('CSV file closed cleanly')
        super().destroy_node()


def main():
    rclpy.init()
    node = GpsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
