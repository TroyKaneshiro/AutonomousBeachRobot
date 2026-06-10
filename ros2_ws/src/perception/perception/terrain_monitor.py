import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String


class TerrainMonitor(Node):
    """
    Publishes /terrain_events when terrain ahead becomes unsafe.

    Two independent safety channels share the same output topic:
      STOP_IMU — pitch too steep or vibration spike (IMU)
      STOP_CAM — sand ratio in bottom-third ROI below threshold (camera)
      CLEAR     — channel that fired STOP has returned to safe range

    The mission_fsm subscribes to /terrain_events and halts on any STOP.
    CLEAR is informational; the FSM resumes naturally via its own state logic.
    """

    def __init__(self):
        super().__init__('terrain_monitor')

        # Declare all tunable values as ROS 2 parameters so they can be
        # overridden from robot_params.yaml without editing this file.
        # HSV bounds must be calibrated at the actual test location.
        self.declare_parameter('pitch_threshold', 0.15)         # rad, ~8.6°
        self.declare_parameter('vibration_multiplier', 2.5)     # × baseline variance
        self.declare_parameter('sand_ratio_threshold', 0.5)     # fraction of ROI
        self.declare_parameter('hsv_lower', [10, 20, 150])      # tune on-site
        self.declare_parameter('hsv_upper', [25, 255, 255])     # tune on-site

        self.pitch_threshold = self.get_parameter('pitch_threshold').value
        self.vibration_multiplier = self.get_parameter('vibration_multiplier').value
        self.sand_ratio_threshold = self.get_parameter('sand_ratio_threshold').value
        self.hsv_lower = list(self.get_parameter('hsv_lower').value)
        self.hsv_upper = list(self.get_parameter('hsv_upper').value)

        self.publisher = self.create_publisher(String, '/terrain_events', 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        self.cv_bridge = CvBridge()

        # Per-channel safe/unsafe state — only publish transitions, not every frame.
        self.imu_safe = True
        self.terrain_safe = True

        # IMU calibration: collect 100 samples (~1 s at 100 Hz) to establish
        # the baseline accel_z variance for smooth, flat ground.
        self.imu_calibrated = False
        self.baseline_variance = 0.0
        self.calibration_samples = 0
        self.CAL_TARGET = 100
        self.accel_z_history = deque(maxlen=100)

        # Camera throttle — limit HSV classification to 10 fps.
        self.last_camera_time = 0.0
        self.CAMERA_THROTTLE = 0.1

        self.get_logger().info('TerrainMonitor started — calibrating IMU baseline')

    def imu_callback(self, msg: Imu):
        self.accel_z_history.append(msg.linear_acceleration.z)

        if not self.imu_calibrated:
            self.calibration_samples += 1
            if self.calibration_samples >= self.CAL_TARGET:
                self.imu_calibrated = True
                self.baseline_variance = np.var(list(self.accel_z_history))
                self.get_logger().info(
                    f'IMU calibrated — baseline variance: {self.baseline_variance:.6f}')
            return

        ax = msg.linear_acceleration.x
        az = msg.linear_acceleration.z
        current_pitch = math.atan2(ax, az)
        current_variance = np.var(list(self.accel_z_history))

        safe = (
            abs(current_pitch) < self.pitch_threshold
            and current_variance < self.baseline_variance * self.vibration_multiplier
        )

        if self.imu_safe and not safe:
            self.imu_safe = False
            self.publisher.publish(String(data='STOP_IMU'))
        elif not self.imu_safe and safe:
            self.imu_safe = True
            self.publisher.publish(String(data='CLEAR'))

    def camera_callback(self, msg: Image):
        now = time.monotonic()
        if now - self.last_camera_time < self.CAMERA_THROTTLE:
            return
        self.last_camera_time = now

        frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        safe, ratio = self.is_sand(frame)

        if self.terrain_safe and not safe:
            self.terrain_safe = False
            self.publisher.publish(String(data='STOP_CAM'))
        elif not self.terrain_safe and safe:
            self.terrain_safe = True
            self.publisher.publish(String(data='CLEAR'))

    def is_sand(self, frame):
        # Classify only the bottom third of the frame — terrain immediately ahead.
        roi = frame[int(frame.shape[0] * 0.67):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.hsv_lower), np.array(self.hsv_upper))
        sand_ratio = np.sum(mask > 0) / mask.size
        return sand_ratio >= self.sand_ratio_threshold, sand_ratio


def main():
    rclpy.init()
    node = TerrainMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
