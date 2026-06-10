import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
from collections import deque

# Terrain monitor for the Autonomous Beach Robot - detects changes in terrain to find
# unsafe terrain conditions, can stop the robot in case of danger
class TerrainMonitor(Node):
    def __init__(self):
        super().__init__('terrain_monitor')

        # Parameters
        # Publisher: /terrain_events
        self.publisher = self.create_publisher(String, '/terrain_events', 10)
        # Subscribers: /camera/image_raw, /imu/data
        self.subscriber_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.subscriber_camera = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.cv_bridge = CvBridge()
        self.pitch_threshold = 0.15 #allows 8.6 degrees of pitch
        self.vibration_multiplier = 2.5 #vibratino may be up to 2.5 times normal (possibly tweak)
        self.sand_ratio_threshold = 0.5 #tune at location, amount of ground allowed to be sand in bottom third
        self.camera_interval = 0.1
        self.hsv_lower = [10, 20, 150] #tune at location
        self.hsv_upper = [25, 255, 255] #tune at location
        self.terrain_safe = True
        self.imu_safe = True

        # IMU calibration state
        self.imu_calibrated = False
        self.baseline_variance = 0.0
        self.calibration_samples = 0
        self.CAL_TARGET = 100 #100 calibration samples until baseline (1 second)
        self.accel_z_history = deque(maxlen=100)
        # Camera throttle timer
        self.last_camera_time = 0.0
        self.CAMERA_THROTTLE = 0.1 # 10fps

    def imu_callback(self, msg):
        # Phase 1: collect calibration samples
        current_time = time.time()
        self.accel_z_history.append(msg.linear_acceleration.z)
        if not self.imu_calibrated:
            self.calibration_samples += 1
            if self.calibration_samples >= self.CAL_TARGET:
                self.imu_calibrated = True
                self.baseline_variance = np.var(list(self.accel_z_history)) #smooth sand should have low var
        else:
            # Phase 2: check pitch threshold
            ax = msg.linear_acceleration.x
            az = msg.linear_acceleration.z
            current_pitch = math.atan2(ax, az)   # actual pitch in radians
            #current ground not safe if pitch or vibrations too high
            safe = (abs(current_pitch) < self.pitch_threshold and
             current_variance < self.baseline_variance * self.vibration_multiplier) 
            if self.imu_safe and not safe: #terrain was safe, now is NOT safe - stop 
                self.imu_safe = False 
                self.publisher.publish(String(data='STOP_IMU'))
            elif not self.imu_safe and safe: #terrain was NOT safe, now is 
                self.imu_safe = True
                self.publisher.publish(String(data='CLEAR'))
            

    def camera_callback(self, msg):
        # Throttle to 10fps
        current_time = time.time()
        if current_time - self.last_camera_time < self.CAMERA_THROTTLE:
            return
        self.last_camera_time = current_time
        # Convert to OpenCV
        frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Publish STOP_CAM if sand_ratio drops below threshold
        # Publish CLEAR if terrain looks safe
        safe, ratio = self.is_sand(frame)
        if self.terrain_safe and not safe: #terrain was safe, now is NOT safe - stop camera
            self.terrain_safe = False 
            self.publisher.publish(String(data='STOP_CAM'))
        elif not self.terrain_safe and safe: #terrain was NOT safe, now is 
            self.terrain_safe = True
            self.publisher.publish(String(data='CLEAR'))



    def is_sand(self, frame):
        roi = frame[int(frame.shape[0] * 0.67):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
            np.array(self.hsv_lower),
            np.array(self.hsv_upper))
        sand_ratio = np.sum(mask > 0) / mask.size
        return sand_ratio >= self.sand_ratio_threshold, sand_ratio


def main():
    rclpy.init()
    node = TerrainMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()