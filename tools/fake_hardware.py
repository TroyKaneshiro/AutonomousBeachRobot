"""
fake_hardware.py — stub publisher for hardwareless testing.

Publishes the hardware topics that the real ESP32, IMU, and GPS would provide,
so the full ROS 2 pipeline can run without physical sensors.

Topics published:
  /wheel/odometry    nav_msgs/Odometry   — constant slow forward movement
                                            keeps STUCK watchdog happy
  /fix               sensor_msgs/NavSatFix — static GPS fix at a San Diego beach
  /battery_voltage   std_msgs/Float32    — 11.5 V nominal (above 10.5V cutoff)
  /imu/data          sensor_msgs/Imu     — flat-ground accelerations
  /motor_events      std_msgs/String     — silent (no stall events)

Usage:
  python3 tools/fake_hardware.py

Then in separate terminals:
  python3 tools/fake_camera.py <image_path>
  ros2 launch robot_bringup beach_robot.launch.py
  ros2 topic pub --once /mission_control/command std_msgs/msg/String "data: START"
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float32


class FakeHardware(Node):
    def __init__(self):
        super().__init__('fake_hardware')

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.battery_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Simulated odometry position — increment each tick so the STUCK
        # watchdog sees movement and never fires during testing.
        self._odom_x = 0.0

        self.create_timer(0.02, self._publish_odom)    # 50 Hz — matches real ESP32
        self.create_timer(0.2,  self._publish_gps)     # 5 Hz
        self.create_timer(1.0,  self._publish_battery) # 1 Hz
        self.create_timer(0.01, self._publish_imu)     # 100 Hz — matches real IMU

        self.get_logger().info(
            'fake_hardware running — publishing /wheel/odometry, /fix, '
            '/battery_voltage, /imu/data')

    def _publish_odom(self):
        # Increment x by 1 cm/tick so the FSM always sees movement.
        self._odom_x += 0.01

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self._odom_x
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0  # facing forward, no rotation
        self.odom_pub.publish(msg)

    def _publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        # Coronado Beach, San Diego — placeholder coordinates for testing
        msg.latitude = 32.6859
        msg.longitude = -117.1831
        msg.altitude = 0.0
        # position_covariance[0] is the east variance in m^2; sqrt(1.0) = 1m accuracy
        msg.position_covariance = [1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(msg)

    def _publish_battery(self):
        msg = Float32()
        msg.data = 11.5  # healthy 3S LiPo — above 10.5V cutoff
        self.battery_pub.publish(msg)

    def _publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu'
        # Flat ground: gravity along -Z, negligible X/Y acceleration.
        # atan2(ax, az) = atan2(0, -9.81) ≈ 0 rad pitch — within threshold.
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = -9.81
        msg.orientation.w = 1.0
        # Mark covariance as unknown (-1 diagonal) — terrain_monitor uses raw accel
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FakeHardware())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
