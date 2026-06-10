import json
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, String


class Coordinator(Node):
    """
    High-level operator interface that sits alongside the mission FSM.

    Responsibilities
    ----------------
    - Track session statistics: items flagged, safety events, battery level.
    - Publish /mission_status (JSON) at 1 Hz for monitoring and telemetry.
    - Accept operator commands on /mission_control/command:
        "START"  — arm the mission; sends /e_stop False so the ESP32 enables motors
        "STOP"   — disarm; sends /e_stop True and zero /cmd_vel
        "RESET"  — clear session counters without stopping
    - Publish /e_stop (Bool) for the ESP32 firmware to cut motor power immediately.
      The FSM continues to publish /cmd_vel, but the ESP32 ignores it while
      e_stop=True. This avoids a ROS-level cmd_vel publish race.

    This node does NOT participate in the real-time control loop —
    all navigation decisions stay inside mission_fsm.
    """

    def __init__(self):
        super().__init__('coordinator')

        # --- Publishers ---
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        # /e_stop True = motors cut; False = motors enabled.
        # The ESP32 firmware subscribes to this topic.
        self.e_stop_pub = self.create_publisher(Bool, '/e_stop', 10)
        # Zero cmd_vel sent on STOP to flush any in-flight velocity command.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_pub = self.create_publisher(Empty, '/trash_detector/reset', 10)

        # --- Subscribers ---
        self.create_subscription(
            String, '/mission_control/command', self._on_command, 10)
        self.create_subscription(
            String, '/trash_detections', self._on_detection, 10)
        self.create_subscription(
            Float32, '/battery_voltage', self._on_battery, 10)
        self.create_subscription(
            String, '/terrain_events', self._on_terrain_event, 10)
        self.create_subscription(
            String, '/motor_events', self._on_motor_event, 10)

        # --- Session state ---
        self.running = False
        self.session_start = time.monotonic()
        self.items_flagged = 0
        self.terrain_event_count = 0
        self.motor_event_count = 0
        self.battery_voltage = None
        self.last_conf = None
        self.last_detection_time = None

        # Publish status at 1 Hz
        self.create_timer(1.0, self._publish_status)

        # Publish /e_stop=True at startup — operator must send "START" explicitly.
        # This ensures the robot doesn't drive on accidental node restart.
        self._set_e_stop(True)
        self.get_logger().info(
            'Coordinator ready — motors DISABLED. '
            'Publish "START" to /mission_control/command to arm.')

    # ------------------------------------------------------------------ #
    # Operator command handler                                             #
    # ------------------------------------------------------------------ #

    def _on_command(self, msg):
        cmd = msg.data.strip().upper()

        if cmd == 'START':
            if self.running:
                self.get_logger().info('Already running — ignoring START')
                return
            self.running = True
            self.session_start = time.monotonic()
            self._set_e_stop(False)
            self.get_logger().info('Mission STARTED — motors enabled')

        elif cmd == 'STOP':
            self.running = False
            # Zero cmd_vel first to flush any in-flight command, then cut power.
            self.cmd_vel_pub.publish(Twist())
            self._set_e_stop(True)
            self.reset_pub.publish(Empty())
            self.get_logger().info('Mission STOPPED — motors disabled, tracker reset')

        elif cmd == 'RESET':
            self.items_flagged = 0
            self.terrain_event_count = 0
            self.motor_event_count = 0
            self.session_start = time.monotonic()
            self.last_conf = None
            self.last_detection_time = None
            self.get_logger().info('Session counters reset')

        else:
            self.get_logger().warning(
                f'Unknown command "{cmd}" — valid: START, STOP, RESET')

    # ------------------------------------------------------------------ #
    # Monitoring callbacks                                                 #
    # ------------------------------------------------------------------ #

    def _on_detection(self, msg):
        try:
            det = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self.last_conf = det.get('conf')
        self.last_detection_time = time.monotonic()

        # Count items flagged by watching for tracking=False (fresh YOLO hit after
        # a PICKUP reset). A PICKUP reset clears the tracker, so the next detection
        # will always be tracking=False — use that as a "new item" signal.
        # This is a heuristic counter; the authoritative record is the CSV log.
        if not det.get('tracking', True):
            # Only increment if this looks like a post-reset re-acquisition
            # (i.e. a brief silence preceded this detection).
            if (self.last_detection_time is None or
                    time.monotonic() - (self.last_detection_time or 0) > 1.0):
                self.items_flagged += 1

    def _on_battery(self, msg):
        self.battery_voltage = round(msg.data, 2)

    def _on_terrain_event(self, msg):
        if msg.data in ('STOP_CAM', 'STOP_IMU'):
            self.terrain_event_count += 1
            self.get_logger().info(f'Terrain event #{self.terrain_event_count}: {msg.data}')

    def _on_motor_event(self, msg):
        if 'STALL' in msg.data or 'HIGH_CURRENT' in msg.data:
            self.motor_event_count += 1
            self.get_logger().info(f'Motor event #{self.motor_event_count}: {msg.data}')

    # ------------------------------------------------------------------ #
    # Status publisher (1 Hz)                                              #
    # ------------------------------------------------------------------ #

    def _publish_status(self):
        uptime = time.monotonic() - self.session_start

        # Seconds since last detection — None if never seen one this session.
        detection_age = None
        if self.last_detection_time is not None:
            detection_age = round(time.monotonic() - self.last_detection_time, 1)

        payload = {
            'running': self.running,
            'uptime_s': round(uptime, 1),
            'items_flagged': self.items_flagged,
            'battery_v': self.battery_voltage,
            'last_detection_conf': self.last_conf,
            'detection_age_s': detection_age,
            'terrain_events': self.terrain_event_count,
            'motor_events': self.motor_event_count,
            'timestamp': datetime.utcnow().isoformat(),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _set_e_stop(self, active: bool):
        msg = Bool()
        msg.data = active
        self.e_stop_pub.publish(msg)


def main():
    rclpy.init()
    node = Coordinator()
    rclpy.spin(node)
    rclpy.shutdown()
