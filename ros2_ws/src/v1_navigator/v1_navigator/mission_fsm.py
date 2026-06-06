import csv
import enum
import json
import math
import os
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, Float32, String


class State(enum.Enum):
    CALIBRATE = 'CALIBRATE'
    SCAN = 'SCAN'
    TRACK = 'TRACK'
    PICKUP = 'PICKUP'
    STUCK = 'STUCK'
    LOW_BATTERY = 'LOW_BATTERY'


class MissionFSM(Node):
    """
    Mission state machine for the Autonomous Beach Robot.

    States
    ------
    CALIBRATE   Hold still 1.5 s so the IMU can lock a variance baseline.
    SCAN        Rotate at 0.3 rad/s; wait for a high-confidence detection.
    TRACK       Proportional visual servoing toward the tracked target.
    PICKUP      V1: log GPS + confidence to CSV, reset tracker, return to SCAN.
    STUCK       No movement for 5 s despite cmd_vel → reverse + rotate, resume.
    LOW_BATTERY Voltage below 3S cutoff → stop all motion, await operator.

    Key interfaces
    --------------
    Subscribes: /trash_detections, /terrain_events, /motor_events,
                /wheel/odometry, /battery_voltage, /fix
    Publishes:  /cmd_vel, /trash_detector/reset
    """

    def __init__(self):
        super().__init__('mission_fsm')

        # --- Parameters (overridable from launch file) ---
        self.declare_parameter('kp', 0.002)
        # Proportional gain for visual servoing: angular_z = -kp * pixel_error

        self.declare_parameter('approach_speed', 0.2)
        # m/s forward speed while closing in on a target

        self.declare_parameter('scan_angular_speed', 0.3)
        # rad/s rotation during SCAN and STUCK rotate phase

        self.declare_parameter('pickup_area_threshold', 0.15)
        # Normalised bbox area (bbox_px / frame_px) that triggers PICKUP.
        # Calibrate this at your actual physical pickup distance.

        self.declare_parameter('detection_conf_threshold', 0.45)
        # Minimum YOLO confidence to accept a detection in SCAN.

        self.declare_parameter('track_loss_timeout', 0.5)
        # Seconds without a detection in TRACK before falling back to SCAN.

        self.declare_parameter('calibrate_duration', 1.5)
        self.declare_parameter('stuck_timeout', 5.0)
        self.declare_parameter('battery_cutoff_voltage', 10.5)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('csv_log_path', 'trash_detections_log.csv')

        self.kp = self.get_parameter('kp').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.scan_angular_speed = self.get_parameter('scan_angular_speed').value
        self.pickup_area_threshold = self.get_parameter('pickup_area_threshold').value
        self.detection_conf_threshold = self.get_parameter('detection_conf_threshold').value
        self.track_loss_timeout = self.get_parameter('track_loss_timeout').value
        self.calibrate_duration = self.get_parameter('calibrate_duration').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.battery_cutoff_voltage = self.get_parameter('battery_cutoff_voltage').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.csv_log_path = self.get_parameter('csv_log_path').value

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_pub = self.create_publisher(Empty, '/trash_detector/reset', 10)

        # --- Subscribers ---
        self.create_subscription(String, '/trash_detections', self._on_detection, 10)
        self.create_subscription(String, '/terrain_events', self._on_terrain_event, 10)
        self.create_subscription(String, '/motor_events', self._on_motor_event, 10)
        self.create_subscription(Odometry, '/wheel/odometry', self._on_odometry, 10)
        self.create_subscription(Float32, '/battery_voltage', self._on_battery, 10)
        self.create_subscription(NavSatFix, '/fix', self._on_gps, 10)

        # --- State machine ---
        self.state = State.CALIBRATE
        self.prior_state = State.SCAN      # state to resume after STUCK recovery
        self.state_enter_time = time.monotonic()

        # --- Detection cache ---
        self.latest_detection = None       # most recent parsed JSON dict
        self.last_detection_time = 0.0     # monotonic timestamp of last detection

        # --- GPS cache ---
        self.latest_fix = None             # most recent NavSatFix message

        # --- Odometry / STUCK watchdog ---
        self.last_odom_pos = None          # (x, y) from previous odometry message
        self.last_movement_time = time.monotonic()
        self.cmd_vel_nonzero = False       # True when publishing non-zero velocity

        # --- STUCK recovery sub-state ---
        self.stuck_phase = 'reverse'       # 'reverse' | 'rotate'
        self.stuck_phase_start = 0.0

        # --- CSV log ---
        self._init_csv()

        # Main control loop at 10 Hz
        self.create_timer(0.1, self._tick)
        self.get_logger().info('MissionFSM started — state: CALIBRATE')

    # ------------------------------------------------------------------ #
    # CSV initialisation                                                   #
    # ------------------------------------------------------------------ #

    def _init_csv(self):
        write_header = not os.path.exists(self.csv_log_path)
        self.csv_file = open(self.csv_log_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if write_header:
            self.csv_writer.writerow(['timestamp_utc', 'lat', 'lon', 'conf', 'bbox_area'])

    # ------------------------------------------------------------------ #
    # Subscription callbacks                                               #
    # ------------------------------------------------------------------ #

    def _on_detection(self, msg):
        try:
            self.latest_detection = json.loads(msg.data)
            self.last_detection_time = time.monotonic()
        except json.JSONDecodeError:
            self.get_logger().warning('Malformed /trash_detections JSON — ignoring')

    def _on_terrain_event(self, msg):
        # Camera or IMU raised a safety boundary — stop and re-scan.
        # Ignored while already in a safety state.
        if msg.data in ('STOP_CAM', 'STOP_IMU'):
            if self.state not in (State.STUCK, State.LOW_BATTERY):
                self.get_logger().warning(f'Terrain event: {msg.data} — halting')
                self._stop()
                self._enter_state(State.SCAN)

    def _on_motor_event(self, msg):
        # Stall or over-current from ESP32 triggers STUCK recovery.
        if 'STALL' in msg.data or 'HIGH_CURRENT' in msg.data:
            if self.state not in (State.STUCK, State.LOW_BATTERY):
                self.get_logger().warning(f'Motor event: {msg.data} — entering STUCK')
                self._enter_stuck()

    def _on_odometry(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_odom_pos is not None:
            dx = x - self.last_odom_pos[0]
            dy = y - self.last_odom_pos[1]
            if math.hypot(dx, dy) > 0.01:   # moved more than 1 cm
                self.last_movement_time = time.monotonic()
        self.last_odom_pos = (x, y)

    def _on_battery(self, msg):
        if msg.data < self.battery_cutoff_voltage:
            if self.state != State.LOW_BATTERY:
                self.get_logger().error(
                    f'Battery critical: {msg.data:.2f} V — stopping all motion')
                self._stop()
                self._enter_state(State.LOW_BATTERY)

    def _on_gps(self, msg):
        self.latest_fix = msg

    # ------------------------------------------------------------------ #
    # Main tick (10 Hz)                                                    #
    # ------------------------------------------------------------------ #

    def _tick(self):
        now = time.monotonic()

        # STUCK watchdog: if we have been commanding motion and odometry
        # shows no movement for stuck_timeout seconds, trigger recovery.
        if self.state in (State.SCAN, State.TRACK) and self.cmd_vel_nonzero:
            if now - self.last_movement_time > self.stuck_timeout:
                self.get_logger().warning('STUCK: no odometry movement for 5 s')
                self._enter_stuck()
                return

        if self.state == State.CALIBRATE:
            self._tick_calibrate(now)
        elif self.state == State.SCAN:
            self._tick_scan()
        elif self.state == State.TRACK:
            self._tick_track(now)
        elif self.state == State.PICKUP:
            self._tick_pickup()
        elif self.state == State.STUCK:
            self._tick_stuck(now)
        elif self.state == State.LOW_BATTERY:
            pass  # await operator intervention

    # ------------------------------------------------------------------ #
    # Per-state tick handlers                                              #
    # ------------------------------------------------------------------ #

    def _tick_calibrate(self, now):
        # Hold completely still so the IMU can establish its variance baseline.
        self._stop()
        if now - self.state_enter_time >= self.calibrate_duration:
            self.get_logger().info('Calibration done — entering SCAN')
            self._enter_state(State.SCAN)

    def _tick_scan(self):
        # Rotate slowly so the camera sweeps the environment.
        twist = Twist()
        twist.angular.z = self.scan_angular_speed
        self._publish_cmd(twist)

        det = self.latest_detection
        if det is None:
            return

        conf = det.get('conf', 0.0)
        if conf >= self.detection_conf_threshold:
            self.get_logger().info(
                f'Target acquired conf={conf:.3f} — entering TRACK')
            self._enter_state(State.TRACK)

    def _tick_track(self, now):
        # Detection timeout: if /trash_detections goes silent, target is lost.
        if now - self.last_detection_time > self.track_loss_timeout:
            self.get_logger().info('Target lost (detection timeout) — returning to SCAN')
            self._stop()
            self._enter_state(State.SCAN)
            return

        det = self.latest_detection
        if det is None:
            return

        cx = det.get('cx', self.frame_width / 2.0)
        area = det.get('area', 0)
        norm_area = area / (self.frame_width * self.frame_height)

        # Switch to PICKUP when the target fills enough of the frame.
        if norm_area >= self.pickup_area_threshold:
            self.get_logger().info(
                f'Close enough (norm_area={norm_area:.3f}) — entering PICKUP')
            self._stop()
            self._enter_state(State.PICKUP)
            return

        # Proportional controller: steer to centre the target horizontally.
        # error > 0 → target is right of centre → turn right (negative angular_z).
        pixel_error = cx - self.frame_width / 2.0
        twist = Twist()
        twist.linear.x = self.approach_speed
        twist.angular.z = -self.kp * pixel_error
        self._publish_cmd(twist)

    def _tick_pickup(self):
        # V1 behaviour: log GPS coordinates to CSV and return to SCAN.
        det = self.latest_detection
        conf = det.get('conf', 0.0) if det else 0.0
        area = det.get('area', 0) if det else 0

        lat, lon = None, None
        if self.latest_fix is not None:
            lat = self.latest_fix.latitude
            lon = self.latest_fix.longitude

        self.csv_writer.writerow([
            datetime.utcnow().isoformat(),
            lat,
            lon,
            round(conf, 4),
            area,
        ])
        self.csv_file.flush()
        self.get_logger().info(f'Flagged trash → lat={lat}, lon={lon}, conf={conf:.3f}')

        # Tell trash_detector to forget this target so the next SCAN is fresh.
        self.reset_pub.publish(Empty())
        self.latest_detection = None
        self._enter_state(State.SCAN)

    def _tick_stuck(self, now):
        elapsed = now - self.stuck_phase_start

        if self.stuck_phase == 'reverse':
            # Reverse for ~1.5 s ≈ 0.3 m at approach_speed
            twist = Twist()
            twist.linear.x = -self.approach_speed
            self._publish_cmd(twist)
            if elapsed >= 1.5:
                self.stuck_phase = 'rotate'
                self.stuck_phase_start = now

        elif self.stuck_phase == 'rotate':
            # Rotate 45° (π/4 rad) at scan_angular_speed
            twist = Twist()
            twist.angular.z = self.scan_angular_speed
            self._publish_cmd(twist)
            rotate_duration = (math.pi / 4.0) / self.scan_angular_speed
            if elapsed >= rotate_duration:
                self.get_logger().info(
                    f'STUCK recovery complete — resuming {self.prior_state.value}')
                self._stop()
                self.last_movement_time = time.monotonic()  # reset watchdog clock
                self._enter_state(self.prior_state)

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _enter_state(self, new_state):
        self.get_logger().info(f'FSM: {self.state.value} → {new_state.value}')
        self.state = new_state
        self.state_enter_time = time.monotonic()

    def _enter_stuck(self):
        self.prior_state = self.state
        self._stop()
        self.stuck_phase = 'reverse'
        self.stuck_phase_start = time.monotonic()
        self._enter_state(State.STUCK)

    def _stop(self):
        self._publish_cmd(Twist())  # all-zero Twist stops the robot

    def _publish_cmd(self, twist):
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_nonzero = (
            twist.linear.x != 0.0
            or twist.linear.y != 0.0
            or twist.angular.z != 0.0
        )


def main():
    rclpy.init()
    node = MissionFSM()
    try:
        rclpy.spin(node)
    finally:
        node.csv_file.close()
        rclpy.shutdown()
