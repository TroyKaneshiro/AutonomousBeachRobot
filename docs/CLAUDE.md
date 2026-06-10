# AutonomousBeachRobot — Project Context

## What this is
Autonomous beach litter collection robot. Scans for trash using YOLOv8n,
tracks targets with CSRT visual servoing, drives toward them, and logs
GPS coordinates. V2 adds a servo gripper arm for physical pickup.

## Hardware
- Raspberry Pi 5 (4GB) — ROS 2, inference, mission FSM
- ESP32 DevKit C — micro-ROS, PID motor control at 100Hz
- Cytron MDD10A — dual motor driver
- Pi Camera Module 3 Wide — trash detection + terrain classification
- MPU-6050 IMU — terrain safety backstop
- u-blox NEO-M8N GPS — logs trash coordinates
- ACS712 current sensors x2 — motor stall detection
- 3S LiPo 5000mAh — power

## Software stack
- ROS 2 Humble (dev on WSL2, deploy on Pi 5)
- YOLOv8n fine-tuned on TACO dataset (single class: trash) — ONNX export, CPU inference
- OpenCV CSRT tracker
- micro-ROS on ESP32
- Python 3.10

## ROS 2 topic map
| Topic                    | Type                    | Publisher        | Subscriber(s)              |
|--------------------------|-------------------------|------------------|----------------------------|
| /camera/image_raw        | sensor_msgs/Image       | v4l2_camera_node | trash_detector, terrain_monitor |
| /trash_detections        | std_msgs/String (JSON)  | trash_detector   | mission_fsm, coordinator   |
| /trash_detector/reset    | std_msgs/Empty          | mission_fsm, coordinator | trash_detector       |
| /terrain_events          | std_msgs/String         | terrain_monitor  | mission_fsm, coordinator   |
| /motor_events            | std_msgs/String         | ESP32            | mission_fsm, coordinator   |
| /cmd_vel                 | geometry_msgs/Twist     | mission_fsm      | ESP32                      |
| /wheel/odometry          | nav_msgs/Odometry       | ESP32            | mission_fsm                |
| /imu/data                | sensor_msgs/Imu         | mpu6050_driver   | terrain_monitor            |
| /fix                     | sensor_msgs/NavSatFix   | ublox_gps_node   | mission_fsm                |
| /battery_voltage         | std_msgs/Float32        | ESP32            | mission_fsm, coordinator   |
| /e_stop                  | std_msgs/Bool           | coordinator      | ESP32 firmware             |
| /mission_control/command | std_msgs/String         | operator         | coordinator                |
| /mission_status          | std_msgs/String (JSON)  | coordinator      | operator monitoring        |

## Mission FSM states
CALIBRATE → SCAN → TRACK → PICKUP/FLAG → back to SCAN
+ STUCK and LOW_BATTERY safety states

Transitions:
- SCAN → TRACK: detection conf ≥ 0.45
- TRACK → PICKUP: normalised bbox area ≥ 0.15 (calibrate at actual pickup distance)
- TRACK → SCAN: no detection for 0.5 s
- any → STUCK: no odometry movement for 5 s while cmd_vel_nonzero
- any → LOW_BATTERY: voltage < 10.5 V (3S cutoff)

## Target selection scoring
score = (0.7 × normalised_bbox_area) + (0.3 × confidence)
Closest target preferred; confidence is a tiebreaker.

## Visual servoing
angular_z = -kp * (cx - frame_width/2)   kp=0.002, tune empirically
linear_x  = 0.2 m/s (approach_speed)

## Packages
```
ros2_ws/src/
  perception/           trash_detector.py (YOLO+CSRT hybrid)
                        terrain_monitor.py — NOT YET WRITTEN
  v1_navigator/         mission_fsm.py
  mission_control/      coordinator.py (operator interface, e-stop, session stats)
  robot_bringup/        launch/beach_robot.launch.py
                        config/robot_params.yaml
```

## Key files
- `ml/models/trash_v1_best.onnx` — trained model (mAP50=0.549 on TACO validation set)
- `tools/fake_camera.py` — publishes a static JPEG as 30fps /camera/image_raw for offline testing
- `ros2_ws/src/robot_bringup/config/robot_params.yaml` — all tunable params in one place

## Model path
Default: `/home/ttkan/AutonomousBeachRobot/ml/models/trash_v1_best.onnx`
Override at launch: `ros2 launch robot_bringup beach_robot.launch.py model_path:=<path>`

## Operator workflow
```bash
# Start everything
ros2 launch robot_bringup beach_robot.launch.py

# Arm the robot (coordinator starts with e_stop=True — must send START)
ros2 topic pub --once /mission_control/command std_msgs/msg/String "data: START"

# Monitor status (1 Hz JSON: running, uptime, items_flagged, battery_v, ...)
ros2 topic echo /mission_status

# Emergency stop
ros2 topic pub --once /mission_control/command std_msgs/msg/String "data: STOP"
```

## Current status
- [x] ROS 2 Humble installed (WSL2 dev environment)
- [x] YOLOv8n trained on TACO dataset — `trash_v1_best.onnx` (mAP50=0.549)
- [x] `trash_detector.py` — YOLO+CSRT node, reset mechanism, CPU inference
- [x] `mission_fsm.py` — full FSM with visual servoing, STUCK, LOW_BATTERY, CSV log
- [x] `coordinator.py` — operator interface, e-stop, session stats, /mission_status
- [x] `robot_bringup` — launch file + params YAML, all packages build clean
- [ ] `terrain_monitor.py` — not yet written
- [ ] ESP32 firmware — not yet written
- [ ] Model fine-tuning — deferred (current model detects poorly on beach-specific trash)
- [ ] Hardware — not yet arrived
