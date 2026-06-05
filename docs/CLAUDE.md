cat << 'EOF' > ~/beach-bot/CLAUDE.md
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
- ROS 2 Humble
- YOLOv8n fine-tuned on TACO dataset (single class: trash)
- OpenCV CSRT tracker
- micro-ROS on ESP32
- Python 3.10

## ROS 2 topics
- /camera/image_raw     — camera feed (shared by all vision nodes)
- /trash_detections     — best scoring detection per frame (JSON string)
- /terrain_events       — STOP_CAM or STOP_IMU
- /motor_events         — HIGH_CURRENT_L/R, STALL
- /cmd_vel              — velocity commands to ESP32
- /wheel/odometry       — encoder odometry from ESP32
- /fix                  — GPS NavSatFix
- /battery_voltage      — LiPo voltage float

## Mission FSM states
CALIBRATE → SCAN → TRACK → PICKUP/FLAG → back to SCAN
+ STUCK and LOW_BATTERY safety states

## Target selection scoring
score = (0.7 × normalised_bbox_area) + (0.3 × confidence)
Blacklist targets that fail pickup twice.

## Packages
- perception/       trash_detector.py, terrain_monitor.py
- v1_navigator/     mission_fsm.py
- mission_control/  coordinator
- robot_bringup/    launch files

## Current status
- ROS 2 Humble installed on WSL2
- YOLOv8 installed, GPU verified (RTX 5060)
- Training on TACO dataset in progress
- Hardware not yet arrived
EOF

git add CLAUDE.md
git commit -m "add CLAUDE.md project context"
git push