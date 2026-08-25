# Autonomous Beach Robot

Perception-driven outdoor mobile robot that autonomously detects, tracks, and approaches litter using computer vision and a reactive finite state machine.

Built from scratch with ROS 2 Jazzy, micro-ROS, and custom motor control firmware split across a Raspberry Pi 5 (intelligence) and ESP32 (real-time motor control).

**Current status:** Visual servoing and full FSM state transitions working on hardware. Chassis integration and physical collection mechanism in progress.  
**V2 goal:** Physical trash collection via scoop mechanism, more robust chasis in progress.

---

## System Architecture

### Compute split

| Platform | Role |
|---|---|
| Raspberry Pi 5 (4GB) | ROS 2 Jazzy, YOLOv8n inference, CSRT tracking, mission FSM, GPS logging |
| ESP32 (micro-ROS) | PID motor control at 100Hz on Core 1; encoder odometry on Core 0 |

The Pi decides what to do. The ESP32 executes it with no OS jitter.

### ROS 2 node graph

```
/camera/image_raw ──┬──▶ trash_detector ──▶ /trash_detections ──▶ mission_fsm ──▶ /cmd_vel ──▶ ESP32
                    └──▶ terrain_monitor ──▶ /terrain_events   ──▶ mission_fsm
/wheel/odometry ────────────────────────────────────────────────────────────────▶ mission_fsm
/imu/data ──────────────────────────────────────────────────────────────────────▶ terrain_monitor
/fix (GPS) ──────────────────────────────────────────────────────────────────────▶ mission_fsm
                                                          coordinator ───────────▶ /e_stop ──▶ ESP32
```

---

## Perception Pipeline

**YOLO + CSRT hybrid** — balances detection accuracy with compute budget on Pi 5:

- YOLOv8n runs at ~10 FPS for detection and target selection
- CSRT correlation filter tracks at 30+ FPS between YOLO cycles
- When CSRT loses the target, YOLO re-acquires on the next cycle

**Target scoring** when multiple detections exist:

```python
score = (0.7 * normalised_bbox_area) + (0.3 * confidence)
```

Proximity-weighted: closest target preferred, confidence as tiebreaker.

**Model:** YOLOv8n fine-tuned on [TACO dataset](http://tacodataset.org/) (Trash Annotations in Context), single class `trash`. ONNX export for CPU inference. Current validation **mAP50: 0.549**.

---

## Navigation

**Visual servoing** — no depth sensor, no coordinate transforms:

```python
pixel_error = bbox_centre_x - frame_width / 2
angular_z   = -Kp * pixel_error    # Kp ≈ 0.002
linear_x    = 0.2                  # m/s approach speed
```

Distance inferred from bounding box area growth. Calibrate `pickup_area_threshold` once at physical pickup distance.

**Terrain safety — three independent layers:**

| Layer | Sensor | Detects | Blind spots |
|---|---|---|---|
| Primary | Camera HSV (bottom ⅓ of frame) | Sand→grass, sand→pavement boundaries | Dropoffs, night, glare |
| Backstop | MPU-6050 IMU (pitch + variance) | Dropoffs, vibration spikes, soft sand | Gradual colour-only transitions |
| Backstop | ACS712 current sensors | Motor stall, high-resistance terrain | Airborne obstacles |

---

## Mission FSM

| State | Entry | Behaviour | Exit |
|---|---|---|---|
| CALIBRATE | Startup | Hold still 1.5s; IMU baseline lock | Baseline set → SCAN |
| SCAN | Post-calibrate / post-flag | Rotate 0.3 rad/s; YOLO every 0.5s | Detection conf ≥ 0.45 → TRACK |
| TRACK | Detection acquired | CSRT tracking + visual servoing forward | bbox area ≥ threshold → PICKUP; timeout → SCAN |
| PICKUP | Close enough | V1: log GPS + confidence to CSV, reset tracker. V2: actuate collection mechanism | Complete → SCAN |
| STUCK | No odometry movement for 5s despite cmd_vel | Reverse 0.3m, rotate 45°, resume prior state | Movement detected → prior state |
| LOW_BATTERY | Voltage < 10.5V (3S cutoff) | Stop all motion | Manual operator reset |

---

## ESP32 Firmware

The ESP32 runs custom firmware with:

- **PID motor control at 100Hz** on Core 1 — differential drive kinematics, closed-loop velocity control from encoder feedback
- **micro-ROS bridge** on Core 0 — subscribes to `/cmd_vel` and `/e_stop`, publishes `/wheel/odometry` and `/motor_events`
- **Safety watchdog** — automatically stops motors if no `/cmd_vel` received for 1 second
- **Quadrature encoder odometry** — left and right wheel position and velocity

### Pin assignments

| Signal | GPIO |
|---|---|
| Left Motor PWM | 25 |
| Left Motor DIR | 26 |
| Right Motor PWM | 27 |
| Right Motor DIR | 14 |
| Left Encoder A | 32 |
| Left Encoder B | 33 |
| Right Encoder A | 23 |
| Right Encoder B | 22 |

---

## Hardware Stack

| Component | Role |
|---|---|
| Raspberry Pi 5 (4GB) | Main compute |
| ESP32 DevKit C | Real-time motor control, micro-ROS |
| Cytron MDD10A | Dual motor driver (10A/ch, 3.3V logic native) |
| 4× 12V DC gear motors w/ encoders (JGA25-370) | Locomotion + odometry |
| HD USB Webcam | Trash detection + terrain classification |
| MPU-6050 IMU | Terrain safety |
| u-blox NEO-M8N GPS | Detection coordinate logging |
| ACS712 30A ×2 | Per-motor current sensing |
| 3S LiPo 5000mAh | Main power |

---

## Repository Structure

```
ros2_ws/src/
├── perception/           # trash_detector (YOLO+CSRT) and terrain_monitor nodes
├── v1_navigator/         # mission_fsm — 6-state FSM, visual servoing, GPS CSV logging
├── mission_control/      # coordinator — operator interface, e-stop, 1Hz telemetry
└── robot_bringup/        # launch files and robot_params.yaml

esp32/
└── src/main.cpp          # ESP32 firmware — PID motor control + micro-ROS

ml/
├── convert.py            # TACO COCO→YOLO format converter (80/20 train/val split)
├── trash.yaml            # YOLOv8 dataset config
└── models/               # Trained ONNX weights (Git LFS)

tools/
├── fake_camera.py        # Publishes static image as /camera/image_raw at 30fps
├── fake_hardware.py      # Stubs /wheel/odometry, /imu/data, /fix, /battery_voltage
├── stream_camera.py      # MJPEG HTTP stream for viewing camera feed over network
└── model_tester.py       # Standalone YOLO inference check without ROS

docs/
└── beach_robot_spec_v2.md  # Full system specification with V2 plans
```

---

## Hardware-less Simulation

The full ROS 2 pipeline runs without physical hardware:

```bash
# Terminal 1 — launch in sim mode
ros2 launch robot_bringup sim.launch.py image_path:=/path/to/trash.jpg

# Terminal 2 — arm the robot
ros2 topic pub --once /mission_control/command std_msgs/msg/String "data: START"

# Monitor state
ros2 topic echo /mission_status

# Emergency stop
ros2 topic pub --once /mission_control/command std_msgs/msg/String "data: STOP"
```

---

## Live Camera Stream

To view the camera feed over the network during field testing:

```bash
python3 tools/stream_camera.py
```

Open `http://beachrobot.local:8080` in a browser. Displays live feed with detection bounding boxes overlaid.

---

## V1 vs V2

| Feature | V1 — current | V2 — planned |
|---|---|---|
| Navigation | Reactive visual servoing | Nav2 + EKF + GPS area coverage |
| Detection | YOLOv8n + CSRT | Same, improved dataset |
| Distance | Bounding box area heuristic | Monocular depth or OAK-D stereo |
| Collection | GPS coordinate logging | Servo gripper arm or scoop mechanism |
| Coverage | Scan-navigate-scan | Lawnmower path planning |

---

## Known Limitations

- CSRT tracker loses target under fast motion or significant viewpoint change — re-acquired by YOLO on next cycle
- Confidence threshold (0.45) tuned for outdoor daylight — performance degrades in low light
- Single encoder odometry on current breadboard build — both encoders to be added on perfboard
- No ROS timestamp sync on ESP32 in current build — STUCK watchdog uses position delta rather than message age
