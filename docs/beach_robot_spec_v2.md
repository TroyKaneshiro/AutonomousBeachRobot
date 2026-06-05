  
**Beach Trash Collection Robot**

Project Specification  ·  v2

EE/CS \+ MechE/ECE  ·  Summer Build

*Autonomous outdoor robot that scans for litter, tracks and approaches the highest-priority target, and logs GPS coordinates for collection — with a robotic arm pickup system as the v2 goal.*

# **1\. Project overview**

This robot patrols a beach or open field autonomously, uses computer vision to detect and track litter, and drives toward the highest-priority piece of trash. V1 logs GPS coordinates of detections. V2 adds a robotic arm for physical collection.

## **1.1 Core behaviour loop**

* SCAN — robot rotates 360°, YOLOv8n identifies all visible trash, scores each detection by proximity and confidence

* TRACK — CSRT visual tracker locks onto the best target, robot steers toward it using visual servoing

* APPROACH — bounding box area grows as robot closes in; no explicit distance measurement needed

* FLAG / PICKUP — at threshold bbox size: V1 logs GPS coordinate; V2 actuates gripper arm

* SCAN again — repeat until battery low or operator stop

*Design principle: the robot is perception-driven — every movement decision comes from what the camera sees, not a pre-planned path. This makes behaviour more robust and the project significantly more impressive as a portfolio piece.*

## **1.2 V1 vs V2 scope**

| Feature | V1 — ship by end of summer | V2 — stretch / fall |
| :---- | :---- | :---- |
| Navigation | Reactive — IMU \+ camera terrain boundary | Nav2 \+ EKF \+ GPS positioning |
| Target detection | YOLOv8n scan \+ CSRT tracking | Same, improved dataset |
| Distance sensing | Bounding box area heuristic | Monocular depth or OAK-D |
| Trash collection | GPS flag and log | Servo gripper arm pickup |
| Coverage | Scan-navigate-scan | Lawnmower path planning |
| Localisation | None required | Odometry \+ IMU \+ GPS EKF fusion |

# **2\. Team & responsibility split**

| Domain | EE/CS | MechE/ECE partner |
| :---- | :---- | :---- |
| Chassis & drivetrain | — | CAD design, 3D-printed mounts, assembly |
| Motor control firmware | Serial protocol, ROS bridge | PID loop on ESP32 |
| Power system | Wiring, buck converter, fusing, LiPo safety | Cable routing through chassis |
| Sensors | IMU, GPS, camera wiring and drivers | Encoder wiring, current sensor placement |
| Perception | YOLOv8 training, CSRT node, terrain classifier | Camera mounting, FOV optimisation |
| Navigation FSM | Mission state machine, visual servoing | Odometry calibration, field testing |
| Integration | ROS node graph, launch files, logging | Mechanical fixes, sand-proofing |
| V2 arm (stretch) | PCA9685 servo driver, arm ROS node | Gripper mechanism design and fabrication |

# **3\. System architecture**

## **3.1 Hardware layers**

The system is split across two compute platforms with a clean interface between them.

| Layer | Hardware | Responsibility |
| :---- | :---- | :---- |
| Brain | Raspberry Pi 5 (4GB) | ROS 2, YOLOv8 inference, CSRT tracker, mission FSM, GPS logging |
| Muscles | ESP32 DevKit C | PID motor control at 100Hz, encoder counting, current sensing, micro-ROS bridge |
| Drive | Cytron MDD10A \+ 4× 12V DC gear motors | Locomotion — converts PWM signals to motor power |
| Vision | Pi Camera Module 3 (wide angle) | Single shared feed for trash detection and terrain classification |
| Orientation | MPU-6050 IMU | Safety layer — pitch and vibration spike detection for terrain boundaries and dropoffs |
| Location | u-blox NEO-M8N GPS | Logs coordinates of every detected trash item at 5Hz |
| Current sense | ACS712 30A ×2 | Per-motor current monitoring — detects stall and terrain resistance changes |
| Power | 3S LiPo 5000mAh \+ buck converter 12V→5V | Isolated rails for motors and compute; LiPo alarm on balance lead |
| V2: Arm driver | PCA9685 servo driver board | Controls gripper arm servos from Pi over I2C |

## **3.2 ROS 2 topic map**

| Topic | Message type | Publisher | Subscriber(s) |
| :---- | :---- | :---- | :---- |
| /camera/image\_raw | sensor\_msgs/Image | v4l2\_camera\_node | perception, terrain\_monitor |
| /trash\_detections | std\_msgs/String | trash\_detector | mission\_fsm |
| /terrain\_events | std\_msgs/String | terrain\_monitor | mission\_fsm |
| /motor\_events | std\_msgs/String | ESP32 | mission\_fsm |
| /cmd\_vel | geometry\_msgs/Twist | mission\_fsm / navigator | ESP32 |
| /wheel/odometry | nav\_msgs/Odometry | ESP32 | mission\_fsm (V2: EKF) |
| /imu/data | sensor\_msgs/Imu | mpu6050\_driver | terrain\_monitor |
| /fix | sensor\_msgs/NavSatFix | ublox\_gps\_node | gps\_logger |
| /battery\_voltage | std\_msgs/Float32 | ESP32 | mission\_fsm |

## **3.3 Mission FSM**

All navigation and task behaviour lives in a single finite state machine node. States are mutually exclusive; transitions are event-driven.

| State | Entry condition | Behaviour | Exit condition |
| :---- | :---- | :---- | :---- |
| CALIBRATE | Startup | Hold still, establish IMU baseline variance over 1.5s | Baseline locked → SCAN |
| SCAN | After CALIBRATE, FLAG, or PICKUP | Rotate slowly at 0.3 rad/s, run YOLO every 0.5s, score all detections | Detection above threshold → TRACK |
| TRACK | Detection scored and selected | CSRT locks on target, visual servoing steers robot toward it | Bbox area \> threshold → PICKUP; target lost → SCAN |
| PICKUP / FLAG | Close enough to target | V1: log GPS \+ confidence to CSV. V2: stop, actuate arm | Attempt complete → SCAN |
| STUCK | No movement for 5s despite cmd\_vel | Reverse 0.3m, rotate 45°, resume prior state | Movement detected → resume |
| LOW\_BATTERY | Voltage \< 10.5V (3S cutoff) | Stop all motion, publish alert, await operator | Manual reset only |

# **4\. Perception pipeline**

## **4.1 Trash detection — YOLOv8n**

* Model: YOLOv8 nano (yolov8n.pt), fine-tuned on TACO dataset (Trash Annotations in Context)

* Classes: collapsed to single class 'trash' for v1 — simplifies training, sufficient for detection

* Export format: ONNX, imgsz=640, simplified

* Target inference rate: 8–15 FPS on Pi 5 with active cooling

* Target mAP50: \> 0.40 on validation set

## **4.2 Target selection scoring**

When multiple detections exist, a weighted score selects the best target per frame:

score \= (0.7 × normalised\_bbox\_area) \+ (0.3 × confidence)

* Proximity weight 0.7 — closest target preferred; bbox area is proxy for distance

* Confidence weight 0.3 — avoids wasting approach on false positives

* Blacklist — targets that fail pickup twice are excluded from scoring until next full scan

* Re-score every 2s during TRACK — relative distances change as robot moves

## **4.3 CSRT visual tracking**

After YOLO selects a target, a CSRT tracker (OpenCV built-in) takes over for per-frame localisation. YOLO runs at 8–10 FPS; CSRT runs at 30+ FPS on the same detection region without re-running the neural net.

* Init: tracker.init(frame, bbox) called once after YOLO detection

* Update: ok, bbox \= tracker.update(frame) every frame

* Lost: if ok is False → fall back to SCAN state to re-acquire

* Proximity trigger: bbox\_area / frame\_area \> PICKUP\_THRESHOLD (calibrate at your actual pickup distance)

## **4.4 Terrain classification**

A lightweight secondary perception node subscribes to the same camera feed and publishes STOP\_CAM events when the terrain ahead changes. The camera is a secondary signal; the IMU is the safety backstop.

* Method: HSV colour thresholding on bottom third of frame

* Trigger: sand\_ratio \< 0.5 in the ROI

* Rate: throttled to 10 FPS to leave headroom for YOLO

* Tuning: HSV range must be calibrated at the actual test location in the actual lighting conditions

*Camera terrain classification catches visual boundaries (sand/grass, sand/pavement). IMU catches physical events (dropoffs, vibration spikes, soft sand sinking). They fail on different inputs — both are needed.*

# **5\. Navigation & motor control**

## **5.1 Visual servoing**

The robot steers toward a tracked target using proportional control on the horizontal offset of the bounding box centre from the frame centre. No distance measurement, no coordinate frame transformation needed.

error \= bbox\_centre\_x − frame\_width/2angular\_z \= −Kp × error   (Kp ≈ 0.002, tune empirically)linear\_x \= APPROACH\_SPEED (0.2 m/s)

## **5.2 Motor control (ESP32)**

* PID loop runs at 100Hz on ESP32 Core 1 — real-time, no OS jitter

* Core 0 handles micro-ROS WiFi/serial comms — physically isolated from PID

* Per-motor PID — each motor has independent Kp/Ki/Kd; motors are never identical

* ACS712 current sensors publish HIGH\_CURRENT events when average exceeds 2.5A per motor

* Encoder odometry published to /wheel/odometry at 50Hz

## **5.3 Terrain safety — dual-layer**

| Layer | Sensor | Catches | Misses |
| :---- | :---- | :---- | :---- |
| Primary | Camera HSV classifier | Visual boundaries: sand→grass, sand→pavement | Dropoffs, soft sand, glare, night |
| Safety backstop | MPU-6050 IMU | Dropoffs, vibration spikes, pitch \> 8.5°, soft sand sinking | Gradual colour-only boundaries |
| Safety backstop | ACS712 current sensors | Motor stall, high resistance terrain, wheel sinking | Airborne obstacles, visual changes |

# **6\. Hardware bill of materials**

## **6.1 Compute & firmware**

| Component | Purpose | Cost | Notes |
| :---- | :---- | :---- | :---- |
| Raspberry Pi 5 (4GB) | Main compute, ROS 2, inference | $60 | Order immediately — can be backordered |
| Pi 5 active cooler | Thermal management under inference load | $5 | Required — YOLOv8 will throttle without it |
| ESP32 DevKit C ×2 | Real-time motor PID, micro-ROS | $16 | Buy two — one will get bricked in dev |
| MicroSD 64GB A2-rated | Pi OS \+ ROS workspace \+ rosbag logs | $10 | A2 speed class required for ROS I/O |

## **6.2 Chassis & drivetrain**

| Component | Purpose | Cost | Notes |
| :---- | :---- | :---- | :---- |
| 4WD aluminium chassis kit | Frame, mounts, hubs | $45 | Get one that includes motors \+ encoders |
| 12V DC gear motors ×4 with encoders | Locomotion \+ odometry feedback | Included | Verify \> 200 CPR encoder resolution |
| Wide rubber wheels ×4 | Traction on sand and grass | Included | Wide profile preferred; avoid narrow plastic |
| M3/M4 hardware assortment | Electronics mounting | $8 | Standoffs, screws, nuts — use everything |

## **6.3 Motor driver**

| Component | Purpose | Cost | Notes |
| :---- | :---- | :---- | :---- |
| Cytron MDD10A (recommended) | Dual motor driver, 10A/ch, 3.3V safe | $25 | MOSFET-based, no heatsink, ESP32 native logic |
| L298N (budget alternative) | Dual motor driver, 2A/ch | $4 | Add heatsink \+ logic level shifter; buy 2 as spares |
| Logic level shifter (if L298N) | 3.3V→5V for ESP32→L298N | $1 | Only needed with L298N |

## **6.4 Sensors**

| Component | Purpose | Cost | Notes |
| :---- | :---- | :---- | :---- |
| Pi Camera Module 3 Wide | Trash detection \+ terrain classifier | $25 | Wide angle; order immediately with Pi |
| MPU-6050 IMU | Terrain safety: pitch, vibration spike | $5 | Mount on 1mm foam tape away from motors |
| ACS712 30A current sensors ×2 | Per-motor stall and terrain detection | $6 | Inline with each motor lead |
| u-blox NEO-M8N GPS | Logs trash detection coordinates | $30 | Configure at 5Hz via u-center before deployment |
| Camera mount / articulating arm | Positions camera \~30° down at 0.5m | $8 | Can 3D print a custom mount |

## **6.5 Power system**

| Component | Purpose | Cost | Notes |
| :---- | :---- | :---- | :---- |
| 3S LiPo 5000mAh (Tattu / Gens Ace) | Main power source | $35 | Quality brand only; check C-rating ≥ 20C |
| LiPo battery alarm | Per-cell undervoltage alert at 3.5V | $3 | Non-negotiable safety item |
| LiPo balance charger (ISDT Q6) | Safe balanced charging | $25 | Never charge without a balance charger |
| LiPo safe bag | Fire containment during charging | $8 | Always charge in this bag |
| Buck converter 12V→5V 5A | Regulated 5V for Pi and sensors | $8 | Pololu D24V50F5 is reliable; verify 5.1V output |
| XT60 connector pairs ×5 | Battery main lead | $6 | Standard 60A+ rated; solder properly |
| 20A blade fuse \+ holder | Main line protection | $4 | Inline on battery positive; saves the chassis |

## **6.6 Wiring & consumables**

| Component | Purpose | Cost |
| :---- | :---- | :---- |
| Silicone wire 18AWG red/black 5m ea | Motor and power runs | $10 |
| Jumper wire kit (M-M, M-F, F-F) | Sensor connections and prototyping | $7 |
| JST-XH 2.54mm connector kit | Encoder and sensor wire management | $8 |
| Heat shrink tubing assortment | Insulation on all solder joints | $5 |
| 1mm foam mounting tape | IMU vibration damping | $4 |
| Velcro straps | Battery retention and cable routing | $5 |
| Conformal coat spray | Board protection against sand and moisture | $12 |

## **6.7 V2 additions (arm)**

| Component | Purpose | Cost | Notes |
| :---- | :---- | :---- | :---- |
| PCA9685 servo driver board | Controls arm servos from Pi over I2C | $8 | 16-channel; only needs 3–4 for a simple arm |
| Servo gripper arm kit (2–3 DOF) | Physical trash pickup | $40–60 | Your partner's mechanical design domain |
| High-torque servos ×3 (MG996R) | Arm joints and gripper | Included or \~$20 | Sand-rated or enclose them |

## **6.8 Budget summary**

| Build variant | Estimated total |
| :---- | :---- |
| V1 — Cytron MDD10A (recommended) | \~$300–340 |
| V1 — L298N (budget) | \~$275–315 |
| V2 — add arm components | \+ \~$70–90 |

# **7\. Software stack**

## **7.1 Environment**

| Component | Version / detail |
| :---- | :---- |
| OS | Raspberry Pi OS 64-bit (Bookworm) |
| ROS 2 | Humble Hawksbill (LTS) |
| Python | 3.10+ |
| ESP32 framework | micro-ROS \+ Arduino via PlatformIO |
| ML inference | Ultralytics YOLOv8, ONNX runtime |
| Vision | OpenCV 4.x (CSRT tracker built-in) |
| Training | Google Colab T4 GPU, YOLOv8n.pt pretrained |
| Dataset | TACO (Trash Annotations in Context), collapsed to 1 class |

## **7.2 Repository structure**

* firmware/esp32/ — PlatformIO project: PID, encoder, current sense, micro-ROS

* ros2\_ws/src/v1\_navigator/ — mission FSM, visual servoing, STUCK/LOW\_BATTERY states

* ros2\_ws/src/perception/ — trash\_detector node (YOLOv8 \+ CSRT) and terrain\_monitor node

* ros2\_ws/src/mission\_control/ — high-level state coordinator

* ros2\_ws/src/robot\_bringup/ — launch files, YAML configs

* ml/ — training script, trash.yaml, exported models (gitignored if \> 100MB)

* docs/ — wiring diagram PDF, serial protocol spec, field tuning notes

## **7.3 Key design decisions**

* One camera, two consumers — camera node publishes /camera/image\_raw once; both perception nodes subscribe and throttle themselves independently. Never open the camera device from two nodes.

* ESP32 owns real-time, Pi owns intelligence — PID loop on ESP32 Core 1 at 100Hz with no OS interruption. Pi decides what to do; ESP32 executes it precisely.

* CSRT over YOLO for tracking — YOLO detects once, CSRT tracks at 30+ FPS. Hybrid approach gives accuracy without burning all compute budget on inference.

* Bounding box area as distance proxy — elegant, hardware-free, calibrated once at actual pickup distance. No depth camera required for V1.

* IMU as safety backstop only — camera terrain classification is primary; IMU is the last-resort emergency brake for physical events the camera cannot see.

# **8\. 10-week build timeline**

## **EE/CS track (your responsibilities)**

| Week | Phase | Your deliverables |
| :---- | :---- | :---- |
| 1 | Setup | Pi OS \+ ROS 2 installed, ESP32 \+ micro-ROS ping-pong, repo scaffold, protocol doc, power bench test, parts ordered |
| 2 | Drive | ESP32 firmware: /cmd\_vel → PID → motors; /wheel/odometry from encoders; ACS712 current sensing wired and publishing |
| 3 | Sensors | IMU terrain detection node live; camera feed publishing; terrain\_monitor combining IMU \+ camera signals |
| 4 | Navigate | Mission FSM: CALIBRATE → SCAN → TRACK → FLAG; visual servoing proportional controller tuned on bench |
| 5 | ML | YOLOv8n trained on TACO, ONNX exported, benchmarked on Pi at ≥ 8 FPS, ROS perception node publishing /trash\_detections |
| 6 | Integrate | CSRT tracker integrated; full pipeline: camera → detect → track → approach → flag; GPS wired and logging |
| 7 | Test | First outdoor test; re-tune IMU baseline, HSV range, current thresholds at actual beach/field location |
| 8 | Harden | STUCK \+ LOW\_BATTERY states; rosbag logging every run; edge case handling (glare, wet sand, GPS loss) |
| 9 | Polish | Post-run visualisation: plot GPS detection pins on map; 2× clean outdoor runs recorded; repo cleaned up |
| 10 | Demo | Final demo video, README writeup, ML pipeline documented, tag v1.0 |

## **Milestones**

| Week | Milestone |
| :---- | :---- |
| End week 2 | Robot drives and stops on /cmd\_vel command |
| End week 4 | Robot scans, detects trash in camera feed, visual servoing steers toward it on a table |
| End week 6 | Full pipeline running: scan → track → approach → GPS log |
| End week 8 | 10-minute autonomous outdoor run without intervention |
| End week 10 | Portfolio-ready project with demo video and write-up |

# **9\. Engineering notes & lessons**

## **9.1 Sand-specific concerns**

* Conformal coat all PCBs before first outdoor deployment — sand \+ salt air \+ moisture is corrosive

* Enclose Pi and ESP32 in 3D-printed PETG/ASA boxes with rubber-grommeted cable holes

* Mesh or foam over any ventilation holes — sand gets into everything

* Wide-profile rubber wheels only — narrow wheels sink; plastic wheels slip

* Motors will draw more current on sand than in indoor tests — tune current threshold accordingly

* GPS antenna needs clear sky view — test fix quality before full runs (should read position\_covariance\[0\] \< 5.0)

## **9.2 LiPo safety — non-negotiable**

*Always charge in a LiPo bag. Always use a balance charger. Never leave charging unattended. Never discharge below 3.5V/cell. Store at 3.8V/cell if not using for more than a week.*

## **9.3 Power system rules**

* 20A blade fuse inline on main battery positive — $0.50 that protects the chassis

* Motors and Pi on separate regulated rails — motor noise on the Pi supply causes random crashes

* Buck converter must handle 5A+ — Pi 5 draws up to 5A at peak

* Bench-test full power system before connecting Pi: verify 5.0–5.2V output under load

## **9.4 Key principles**

* Ship v1 first — a working simple system beats a broken complex one every time

* Record everything — rosbag every outdoor session; you debug from logs, not memory

* Tune at the real location — indoor calibration is always wrong outdoors

* Weekly sync with partner — explicit agenda, written deliverables; assumptions cause failures

* Document as you go — commits with meaning, photos at each milestone, video at each working demo

