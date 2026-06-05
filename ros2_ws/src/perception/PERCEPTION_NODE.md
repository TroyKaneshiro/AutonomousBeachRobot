# Perception Node — Implementation Brief
> Context file for Claude Code. Write this node from scratch based on the spec below.

---

## What this node does

Subscribes to the robot's camera feed, runs YOLOv8n trash detection on each frame, tracks the best target using OpenCV CSRT, and publishes the result for the mission FSM to act on.

Two classes live in this file:
- `TrashDetector` — the ROS 2 node
- Internal CSRT tracker state managed within the node

---

## File location

```
AutonomousBeachRobot/
└── ros2_ws/
    └── src/
        └── perception/
            └── perception/
                └── trash_detector.py   ← write this file
```

---

## ROS 2 interface

### Subscribes to
| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | 30Hz | Throttle internally to 10fps for YOLO |

### Publishes to
| Topic | Type | Notes |
|---|---|---|
| `/trash_detections` | `std_msgs/String` | JSON string, only published when detection exists |

### Published message format
```json
{
  "cx": 320.5,
  "cy": 410.2,
  "area": 14500,
  "conf": 0.82,
  "tracking": true,
  "bbox": [280, 390, 361, 431]
}
```
- `cx`, `cy` — bounding box centre in pixels
- `area` — bounding box area in pixels (proxy for distance — bigger = closer)
- `conf` — YOLO confidence score 0.0–1.0
- `tracking` — true if CSRT tracker is active, false if fresh YOLO detection
- `bbox` — [x1, y1, x2, y2] pixel coordinates

---

## Detection pipeline

```
Camera frame arrives
        │
        ├── [every frame]  CSRT tracker update (30fps)
        │         │
        │         ├── ok=True  → publish tracker result
        │         └── ok=False → tracker lost, fall back to YOLO
        │
        └── [every 0.1s]   YOLO inference (10fps)
                  │
                  ├── detections found → score them → init CSRT on best
                  └── no detections    → publish nothing
```

Key design: YOLO runs at 10fps (expensive), CSRT runs every frame (cheap). When CSRT loses the target, YOLO re-acquires it.

---

## Target scoring

When YOLO finds multiple detections, pick the best one using:

```python
score = (0.7 * normalised_area) + (0.3 * confidence)

# normalised_area = bbox_area / (frame_width * frame_height)
# confidence is already 0.0–1.0
```

Highest score wins. This prioritises the closest trash (biggest bbox) with confidence as a tiebreaker.

---

## CSRT tracker behaviour

```python
# Initialise tracker after a fresh YOLO detection
tracker = cv2.TrackerCSRT_create()
tracker.init(frame, (x, y, w, h))   # bbox as x,y,w,h not x1,y1,x2,y2

# Update every frame
ok, bbox = tracker.update(frame)
# ok=False means lost — set tracker to None, wait for next YOLO cycle
```

Re-initialise the tracker whenever YOLO finds a new higher-scoring target.

---

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `model_path` | `'models/trash_v1_best.onnx'` | Path to ONNX model file |
| `confidence_threshold` | `0.45` | Ignore YOLO detections below this |
| `yolo_interval` | `0.1` | Seconds between YOLO inference calls |
| `frame_width` | `640` | Expected camera frame width |
| `frame_height` | `480` | Expected camera frame height |

Declare these as ROS 2 parameters so they can be overridden from the launch file without editing code:
```python
self.declare_parameter('confidence_threshold', 0.45)
conf = self.get_parameter('confidence_threshold').value
```

---

## Key imports

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import json
import time
```

---

## Node skeleton

```python
class TrashDetector(Node):
    def __init__(self):
        super().__init__('trash_detector')

        # Parameters
        # Model loading
        # Publisher + Subscriber
        # CSRT tracker state
        # Timing for YOLO throttle

    def image_callback(self, msg):
        # Convert ROS image to OpenCV frame using CvBridge
        # Run CSRT update every frame
        # Run YOLO every self.yolo_interval seconds
        # Score detections, pick best
        # Re-init CSRT if new best target found
        # Publish result

    def run_yolo(self, frame):
        # Run YOLO inference
        # Filter by confidence threshold
        # Score each detection
        # Return best detection or None

    def score_detection(self, bbox, confidence):
        # score = 0.7 * normalised_area + 0.3 * confidence
        # Return float score

    def publish_detection(self, bbox, confidence, tracking):
        # Build JSON dict
        # Publish as std_msgs/String

def main():
    rclpy.init()
    node = TrashDetector()
    rclpy.spin(node)
    rclpy.shutdown()
```

---

## setup.py entry point

Make sure this line exists in the `console_scripts` section of
`ros2_ws/src/perception/setup.py`:

```python
'trash_detector = perception.trash_detector:main',
```

Without this, `ros2 run perception trash_detector` won't find the node.

---

## package.xml dependencies

Add these to `ros2_ws/src/perception/package.xml` inside `<package>`:

```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
<depend>cv_bridge</depend>
```

---

## Testing without a camera

You can test this node before the Pi or camera arrives by publishing a fake image topic from a static file:

```bash
# Terminal 1 — run the node
ros2 run perception trash_detector

# Terminal 2 — feed it a static image as a ROS topic
# Install: pip3 install ros2-numpy
python3 -c "
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rclpy.init()
node = Node('fake_camera')
pub = node.create_publisher(Image, '/camera/image_raw', 10)
bridge = CvBridge()
img = cv2.imread('path/to/trash_image.jpg')
img = cv2.resize(img, (640, 480))
msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')

timer = node.create_timer(0.033, lambda: pub.publish(msg))
rclpy.spin(node)
"

# Terminal 3 — watch detections
ros2 topic echo /trash_detections
```

Use any JPEG of trash — a photo from your phone, a Google image, anything.

---

## Common mistakes to avoid

- **Don't open the camera device directly** (`cv2.VideoCapture`) — subscribe to `/camera/image_raw` instead. Other nodes share this feed.
- **Don't forget CvBridge** — ROS image messages and OpenCV frames are different formats. Always convert with `bridge.imgmsg_to_cv2(msg, 'bgr8')`.
- **CSRT bbox format** — OpenCV trackers use `(x, y, w, h)` not `(x1, y1, x2, y2)`. Convert before passing to `tracker.init()`.
- **Thread safety** — `rclpy.spin()` is single-threaded by default so callbacks won't interrupt each other. No locks needed for v1.
- **Model path** — use an absolute path or a path relative to where you launch the node. Relative paths break depending on where you run `ros2 run` from.

---

## Related nodes (for context)

This node feeds into:
- `mission_fsm.py` (v1_navigator package) — subscribes to `/trash_detections` and decides robot behaviour
- `terrain_monitor.py` (perception package) — sibling node, shares `/camera/image_raw` feed

---

## What success looks like

When working correctly:
```bash
ros2 topic echo /trash_detections
# Should print JSON like:
# data: '{"cx": 318.2, "cy": 405.1, "area": 12400, "conf": 0.76, "tracking": true, "bbox": [...]}'
```

And topic rate should be ~10Hz when trash is visible, silent when frame is clear.
