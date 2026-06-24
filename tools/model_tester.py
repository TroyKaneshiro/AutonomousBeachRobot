#tests model independently without ROS2 nodes being involved
import cv2
from pathlib import Path
from ultralytics import YOLO

_REPO_ROOT = Path(__file__).resolve().parent.parent
_TOOLS_DIR = _REPO_ROOT / 'tools'

model = YOLO(str(_REPO_ROOT / 'ml' / 'models' / 'trash_v3_best.onnx'), task='detect')

results = model(str(_TOOLS_DIR / 'test_trash2.jpg'), conf=0.45)

annotated_frame = results[0].plot()

output_path = str(_TOOLS_DIR / 'result' / 'test_result2.jpg')
cv2.imwrite(output_path, annotated_frame)
print(f"Inference complete: Output saved to: {output_path}")

