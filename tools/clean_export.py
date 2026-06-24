from pathlib import Path
from ultralytics import YOLO

_REPO_ROOT = Path(__file__).resolve().parent.parent
model = YOLO(str(_REPO_ROOT / 'ml' / 'models' / 'trash_v1_best.pt'))

# Export with explicit geometric simplification parameters
model.export(format="onnx", simplify=True, opset=12)