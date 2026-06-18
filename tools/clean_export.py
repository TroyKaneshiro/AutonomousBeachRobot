from ultralytics import YOLO

# Load the raw PyTorch weights freshly from disk
model = YOLO("/home/ttkan/AutonomousBeachRobot/ml/models/trash_v1_best.pt")

# Export with explicit geometric simplification parameters
model.export(format="onnx", simplify=True, opset=12)