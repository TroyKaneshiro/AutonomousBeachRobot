#tests model independently without ROS2 nodes being involved
import cv2
from ultralytics import YOLO

# Load your exported ONNX model directly into the Ultralytics engine
model = YOLO("/home/ttkan/AutonomousBeachRobot/ml/models/trash_v1_best.onnx", task='detect')

# Run inference on a static test image file (use one the model hasn't seen)
# Set a very low confidence threshold (0.1) to see if it sees ANY objects at all
results = model("/home/ttkan/AutonomousBeachRobot/tools/test_trash2.jpg", conf=0.1)

# Render and show the output array visually
annotated_frame = results[0].plot()

output_path = "/home/ttkan/AutonomousBeachRobot/tools/result/test_result2.jpg"
cv2.imwrite(output_path, annotated_frame)
print(f"Inference complete! Output saved cleanly to: {output_path}")

