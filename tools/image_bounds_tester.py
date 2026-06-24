import cv2
from pathlib import Path

#Ensures that translation from TACO to YOLO dataset is correct visually
# Adjust these paths to point to a test image and its translated text file
_REPO_ROOT = Path(__file__).resolve().parent.parent
IMAGE_PATH = str(_REPO_ROOT / 'ml' / 'TACO' / 'images' / 'train' / '000090.jpg')
LABEL_PATH = str(_REPO_ROOT / 'ml' / 'TACO' / 'labels' / 'train' / '000090.txt')

img = cv2.imread(IMAGE_PATH)
h, w, _ = img.shape

with open(LABEL_PATH, 'r') as f:
    for line in f.readlines():
        parts = line.strip().split()
        class_id = parts[0]
        # Unpack normalized YOLO values
        x_center, y_center, bbox_w, bbox_h = map(float, parts[1:])
        
        # Convert back to raw image pixels
        xmin = int((x_center - bbox_w / 2) * w)
        ymin = int((y_center - bbox_h / 2) * h)
        xmax = int((x_center + bbox_w / 2) * w)
        ymax = int((y_center + bbox_h / 2) * h)
        
        # Draw the box
        cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 3)
        cv2.putText(img, f"Class {class_id}", (xmin, ymin - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

cv2.imshow("Translation Box Check", img)
cv2.waitKey(0)
cv2.destroyAllWindows()