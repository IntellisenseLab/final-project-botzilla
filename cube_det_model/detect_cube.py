
# # Install minimal CPU PyTorch, torchvision
# pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# # Install ultralytics for YOLOv8
# pip install ultralytics

# # Install OpenCV for webcam/video
# pip install opencv-python

from ultralytics import YOLO
import cv2

# Load your trained model
model = YOLO("weights/best.pt")

# Open webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.2)  # adjust confidence threshold
    annotated_frame = results[0].plot()  # draw boxes

    cv2.imshow("Cube Detection", annotated_frame)
    if cv2.waitKey(1) == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()