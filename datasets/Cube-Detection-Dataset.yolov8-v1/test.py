from ultralytics import YOLO
import cv2

# Load your trained model
model = YOLO("yolov8s.pt")  # adjust path if needed

# Open webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Cannot access webcam")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO detection
    results = model(frame, conf=0.25)

    # Draw detections
    annotated_frame = results[0].plot()

    # Show output
    cv2.imshow("Cube Detection", annotated_frame)

    # Press ESC to exit
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()