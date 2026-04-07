from ultralytics import YOLO

# Load your existing trained model
model = YOLO("yolov8s.pt")

# Continue training with new dataset
model.train(
    data="data.yaml",
    epochs=50,
    imgsz=640,
    batch=16
)