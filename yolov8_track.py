import ultralytics
from ultralytics import YOLO
import cv2

cap = cv2.VideoCapture("python_draft/car_-_2165 (540p).mp4")

model = YOLO('yolov8n.pt')  # load an official detection model

i=0
while True:
    ret, frame = cap.read()
    
    #print(i)
    if not ret:
        print("b")
        break
    results = model.track(frame, persist=True)
    boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
    ids = results[0].boxes.id.cpu().numpy().astype(int)
    for box, id in zip(boxes, ids):
        if id == 3:
            print("#######################################")
            print(frame.shape)
            print(box[0], box[1], box[2], box[3])
            cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"Id {id}",
                (box[0], box[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 255),
                2,
            )
        else:
            cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"Id {id}",
                (box[0], box[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2,
            )
    
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
