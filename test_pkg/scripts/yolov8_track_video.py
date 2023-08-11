import ultralytics
from ultralytics import YOLO
import cv2
import numpy as np
import torch

#car_-_2165 (540p).mp4
#apple_-_99975 (540p).mp4
#walking_-_3171 (540p).mp4
cap = cv2.VideoCapture("/home/yitao/ws/src/test_pkg/scripts/walking_-_3171 (540p).mp4")

model = YOLO('yolov8n.pt')  # load an official detection model

i=0
while True:
    ret, frame = cap.read()
    #print(i)
    #i=i+1

    if not ret:
        print("b")
        break
    results = model.track(frame, persist=True)

    if True: #results[0].boxes.cls.shape!=torch.Size([0]):

        boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
        if results[0].boxes.id is None:
            ids = results[0].boxes.cls.cpu().numpy().astype(int)
        else:
            ids = results[0].boxes.id.cpu().numpy().astype(int)

    
        print("cls: ", results[0].boxes.cls, results[0].boxes.cls.shape==torch.Size([0])) #.numpy().astype(int)
        print(results[0].boxes.id)
        print(ids)

        for box, id in zip(boxes, ids):
            if id == 0:
                print("#######################################")
                print(frame.shape)
                print(box[0], box[1], box[2], box[3])
                cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
                cv2.putText(
                    frame,
                    f"Id {id}",
                    (box[0], box[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2,
                )
            else:
                cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
                cv2.putText(
                    frame,
                    f"Id {id}",
                    (box[0], box[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2,
                )
    
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break