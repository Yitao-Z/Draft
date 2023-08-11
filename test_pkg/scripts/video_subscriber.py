#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import ultralytics
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
print("model loaded")

def yolo_callback(data):
    bridge = CvBridge()
    

    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    results = model.track(cv_image, persist=True)
    boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
    if results[0].boxes.id is None:
        ids = results[0].boxes.cls.cpu().numpy().astype(int)
    else:
        ids = results[0].boxes.id.cpu().numpy().astype(int)
    #print(ids)
    #print(results[0].boxes.cls.cpu().numpy().astype(int))

    for box, id in zip(boxes, ids):
        if id == 1:
            print("#######################################")
            print(cv_image.shape)
            print(box[0], box[1], box[2], box[3])
            cv2.rectangle(cv_image, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
            cv2.putText(
                cv_image,
                f"Id {id}",
                (box[0], box[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 255),
                2,
            )
        else:
            cv2.rectangle(cv_image, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
            cv2.putText(
                cv_image,
                f"Id {id}",
                (box[0], box[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2,
            )
    
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def video_subscriber():
    rospy.init_node('video_subscriber', anonymous=True)
    # /camera/color/image_raw
    # video_frames
    rospy.Subscriber("/camera/color/image_raw", Image, yolo_callback)
    rospy.spin()

if __name__ == '__main__':
    video_subscriber()
