#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def video_publisher():
    rospy.init_node('video_publisher', anonymous=True)
    pub = rospy.Publisher('video_frames', Image, queue_size=1)

    # Create a CvBridge object
    bridge = CvBridge()

    cap = cv2.VideoCapture("/home/yitao/ws/src/test_pkg/scripts/car_-_2165 (540p).mp4")
    
    rate = rospy.Rate(30)  # 30 fps, adjust as needed
    
    i=0
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        i = i+1
        print(i)
        if not ret:
            print("break")
            break

        # Convert the image to a ROS image message
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        pub.publish(image_message)

        rate.sleep()
        

        

    cap.release()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
