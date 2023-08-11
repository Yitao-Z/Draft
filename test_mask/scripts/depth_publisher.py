#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

def read_z16_file(file_path, width, height):
    with open(file_path, 'rb') as f:
        arr = np.frombuffer(f.read(), dtype=np.uint16).reshape(height, width)

        # Optional: Scale the data if necessary (e.g., to convert to meters)
        arr = arr.astype(np.float32) / 1000.0  # replace 1000 with appropriate scaling factor

    return arr

def main():
    rospy.init_node('depth_image_publisher')

    # Image dimensions
    width = 424  # replace with your value
    height = 240  # replace with your value

    img_path = "/home/yitao/ws/src/test_mask/src/1depth_Depth.raw"
    image_data = read_z16_file(img_path, width, height)

    pub = rospy.Publisher('depth_publisher', Image, queue_size=10)
    
    bridge = CvBridge()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            depth_image_message = bridge.cv2_to_imgmsg(image_data, encoding="32FC1")
            pub.publish(depth_image_message)
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    main()
