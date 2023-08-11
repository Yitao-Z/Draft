#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int32MultiArray

def array_callback(msg):
    # Print the received message
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)

def array_subscriber():

    # Initiate a Node named 'array_subscriber'
    rospy.init_node('array_subscriber')

    # Create a Subscriber object that will listen to the 'array' topic and 
    # will call the 'array_callback' function each time it reads something from the topic
    rospy.Subscriber('array', Int32MultiArray, array_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    array_subscriber()

## 
