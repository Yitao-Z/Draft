#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray

def array_publisher():

    # Initiate a Node named 'array_publisher'
    rospy.init_node('array_publisher')

    # Create a Publisher object, that will publish on the 'array' topic 
    # messages of type Int32MultiArray
    pub = rospy.Publisher('array', Int32MultiArray, queue_size=10)

    # set the loop rate
    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():

        # Create a Int32MultiArray message
        array_msg = Int32MultiArray()
        array_msg.data = [1, 2, 3, 4, 5]

        # Publish the array
        pub.publish(array_msg)

        # Sleep for a while before publishing new messages
        rate.sleep()

if __name__ == '__main__':
    try:
        array_publisher()
    except rospy.ROSInterruptException:
        pass