#!/usr/bin/env python

import rospy
from custom_msgs.msg import custom

def talker():
    # Initialize the ROS node
    rospy.init_node('my_talker', anonymous=True)

    # Create a publisher that sends messages on the 'my_topic' topic using the custom message type
    pub = rospy.Publisher('my_topic', custom, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10) # 10 Hz

    # Loop until the node is shutdown
    while not rospy.is_shutdown():
        # Create a new message using the custom message type
        msg = custom()

        # Set the message fields
        msg.robot_name = "My Robot"
        msg.date_time = "2022-03-15"

        # Set the Pose2D message fields
        msg.location.x = 1.0
        msg.location.y = 2.0
        msg.location.theta = 3.0

        # Set the integer arrays
        msg.x = [1, 2, 3, 4, 5]
        msg.y = [6, 7, 8, 9, 10]
        msg.theta = [11, 12, 13, 14, 15]

        # Publish the message
        pub.publish(msg)
        rospy.loginfo(msg)

        # Sleep for the remainder of the loop period to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



